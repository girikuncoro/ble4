#include "hal_uart.h"
#include "serialInterface.h"
#include "peripheral.h"
#include "simpleBLEperipheral.h"
#include "hal_led.h"
#include "gap.h"
#include "simpleGATTprofile.h"
#include <string.h>


static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg );

static uint8 serialInterface_TaskID;   // Task ID for internal task/event processing
static BLEPacket_t  rxSerialPkt;
static SerialEventPacket_t txSerialPkt;
uint8 adv_enable;

void SerialInterface_Init( uint8 task_id )
{
  serialInterface_TaskID = task_id;

  NPI_InitTransport(cSerialPacketParser);

  //Send command over UART to verify initiation.
  //Note that these are overun by Application on Start-up

}

uint16 SerialInterface_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( serialInterface_TaskID )) != NULL )
    {
      SerialInterface_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SI_CMD_RX)
  {

    parseCmd();

    return ( events ^ SI_CMD_RX);
  }

  if ( events & SI_EVT_TX)
  {

    //sendSerialEvt();

    return ( events ^ SI_EVT_TX);
  }

  // Discard unknown events
  return 0;
}

static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  default:
    // do nothing
    break;
  }
}

void cSerialPacketParser( uint8 port, uint8 events )
{

  (void)port;
  static npi_serial_parse_state_t  pktState = NPI_SERIAL_STATE_ID;
  uint8           done = FALSE;
  uint16          numBytes;
  static uint8    cmd_identifier;
  static uint8    cmd_opcode;
  static uint8   cmd_len;

  if (events & HAL_UART_RX_TIMEOUT)
  {
    // get the number of available bytes to process
    numBytes = NPI_RxBufLen();

    // check if there's any serial port data to process
    while ( (numBytes > 0) && (!done) )
    {
      // process serial port bytes to build the command or data packet
      switch( pktState )
      {

      case NPI_SERIAL_STATE_ID:
        {
          (void)NPI_ReadTransport((uint8 *)&cmd_identifier, 1);
          // decrement the number of available bytes
          numBytes -= 1;

          if(cmd_identifier != SERIAL_IDENTIFIER)
          {
            // illegal packet type
            return;
          }
          rxSerialPkt.header.identifier = cmd_identifier;
          pktState = NPI_SERIAL_STATE_OPCODE;
          break;
        }

      case NPI_SERIAL_STATE_OPCODE:
        {
          // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
          (void)NPI_ReadTransport((uint8 *)&cmd_opcode, 1);
          // decrement the number of available bytes
          numBytes -= 1;

          // set next state based on the type of packet
          switch( cmd_opcode )
          {
          case APP_CMD_ADVERTISE:
          case APP_CMD_CHANGE_ADV_PACKET:
            rxSerialPkt.header.opCode = cmd_opcode;
            pktState = NPI_SERIAL_STATE_LEN;
            break;
          default:
            // illegal packet type
            return;
          }
          break;
        }

      case NPI_SERIAL_STATE_LEN: // command length
        {
          if (numBytes < 1)
          {
            // not enough data to progress, so leave it in driver buffer
            done = TRUE;
            break;
          }
          // read the length
          // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
          (void)NPI_ReadTransport((uint8 *)&cmd_len, 1);
          rxSerialPkt.length = cmd_len;
          // decrement the number of available bytes
          numBytes -= 1;
          pktState = NPI_SERIAL_STATE_DATA;
          break;
        }

      case NPI_SERIAL_STATE_DATA:       // command payload
        {
          // check if there is enough serial port data to finish reading the payload
          if ( numBytes < cmd_len )
          {
            // not enough data to progress, so leave it in driver buffer
            done = TRUE;
            break;
          }
          (void) NPI_ReadTransport((uint8 *)rxSerialPkt.data, cmd_len);
           pktState = NPI_SERIAL_STATE_ID;
           done = TRUE;

          // Note. using OSAL messaging instead is more effective
          osal_set_event( serialInterface_TaskID, SI_CMD_RX );
        }
      }
    }

  }
  else
  {
    return;
  }

}

void parseCmd(void){

  uint8 opCode =  rxSerialPkt.header.opCode;

    switch (opCode) {
    case APP_CMD_ADVERTISE:
      adv_enable = TRUE;
      //Build Response Send Response
      txSerialPkt.header.identifier = rxSerialPkt.header.identifier;
      txSerialPkt.header.opCode = APP_EVT_CMD_RESPONSE;
      txSerialPkt.length = CMD_RESPONSE;
      txSerialPkt.cmdCode = opCode; //Command Code

      if (GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &adv_enable )==SUCCESS )
      {
        txSerialPkt.status = SUCCESS;
      }
      else
      {
        txSerialPkt.status = FAILURE;
      }

      //osal_set_event( serialInterface_TaskID, SI_EVT_TX );

      break;

    case APP_CMD_CHANGE_ADV_PACKET:

      //Build Response Send Response
      txSerialPkt.header.identifier = rxSerialPkt.header.identifier;
      txSerialPkt.header.opCode = APP_EVT_CMD_RESPONSE;
      txSerialPkt.length = CMD_RESPONSE;
      txSerialPkt.cmdCode = opCode; //Command Code
      uint8 adv_packet[31] = { 0x02,   // length of this data
                               GAP_ADTYPE_FLAGS,
                               GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
                               rxSerialPkt.data[0],
                               rxSerialPkt.data[1],
                               rxSerialPkt.data[2],
                               rxSerialPkt.data[3],
                               rxSerialPkt.data[4],
                               rxSerialPkt.data[5],
                               rxSerialPkt.data[6],
                               rxSerialPkt.data[7],
                               rxSerialPkt.data[8],
                               rxSerialPkt.data[9],
                               rxSerialPkt.data[10],
                               rxSerialPkt.data[11],
                               rxSerialPkt.data[12],
                               rxSerialPkt.data[13],
                               rxSerialPkt.data[14],
                               rxSerialPkt.data[15],
                               rxSerialPkt.data[16],
                               rxSerialPkt.data[17],
                               rxSerialPkt.data[18],
                               rxSerialPkt.data[19],
                               rxSerialPkt.data[20],
                               rxSerialPkt.data[22],
                               rxSerialPkt.data[23],
                               rxSerialPkt.data[24],
                               rxSerialPkt.data[25],
                               rxSerialPkt.data[26],
                               rxSerialPkt.data[27]
      };

      if (GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( adv_packet ), adv_packet )==SUCCESS)
      {
        txSerialPkt.status = SUCCESS;
      }
      else
      {
        txSerialPkt.status = FAILURE;
      }


      //osal_set_event( serialInterface_TaskID, SI_EVT_TX );

      break;

    default:

      txSerialPkt.header.identifier = 0xFF;
      txSerialPkt.header.opCode = 0xFF;
      txSerialPkt.length = 0xFF;
      txSerialPkt.cmdCode = 0xFF;
      txSerialPkt.status = FAILURE;

      //osal_set_event( serialInterface_TaskID, SI_EVT_TX );

      break;

    }

}

/**
void sendSerialEvt(void){

  uint8 opCode =  txSerialPkt.header.opCode;
  switch (opCode) {

  case APP_EVT_CMD_RESPONSE:
    HalUARTWrite(HAL_UART_PORT_0, (uint8*)&txSerialPkt, sizeof(txSerialPkt));
    break;


  }
}
**/