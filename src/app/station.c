#include <string.h>
#include <ti/display/Display.h>
#include "observer.h"
#include "board.h"
#include "station.h"
#include "crc.h"
#include "enc.h"
#include "radio.h"
#include "hw_gpio.h"

#define BUFFER_SIZE     256                          // Define the payload size here

static uint16_t next_time;
static uint16_t radio_freq;

static tRadioDriver *Radio = NULL;
static uint16_t BufferSize = BUFFER_SIZE;			// RF buffer size
static uint8_t  Buffer[BUFFER_SIZE];				// RF buffer
static uint8_t EnableMaster = true; 				// Master/Slave selection


static const uint8_t PingMsg[] = "PING";
static const uint8_t PongMsg[] = "PONG";
static const uint8_t MY_TEST_Msg[] = "LoRa_SX1278_TEST";
static uint8_t station_mode;


extern Display_Handle dispHandle;

void OnMaster( void );
void OnSlave( void );

#define SX1278_RX
//#define SX1278_TX

void station_mode_set(uint8_t mode)
{
    station_mode = mode;
}

uint8_t station_mode_get(void)
{
    return station_mode;
}

void station_init(void)
{
    Radio = RadioDriverInit( );
    
    Radio->Init( );
    
#if defined (SX1278_RX)
    Radio->StartRx( );   //RFLR_STATE_RX_INIT
#elif defined (SX1278_TX)
    Radio->SetTxPacket( MY_TEST_Msg, 17 );
#endif
}

void station_send_data(uint8_t *pbuf, uint8_t len)
{
    if(Radio->Process( ) == RF_TX_DONE)
    {
//                    HwUARTPrintf("RF_LoRa_TX_OK! \n");
        LedToggle( LED_BLUE );
//        Radio->SetTxPacket( MY_TEST_Msg, strlen(MY_TEST_Msg) );   //RFLR_STATE_TX_INIT
        //                    HwUARTPrintf("tick=%d\n", ClockP_getSystemTicks());
        Radio->SetTxPacket(pbuf, len);  
        Display_print2(dispHandle, 0, 0, "tick=%d, len=%d\r\n", ClockP_getSystemTicks(), len);
    }	
}

void station_dump(uint8_t *pbuf, uint16_t len)
{
    char temp[50];
    int totlen;
    char *pstr;
    uint16_t i, j, k;
    int send_len;
    
    totlen = sizeof(temp);
    k = len / 16;
    for (j=0; j<k; j++)
    {
         pstr = temp;
         for (i=0; i<16; i++)
         {
            send_len = snprintf(pstr, totlen, "%02x ", pbuf[i]);
            pstr += send_len;
            totlen -= send_len;
         }
         Display_print0(dispHandle, 0, 0, temp);
    }
    len = len % 16;
    if (len)
    {
        pstr = temp;
        for (i=0; i<len; i++)
        {
            send_len = snprintf(pstr, totlen, "%02x ", pbuf[i]);
            pstr += send_len;
            totlen -= send_len;
        }
        Display_print0(dispHandle, 0, 0, temp);
    }
}

int station_recv_data(uint8_t **pbuf, uint16_t *len)
{
    if( Radio->Process( ) == RF_RX_DONE)
    {
        Radio->GetRxPacket( Buffer, ( uint16_t* )&BufferSize );
        Radio->StartRx( );
        Display_print1(dispHandle, 0, 0, "recvlen=%d", BufferSize);
        *len = BufferSize;
        *pbuf = Buffer;
        station_dump(Buffer, BufferSize);
        return BufferSize;
    }
    else
    {
        return 0;
    }
}


void station_access_request(uint16_t devid)
{
    uint8_t buffer[16];
    uint8_t index;
    uint8_t crc;
    
    index = 0;
    buffer[index++] = 0xaa;
    buffer[index++] = 0x55;
    buffer[index++] = 0xaa;
    buffer[index++] = 0x55;
    buffer[index++] = devid;       // device id lsb
    buffer[index++] = devid >> 8;  // device id hsb
    crc8((char *)buffer, index, (char *)&crc);
    buffer[index++]  = crc;
    station_send_data(buffer, index);
}

void station_upload(uint8_t left, uint16_t devid, uint8_t *pbuf, uint8_t len)
{
    uint8_t buffer[256];
    uint8_t index;
    uint8_t crc;
    uint8_t i;
    
    index = 0;
    buffer[index++] = 0xaa;
    buffer[index++] = 0x57;
    buffer[index++] = 0xaa;
    buffer[index++] = 0x57;
    buffer[index++] = left;
    buffer[index++] = devid;       // device id lsb
    buffer[index++] = devid >> 8;  // device id hsb
    buffer[index++] = len;
    for(i=0; i<len; i++)
    {
        buffer[index++] = pbuf[i];
    }
    crc8((char *)buffer, index, (char *)&crc);
    buffer[index++]  = crc;
}

int station_access_resp(uint8_t *pbuf, uint8_t len)
{
    next_time = (pbuf[1] << 8) + pbuf[0];
    radio_freq = (pbuf[2] << 8) + pbuf[3];
}

int station_upload_resp(uint8_t *pbuf, uint8_t len)
{
    uint16_t id;
    
    id = (pbuf[1] << 8) + pbuf[0];
    next_time = (pbuf[2] << 8) + pbuf[3];
}

int station_recv_handle(uint8_t *pbuf, uint8_t len)
{
    uint8_t crc = -1;
    
    if (len < 7 || pbuf == NULL)
    {
        return -1;
    }
    crc8(pbuf, len, &crc);
    if (crc != 0)
    {
        return -1;
    }
    if ((pbuf[0] == 0xbb) && (pbuf[1] == 0x56) &&
        (pbuf[2] == 0xbb) && (pbuf[3] == 0x56))
    {
           station_access_resp(&pbuf[4], len-5); 
    }
}