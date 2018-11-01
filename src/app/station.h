#ifndef __STATION_H__
#define __STATION_H__

#ifdef __cplusplus
extern "C"
{
#endif
  
#define A1_MODE         0
#define A2_MODE         1
  
void station_mode_set(uint8_t mode);
uint8_t station_mode_get(void);
void station_access_request(uint16_t devid);
int station_recv_data(uint8_t **pbuf, uint16_t *len);
  
void station_init(void);

#endif
  