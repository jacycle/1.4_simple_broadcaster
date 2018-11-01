#ifndef SERIAL_GPIO_H
#define SERIAL_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

#define LED_RED              Board_LED0
#define LED_GREEN            Board_LED1
#define LED_BLUE             Board_LED2
  
#define LedOn(pin)           HwGPIOSet(pin, 1)   
#define LedOff(pin)          HwGPIOSet(pin, 0)        
#define LedToggle(pin)       HwGPIOToggle(pin)
  
/*********************************************************************
 * GPIO��ʼ������
 */
void HwGPIOInit(void);

/*********************************************************************
 * ����GPIO��ƽ
 */
void HwGPIOSet(uint32_t pin, uint8_t flag);

/*********************************************************************
 * ȡ��GPIO��ƽ
 */
void HwGPIOToggle(uint32_t pin);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_GPIO_H */
