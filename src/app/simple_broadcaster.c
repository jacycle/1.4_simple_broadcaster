/******************************************************************************

 @file       simple_broadcaster.c

 @brief This file contains the Simple Broadcaster sample application for
        use with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2011-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_1_40_00_45
 Release Date: 2017-07-20 17:16:59
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "broadcaster.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC


#include "board.h"
#include "iotboard_key.h"
#include "simple_broadcaster.h"

#include "hw_gpio.h"
#include "hw_uart.h"
#include "hw_spi.h"
#include "hw_adc.h"

//sx127x
#include "platform.h"
#include "radio.h"
#include "spi.h"
#include "station.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          (230) //160
#define BEACON_FEATURE

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
    #define SBB_DISPLAY_TYPE Display_Type_LCD
  #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
    #define SBB_DISPLAY_TYPE Display_Type_UART
  #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
    #define SBB_DISPLAY_TYPE 0 // Option not supported
  #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#else // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #define SBB_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL

// Task configuration
#define SBB_TASK_PRIORITY                     1
#define RADIO_TASK_PRIORITY                   1

#ifndef SBB_TASK_STACK_SIZE
#define SBB_TASK_STACK_SIZE                   800 //660
#endif
#ifndef RADIO_TASK_STACK_SIZE
#define RADIO_TASK_STACK_SIZE                 644
#endif

#define SBB_STATE_CHANGE_EVT                  0x0001
#define SBB_KEY_CHANGE_EVT                    0x0002

// Internal Events for RTOS application
#define SBB_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBB_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBB_PERIODIC_EVT                      Event_Id_00
#define SBB_LOOP_EVT                          Event_Id_01

#define SBB_ALL_EVENTS                        (SBB_ICALL_EVT | \
                                               SBB_QUEUE_EVT | \
                                               SBB_PERIODIC_EVT | \
                                               SBB_LOOP_EVT)

// How often to perform periodic event (in msec)
#define SBB_PERIODIC_EVT_PERIOD               5000
#define KEY_EVENT_TIMEOUT                     60000
#define BAT_EVENT_TIMEOUT                     60000

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // Event header.
//  uint8_t *pData;  // event data
} sbbEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;
Watchdog_Handle hWDT;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct oneshotClock;

// Clock instances for loop periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;
static uint8_t bat_low_flag;
static uint8_t key_alarm_flag;
static uint16_t bat_low_timeout;
static uint16_t key_alarm_timeout;

// Task configuration
Task_Struct sbbTask;
Char sbbTaskStack[SBB_TASK_STACK_SIZE];
// Task configuration
Task_Struct radioTask;
Char radioTaskStack[RADIO_TASK_STACK_SIZE];

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x15,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S',
  'i',
  'm',
  'p',
  'l',
  'e',
  'B',
  'L',
  'E',
  'B',
  'r',
  'o',
  'a',
  'd',
  'c',
  'a',
  's',
  't',
  'e',
  'r',

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

#ifndef BEACON_FEATURE

  // three-byte broadcast of the data "1 2 3"
  0x04,   // length of this data including the data type byte
  GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
  0xaa,
  0,
  0

#else

  // 25 byte beacon advertisement data
  // Preamble: Company ID - 0x000D for TI, refer to https://www.bluetooth.org/en-us/specification/assigned-numbers/company-identifiers
  // Data type: Beacon (0x02)
  // Data length: 0x15
  // UUID: 00000000-0000-0000-0000-000000000000 (null beacon)
  // Major: 1 (0x0001)
  // Minor: 1 (0x0001)
  // Measured Power: -59 (0xc5)
  0x1A, // length of this data including the data type byte
  GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
  0x0D, // Company ID - Fixed
  0x00, // Company ID - Fixed
  0x02, // Data Type - Fixed
  0x15, // Data Length - Fixed
  0x00, // UUID - Variable based on different use cases/applications
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // Major
  0x01, // Major
  0x00, // Minor
  0x01, // Minor
  0xc5  // Power - The 2's complement of the calibrated Tx Power

#endif // !BEACON_FEATURE
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleBLEBroadcaster_init(void);
static void SimpleBLEBroadcaster_taskFxn(UArg a0, UArg a1);
static void radio_taskFxn(UArg a0, UArg a1);

static void SimpleBLEBroadcaster_handleKeys(uint8_t shift, uint8_t keys);
static void SimpleBLEBroadcaster_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLEBroadcaster_processAppMsg(sbbEvt_t *pMsg);
static void SimpleBLEBroadcaster_processStateChangeEvt(gaprole_States_t newState);

static void SimpleBLEBroadcaster_stateChangeCB(gaprole_States_t newState);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEBroadcaster_BroadcasterCBs =
{
  SimpleBLEBroadcaster_stateChangeCB   // Profile State Change Callbacks
};

static void watchdog_callback(UArg a0)
{
  Watchdog_clear(hWDT);
}

static void watchdog_init(void)
{
  int t;
  
  Watchdog_Params wp;
  Watchdog_Params_init(&wp);
  wp.callbackFxn    = watchdog_callback;
  wp.debugStallMode = Watchdog_DEBUG_STALL_ON;
  wp.resetMode      = Watchdog_RESET_ON;

  Watchdog_init();
  hWDT = Watchdog_open(CC2640R2_LAUNCHXL_WATCHDOG0, &wp);
  if (hWDT == NULL) 
  {
    /* Error opening Watchdog */
    while (1);
  }
  t = Watchdog_convertMsToTicks(hWDT, 1000);
  Watchdog_setReload(hWDT, t); // 1sec (WDT runs always at 48MHz/32)
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_createTask
 *
 * @brief   Task creation function for the Simple Broadcaster.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLEBroadcaster_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbbTaskStack;
  taskParams.stackSize = SBB_TASK_STACK_SIZE;
  taskParams.priority = SBB_TASK_PRIORITY;

  Task_construct(&sbbTask, SimpleBLEBroadcaster_taskFxn, &taskParams, NULL);
}

void radio_createTask(void)
{
  Task_Params taskParams;
  
  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = radioTaskStack;
  taskParams.stackSize =  RADIO_TASK_STACK_SIZE;
  taskParams.priority = RADIO_TASK_PRIORITY;

  Task_construct(&radioTask, radio_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEObserver_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLEBroadcaster_enqueueMsg(uint8_t event, uint8_t state)
{
  sbbEvt_t *pMsg;

  // Create dynamic pointer to message.
  if (pMsg = ICall_malloc(sizeof(sbbEvt_t)))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
 * @fn      SimpleBLEObserver_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys pressed
 *
 * @return  none
 */
void SimpleBLEBroadcaster_keyChangeHandler(uint8 keys)
{
  SimpleBLEBroadcaster_enqueueMsg(SBB_KEY_CHANGE_EVT, keys);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEBroadcaster_clockHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
}

static void SimpleBLEBroadcaster_loopHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_init
 *
 * @brief   Initialization function for the Simple Broadcaster App
 *          Task. This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_init(void)
{
	// ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Hard code the DB Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0x33, 0x33, 0x33, 0x33, 0x33, 0x33 };
  //HCI_EXT_SetBDADDRCmd(bdAddress);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

//  Board_initKeys(SimpleBLEBroadcaster_keyChangeHandler);
  
  // Create one-shot clocks for internal periodic events.
//  Util_constructClock(&oneshotClock, SimpleBLEBroadcaster_clockHandler,
//                      SBB_PERIODIC_EVT_PERIOD, 0, false, SBB_PERIODIC_EVT);
  
  Util_constructClock(&periodicClock, SimpleBLEBroadcaster_loopHandler,
                      SBB_PERIODIC_EVT_PERIOD, SBB_PERIODIC_EVT_PERIOD, true, SBB_LOOP_EVT);
    
  // Open LCD
  dispHandle = Display_open(SBB_DISPLAY_TYPE, NULL);

  // Setup the GAP Broadcaster Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t gapRole_AdvertOffTime = 0;

#ifndef BEACON_FEATURE
    uint8_t advType = GAP_ADTYPE_ADV_SCAN_IND; // use scannable undirected adv
#else
    uint8_t advType = GAP_ADTYPE_ADV_NONCONN_IND; // use non-connectable adv
#endif // !BEACON_FEATURE

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initial_advertising_enable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &gapRole_AdvertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof (scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType);
  }

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Start the Device
  VOID GAPRole_StartDevice(&simpleBLEBroadcaster_BroadcasterCBs);

  Display_print0(dispHandle, 0, 0, "BLE Broadcaster");
  
  HwGPIOInit();
//  HwGPIOSet(Board_RLED,1);

//  HwUARTInit();
//  HwUARTWrite("observer init\r\n",13);
//  SpiInit();
  
//  radio_createTask();
  watchdog_init();
}

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_processEvent
 *
 * @brief   Application task entry point for the Simple Broadcaster.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_taskFxn(UArg a0, UArg a1)
{
  uint16_t advValue = 0;
  uint32_t volValue;
  HwADCInit();
    
  // Initialize application
  SimpleBLEBroadcaster_init();

  // Application main loop
  for (;;)
  {
    // Get the ticks since startup
    uint32_t tickStart = Clock_getTicks();

    uint32_t events;

    events = Event_pend(syncEvent, Event_Id_NONE, SBB_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleBLEBroadcaster_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & SBB_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          sbbEvt_t *pMsg = (sbbEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            SimpleBLEBroadcaster_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
      if (events & SBB_LOOP_EVT)
      {
        if (key_alarm_flag)
        {
            key_alarm_flag = 0;
            key_alarm_timeout = KEY_EVENT_TIMEOUT;
        }
        else
        {
            if (key_alarm_timeout > SBB_PERIODIC_EVT_PERIOD)
            {
                key_alarm_timeout -= SBB_PERIODIC_EVT_PERIOD;
            }
            else if (key_alarm_timeout > 0)
            {
                key_alarm_timeout = 0;
#ifndef BEACON_FEATURE
                advertData[sizeof(advertData) - 1] = 0;
#else
                advertData[sizeof(advertData) - 3] = 0;
#endif
                GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
            }
        }
        advValue = HwADCRead();
        volValue = (advValue * 430) / 4095;
        if (volValue <= 220)
            bat_low_flag = 1;
        else
            bat_low_flag = 0;
        Display_print1(dispHandle, 1, 0, "advValue:%d",advValue);
        Display_print1(dispHandle, 1, 0, "volValue:%d",volValue);
        if (bat_low_flag)
        {
            bat_low_timeout = BAT_EVENT_TIMEOUT;
#ifndef BEACON_FEATURE
            advertData[sizeof(advertData) - 2] = 1;
#else
            advertData[sizeof(advertData) - 5] = 1;
#endif
            GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
        }
        else
        {
            if (bat_low_timeout > SBB_PERIODIC_EVT_PERIOD)
            {
                bat_low_timeout -= SBB_PERIODIC_EVT_PERIOD;
            }
            else if (bat_low_timeout > 0)
            {
                bat_low_timeout = 0;
#ifndef BEACON_FEATURE
                advertData[sizeof(advertData) - 2] = 0;
#else
                advertData[sizeof(advertData) - 5] = 0;
#endif
                GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
            }
        }
      }
      if (events & SBB_PERIODIC_EVT)
      {
        Display_print0(dispHandle, 0, 0, "SBB_PERIODIC_EVT");
        uint8_t turnOnAdv = FALSE;
#ifndef BEACON_FEATURE
//        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED,sizeof(uint8_t), &turnOnAdv);
#endif
      }
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    default:
      // do nothing
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_processAppMsg(sbbEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBB_STATE_CHANGE_EVT:
      SimpleBLEBroadcaster_processStateChangeEvt((gaprole_States_t)pMsg->
                                                 hdr.state);
      break;
    case SBB_KEY_CHANGE_EVT:
      SimpleBLEBroadcaster_handleKeys(0, pMsg->hdr.state);
      break;
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_stateChangeCB(gaprole_States_t newState)
{
  sbbEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbbEvt_t))))
  {
    pMsg->hdr.event = SBB_STATE_CHANGE_EVT;
    pMsg->hdr.state = newState;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }
}

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_processStateChangeEvt
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_processStateChangeEvt(gaprole_States_t newState)
{
  switch (newState)
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // Display device address
        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, 2, 0, "Initialized");
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        Display_print0(dispHandle, 2, 0, "Advertising");
      }
      break;

    case GAPROLE_WAITING:
      {
        Display_print0(dispHandle, 2, 0, "Waiting");
      }
      break;

    case GAPROLE_ERROR:
      {
        Display_print0(dispHandle, 2, 0, "Error");
      }
      break;

    default:
      {
        Display_clearLine(dispHandle, 2);
      }
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_handleKeys(uint8 shift, uint8 keys)
{
  uint16_t advValue = 0;
  static int key1_counter;
  (void)shift;  // Intentionally unreferenced parameter

  // Right key takes the actio the user has selected.
  if (keys & KEY_BTN1)
  {
    key1_counter++;
    if (key1_counter == 10)
    {
        Display_print0(dispHandle, 2, 0, "key event...");
        uint8_t turnOnAdv = TRUE;     
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED,sizeof(uint8_t), &turnOnAdv);
#ifndef BEACON_FEATURE        
        advertData[sizeof(advertData) - 1] = 1;
#else
        advertData[sizeof(advertData) - 3] = 1;
#endif
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
        Util_startClock(&oneshotClock);
        key_alarm_flag = 1;
    }
  }
  else
  {
    key1_counter = 0;
  }
}

/*********************************************************************
 * @fn      radio_taskFxn
 *
 */

static void radio_taskFxn(UArg a0, UArg a1)
{
//    int ret;
//    uint16_t rcvlen;
//    uint8_t *pbuf;
    
//    station_init();
//    uint16_t advValue = 0;
//    HwADCInit();
    
    while(1)
    {
//        ret = station_recv_data(&pbuf, &rcvlen);
        Task_sleep(5000 * (1000 / Clock_tickPeriod));
    }
}

/*********************************************************************
*********************************************************************/
