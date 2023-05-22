#ifndef DRIVER_CAN
#define DRIVER_CAN

/***********************************************************************/
/*Include*/ 
/***********************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxMultican_Can.h"
#include "IfxPort.h"
#include "Bsp.h"


/***********************************************************************/
/*Define*/ 
/***********************************************************************/

#define STB         &MODULE_P20,6                               /* set STB Pin */

#define TX_INTERRUPT_SRC_ID         IfxMultican_SrcId_0         /* Transmit interrupt service request ID             */
#define ISR_PRIORITY_CAN_TX         2                           /* Define the CAN TX interrupt priority              */
#define PIN5                        5                           /* LED1 used in TX ISR is connected to this pin      */
#define PIN6                        6                           /* LED2 used in RX ISR is connected to this pin      */


#define WAIT_TIME_1ms               1                           /* set wait time 1ms */
#define WAIT_TIME_5ms               5                           /* set wait time 5ms */
#define WAIT_TIME_10ms              10                          /* set wait time 10ms */
#define WAIT_TIME_20ms              20                          /* set wait time 20ms */
#define WAIT_TIME_50ms              50                          /* set wait time 50ms */
#define WAIT_TIME_100ms             100                         /* set wait time 100ms */

#define LED1         &MODULE_P00,5                              /* set LED1 Pin */
#define LED2         &MODULE_P00,6                              /* set LED2 Pin */
/***********************************************************************/
/*Typedef*/ 
/***********************************************************************/
typedef struct                              /* App_MulticanBasic struct */
{
    struct
    {
        IfxMultican_Can        can;          /**< \brief CAN driver handle */
        IfxMultican_Can_Node   canSrcNode;   /**< \brief CAN Source Node */
        IfxMultican_Can_MsgObj canSrcMsgObj; /**< \brief CAN Source Message object */
    }drivers;
} App_MulticanBasic;


typedef struct                              /* CanRxMsg struct */
{
    unsigned long ID;                       /*CanRxMsg ID*/
    unsigned char IDE;                      /*CanRxMsg IDE (Identifier Extension)*/
    unsigned char DLC;                      /*CanRxMsg DLC (Data Length Code)*/
    unsigned char Data[8];                  /*CanRxMsg Data*/
} CanRxMsg;

typedef struct
{
    IfxPort_Pin_Config              led1;                  /* LED1 configuration structure                           */
    IfxPort_Pin_Config              led2;                  /* LED2 configuration structure                           */
} AppLedType;


/***********************************************************************/
/*External Variable*/ 
/***********************************************************************/
IFX_EXTERN App_MulticanBasic g_MulticanBasic;


/***********************************************************************/
/*Global Function Prototype*/ 
/***********************************************************************/
extern void Driver_Can_Init(void);
extern void Driver_Can_TxTest(void);
extern void CAN_send(CanRxMsg *message);
extern void CAN_TEST(void);
void initLed(void);
void blinkLED1(void);
void blinkLED2(void);
#endif /* DRIVER_STM */
