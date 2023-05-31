/***********************************************************************/
/*Include*/ 
/***********************************************************************/
#include "Driver_Can.h"


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
AppLedType        g_led;                                    /* Global LED configuration and control structure        */


/***********************************************************************/
/*Define*/ 
/***********************************************************************/


/***********************************************************************/
/*Typedef*/ 
/***********************************************************************/


/***********************************************************************/
/*Static Function Prototype*/ 
/***********************************************************************/


/* Macro to define Interrupt Service Routine.*/

IFX_INTERRUPT(canIsrTxHandler, 0, ISR_PRIORITY_CAN_TX);                                           // CAN TX interrupt
IFX_INTERRUPT(canIsrRxHandler, 0, ISR_PRIORITY_CAN_RX);                                           // CAN RX interrupt
//////////////////////////////////////// CAN RX interrupt 부분 추가


/***********************************************************************/
/*Variable*/ 
/***********************************************************************/
App_MulticanBasic g_MulticanBasic; /**< \brief Demo information */

volatile CanRxMsg rec;
int a[8]={0,0x1,0x2,0x3,0x4,0x5,0x6,0x7};

/***********************************************************************/
/*Function*/ 
/***********************************************************************/

void Driver_Can_Init(void)
{
    /* create module config */

    /* Interrupt configuration*/
    IfxMultican_Can_initModuleConfig(&g_MulticanBasic.drivers.canConfig, &MODULE_CAN);
    g_MulticanBasic.drivers.canConfig.nodePointer[TX_INTERRUPT_SRC_ID].priority = ISR_PRIORITY_CAN_TX;
    g_MulticanBasic.drivers.canConfig.nodePointer[RX_INTERRUPT_SRC_ID].priority = ISR_PRIORITY_CAN_RX;
    /* initialize module */
    IfxMultican_Can_initModule(&g_MulticanBasic.drivers.can, &g_MulticanBasic.drivers.canConfig);



    /* create CAN node config */
    //IfxMultican_Can_NodeConfig canNodeConfig;
    IfxMultican_Can_Node_initConfig(&g_MulticanBasic.drivers.canNodeConfig, &g_MulticanBasic.drivers.can);

    g_MulticanBasic.drivers.canNodeConfig.baudrate = 500000UL;                                                              // CAN 속도 설정 500kbps
    g_MulticanBasic.drivers.canNodeConfig.nodeId    = (IfxMultican_NodeId)((int)IfxMultican_NodeId_0);                      // CAN의 TX Node ID 설정 0번으로 설정함
    g_MulticanBasic.drivers.canNodeConfig.rxPin     = &IfxMultican_RXD0B_P20_7_IN;                                          // RX핀 설정
    g_MulticanBasic.drivers.canNodeConfig.rxPinMode = IfxPort_InputMode_pullUp;
    g_MulticanBasic.drivers.canNodeConfig.txPin     = &IfxMultican_TXD0_P20_8_OUT;                                          // TX핀 설정
    g_MulticanBasic.drivers.canNodeConfig.txPinMode = IfxPort_OutputMode_pushPull;

    IfxMultican_Can_Node_init(&g_MulticanBasic.drivers.canSrcNode, &g_MulticanBasic.drivers.canNodeConfig);                // CAN node 초기화



    IfxMultican_Can_Node_initConfig(&g_MulticanBasic.drivers.canNodeConfig, &g_MulticanBasic.drivers.can);
    g_MulticanBasic.drivers.canNodeConfig.baudrate = 500000UL;                                                             // CAN 속도 설정 500kbps
    g_MulticanBasic.drivers.canNodeConfig.nodeId    = (IfxMultican_NodeId)((int)IfxMultican_NodeId_0);                     // CAN의 RX Node ID 설정 0번으로 설정함
    g_MulticanBasic.drivers.canNodeConfig.rxPin     = &IfxMultican_RXD0B_P20_7_IN;                                          // RX핀 설정
    g_MulticanBasic.drivers.canNodeConfig.rxPinMode = IfxPort_InputMode_pullUp;
    g_MulticanBasic.drivers.canNodeConfig.txPin     = &IfxMultican_TXD0_P20_8_OUT;                                          // TX핀 설정
    g_MulticanBasic.drivers.canNodeConfig.txPinMode = IfxPort_OutputMode_pushPull;


    IfxMultican_Can_Node_init(&g_MulticanBasic.drivers.canDstNode, &g_MulticanBasic.drivers.canNodeConfig);                // CAN RX node 초기화


    /* Create message object config for TX */

    IfxMultican_Can_MsgObj_initConfig(&g_MulticanBasic.drivers.canMsgObjConfig, &g_MulticanBasic.drivers.canSrcNode);

    g_MulticanBasic.drivers.canMsgObjConfig.msgObjId              = (IfxMultican_MsgObjId)0;                               // 256개의 message object가 있음
    g_MulticanBasic.drivers.canMsgObjConfig.messageId             = 0x90;
    g_MulticanBasic.drivers.canMsgObjConfig.acceptanceMask        = 0x7FFFFFFFUL;
    g_MulticanBasic.drivers.canMsgObjConfig.frame                 = IfxMultican_Frame_transmit;                            // CAN TX로 설정
    g_MulticanBasic.drivers.canMsgObjConfig.control.messageLen    = IfxMultican_DataLengthCode_8;                          // Data 길이는 8
    g_MulticanBasic.drivers.canMsgObjConfig.control.extendedFrame = FALSE;                                                 // Extended ID 사용 안함
    g_MulticanBasic.drivers.canMsgObjConfig.control.matchingId    = TRUE;

    g_MulticanBasic.drivers.canMsgObjConfig.txInterrupt.enabled   = TRUE;
    g_MulticanBasic.drivers.canMsgObjConfig.txInterrupt.srcId     = TX_INTERRUPT_SRC_ID;

    /* initialize message object */
    IfxMultican_Can_MsgObj_init(&g_MulticanBasic.drivers.canSrcMsgObj, &g_MulticanBasic.drivers.canMsgObjConfig);



    /* Create message object config for RX  추가된 부분이며 주소는 91 */
    IfxMultican_Can_MsgObj_initConfig(&g_MulticanBasic.drivers.canMsgObjConfig, &g_MulticanBasic.drivers.canDstNode);

    g_MulticanBasic.drivers.canMsgObjConfig.msgObjId              = (IfxMultican_MsgObjId)1;
    g_MulticanBasic.drivers.canMsgObjConfig.messageId             = 0x91;
    g_MulticanBasic.drivers.canMsgObjConfig.acceptanceMask        = 0x7FFFFFFFUL;
    g_MulticanBasic.drivers.canMsgObjConfig.frame                 = IfxMultican_Frame_receive;
    g_MulticanBasic.drivers.canMsgObjConfig.control.messageLen    = IfxMultican_DataLengthCode_8;                                   // Data 길이는 8
    g_MulticanBasic.drivers.canMsgObjConfig.control.extendedFrame = FALSE;                                                          // Extended ID 사용 안함
    g_MulticanBasic.drivers.canMsgObjConfig.control.matchingId    = TRUE;

    g_MulticanBasic.drivers.canMsgObjConfig.rxInterrupt.enabled   = TRUE;
    g_MulticanBasic.drivers.canMsgObjConfig.rxInterrupt.srcId     = RX_INTERRUPT_SRC_ID;

    /*
    SRC_CAN_CAN0_INT0.B.SRPN = 30u;                                                                        //Register의 역활은?
    SRC_CAN_CAN0_INT0.B.TOS  = 0u;
    SRC_CAN_CAN0_INT0.B.SRE  = 1u;
    */
    IfxMultican_Can_MsgObj_init(&g_MulticanBasic.drivers.canDstMsgObj, &g_MulticanBasic.drivers.canMsgObjConfig);


    //IfxCpu_enableInterrupts();
    /* IO Port */
    IfxPort_setPinModeOutput(STB, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    /* Set STB Pin of CAN chip (low-level active) */
    IfxPort_setPinLow(STB);
    IfxPort_setPinModeOutput(INT_RX_OUT, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinLow(INT_RX_OUT);
}


void Driver_Can_TxTest(void)
{
    const uint32 dataLow  = 0x12340000;
    const uint32 dataHigh = 0x9abc0000;

    /* Transmit Data */
    {
        IfxMultican_Message msg;
        IfxMultican_Message_init(&msg, 0x100, dataLow, dataHigh, IfxMultican_DataLengthCode_8);

        while (IfxMultican_Can_MsgObj_sendMessage(&g_MulticanBasic.drivers.canSrcMsgObj, &msg) == IfxMultican_Status_notSentBusy)
        {}
    }
}


void CAN_send(CanRxMsg *message)
{


    IfxMultican_Message msg;

    const unsigned dataLow = message->Data[0]|(message->Data[1]<<8)|(message->Data[2]<<16)|(message->Data[3]<<24);
    const unsigned dataHigh = message->Data[4]|(message->Data[5]<<8)|(message->Data[6]<<16)|(message->Data[7]<<24);


    IfxMultican_Message_init(&msg,message->ID,dataLow,dataHigh,message->DLC);

    while (IfxMultican_Can_MsgObj_sendMessage(&g_MulticanBasic.drivers.canSrcMsgObj, &msg) == IfxMultican_Status_notSentBusy)
    {

    }
}


void CAN_TEST(void)
{
    CanRxMsg MES;
    int i=0;
    MES.ID=0x90;
    MES.IDE=0;
    MES.DLC=8;
    for(i=0; i<8; i++)
    {
        MES.Data[i]=a[i];
    }
    CAN_send(&MES);
}


/* Interrupt Service Routine (ISR) called once the TX interrupt has been generated.
 * Turns on the LED1 to indicate successful CAN message transmission.
 */
void canIsrTxHandler(void)
{
    /* Just to indicate that the CAN message has been transmitted by turning on LED1 */
    blinkLED1();

    //IfxPort_setPinLow(g_led.led1.port, g_led.led1.pinIndex);
}



//추가된 부분. RX handler
void canIsrRxHandler(void)
{
    IfxMultican_Status readStatus;
    static uint32 u32nuTemp1 = 0u;
    static uint32 u32nuTemp2 = 0u;
    // 32bit 변수 생성

    //IfxCpu_enableInterrupts();
    /* Read the received CAN message and store the status of the operation 메세지 읽어오기*/
    readStatus = IfxMultican_Can_MsgObj_readMessage(&g_MulticanBasic.drivers.canDstMsgObj, &g_MulticanBasic.drivers.rxMsg);

    if( !( readStatus & IfxMultican_Status_newData ) )  //새로운 데이터가 버퍼에 도착하고 readStatus와 비트가 같은 경우 if문 진입
    {
           while(1)//while문 무한루프
           {
           }
    }

    /* If new data has been received but with one message lost, report an error */
    if( readStatus == IfxMultican_Status_newDataButOneLost )        //드라이버에서 한 개의 데이터가 손실되면 if문 진입
    {
         while(1)//while문 무한루프
         {
         }
    }



    if( readStatus == IfxMultican_Status_newData )  //새로운 데이터가 버퍼에 도착하면 if문 진입
    {

        //g_MulticanBasic.drivers.rxMsg.data[0];
        u32nuTemp1 = g_MulticanBasic.drivers.rxMsg.data[0];
        u32nuTemp2 = g_MulticanBasic.drivers.rxMsg.data[1];
        //변수에 rxMsg data 저장.

       // IfxPort_setPinHigh(INT_RX_OUT);
       // waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME_1ms));    /* Wait 1 milliseconds            */
       // IfxPort_setPinLow(INT_RX_OUT);
        blinkLED1();//blinkLED1() 함수 실행
    }


    /* Finally, check if the received data matches with the transmitted one */
    //IfxPort_setPinHigh(INT_RX_OUT);
    //waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME_10ms));    /* Wait 1 milliseconds            */
    //IfxPort_setPinLow(INT_RX_OUT);

    /* Turn on the LED2 to indicate correctness of the received message */

}

void initLed(void)
{
    /* ======================================================================
     * Configuration of the pins connected to the LEDs:
     * ======================================================================
     *  - define the GPIO port
     *  - define the GPIO pin that is the connected to the LED
     *  - define the general GPIO pin usage (no alternate function used)
     *  - define the pad driver strength
     * ======================================================================
     */
    g_led.led1.port      = &MODULE_P00;
    g_led.led1.pinIndex  = PIN5;
    g_led.led1.mode      = IfxPort_OutputIdx_general;
    g_led.led1.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;

    g_led.led2.port      = &MODULE_P00;
    g_led.led2.pinIndex  = PIN6;
    g_led.led2.mode      = IfxPort_OutputIdx_general;
    g_led.led2.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;

    /* Initialize the pins connected to LEDs to level "HIGH"; will keep the LEDs turned off as default state */
    IfxPort_setPinHigh(g_led.led1.port, g_led.led1.pinIndex);
    IfxPort_setPinHigh(g_led.led2.port, g_led.led2.pinIndex);

    /* Set the pin input/output mode for both pins connected to the LEDs */
    IfxPort_setPinModeOutput(g_led.led1.port, g_led.led1.pinIndex, IfxPort_OutputMode_pushPull, g_led.led1.mode);
    IfxPort_setPinModeOutput(g_led.led2.port, g_led.led2.pinIndex, IfxPort_OutputMode_pushPull, g_led.led2.mode);

    /* Set the pad driver mode for both pins connected to the LEDs */
    IfxPort_setPinPadDriver(g_led.led1.port, g_led.led1.pinIndex, g_led.led1.padDriver);
    IfxPort_setPinPadDriver(g_led.led2.port, g_led.led2.pinIndex, g_led.led2.padDriver);
}

void blinkLED1(void)
{
   IfxPort_setPinHigh(LED1);
   waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME_10ms));    /* Wait 1 milliseconds            */
   IfxPort_setPinLow(LED1);
}
void blinkLED2(void)
{
    IfxPort_togglePin(LED2);                                                     /* Toggle the state of the LED      */
    waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME_20ms));    /* Wait 500 milliseconds            */
}
