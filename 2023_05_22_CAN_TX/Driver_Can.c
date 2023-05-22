/***********************************************************************/
/*Include*/ 
/***********************************************************************/
#include "Driver_Can.h"                                                 /*Drive_Can.h  include*/


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

IFX_INTERRUPT(canIsrTxHandler, 0, ISR_PRIORITY_CAN_TX);                         /*set interrupt service routine*/

/***********************************************************************/
/*Variable*/ 
/***********************************************************************/
App_MulticanBasic g_MulticanBasic; /**< \brief Demo information */

volatile CanRxMsg rec;
int a[8]={0,0x1,0x2,0x3,0x4,0x5,0x6,0x7};
 
/***********************************************************************/
/*Function*/ 
/***********************************************************************/



void Driver_Can_Init(void)                                  /*driver can init*/
{
    /* create module config */
    IfxMultican_Can_Config canConfig;                               /*can struct create*/
    IfxMultican_Can_initModuleConfig(&canConfig, &MODULE_CAN);      /*can struct init*/

    /* Interrupt configuration*/

    canConfig.nodePointer[TX_INTERRUPT_SRC_ID].priority = ISR_PRIORITY_CAN_TX;          /*can TX interrupt priority set*/

    /* initialize module */
    IfxMultican_Can_initModule(&g_MulticanBasic.drivers.can, &canConfig);               /*can module init*/

    /* create CAN node config */
    IfxMultican_Can_NodeConfig canNodeConfig;                                           /*can Node init*/
    IfxMultican_Can_Node_initConfig(&canNodeConfig, &g_MulticanBasic.drivers.can);      /*use function Can_Node_init*/

    canNodeConfig.baudrate = 500000UL;                                                                      // CAN 속도 설정 500kbps
    {
        canNodeConfig.nodeId    = IfxMultican_NodeId_0;//(IfxMultican_NodeId)((int)IfxMultican_NodeId_0);   // CAN의 Node ID 설정 0번으로 설정함
        canNodeConfig.rxPin     = &IfxMultican_RXD0B_P20_7_IN;                                              // 입력핀 설정
        canNodeConfig.rxPinMode = IfxPort_InputMode_pullUp;                                                 //set rxPin pullup
        canNodeConfig.txPin     = &IfxMultican_TXD0_P20_8_OUT;                                              // 출력핀 설정
        canNodeConfig.txPinMode = IfxPort_OutputMode_pushPull;                                              //set txPin pushpull

        IfxMultican_Can_Node_init(&g_MulticanBasic.drivers.canSrcNode, &canNodeConfig);                     // CAN node 초기화
    }

    /* Create message object config */
    IfxMultican_Can_MsgObjConfig canMsgObjConfig;                                                           // CAN message object configuration
    IfxMultican_Can_MsgObj_initConfig(&canMsgObjConfig, &g_MulticanBasic.drivers.canSrcNode);               //Can msg object init

    canMsgObjConfig.msgObjId              = 0;                                                              //256개의 message object가 있음
    canMsgObjConfig.messageId             = 0x100;                                                          //set messageId = 0x100
    canMsgObjConfig.acceptanceMask        = 0x7FFFFFFFUL;                                                   //set acceptanceMask (값이 일치하면 메세지 수신)
    canMsgObjConfig.frame                 = IfxMultican_Frame_transmit;                                     // CAN TX로 설정
    canMsgObjConfig.control.messageLen    = IfxMultican_DataLengthCode_8;                                   // Data 길이는 8
    canMsgObjConfig.control.extendedFrame = FALSE;                                                          // Extended ID 사용 안함
    canMsgObjConfig.control.matchingId    = TRUE;                                                           //ID 일치 확인

    canMsgObjConfig.txInterrupt.enabled = TRUE;                                                             //전송 인터럽트 활성화
    canMsgObjConfig.txInterrupt.srcId = TX_INTERRUPT_SRC_ID;                                                //전송 인터럽트 소스 ID 변경

    /* initialize message object */
    IfxMultican_Can_MsgObj_init(&g_MulticanBasic.drivers.canSrcMsgObj, &canMsgObjConfig);                   //can msg init

    /* IO Port */
    IfxPort_setPinModeOutput(STB, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);                  //출력 모드를 설정
    /* Set STB Pin of CAN chip (low-level active) */
    IfxPort_setPinHigh(STB);                                                                                //STB set Pin High
}


void Driver_Can_TxTest(void)                                //driver can TX test
{
    const uint32 dataLow  = 0x12340000;                     //TX data 하위 32bit
    const uint32 dataHigh = 0x9abc0000;                     //TX data 상위 332bit

    /* Transmit Data */
    {
        IfxMultican_Message msg;                            //can 메세지 객체 생성
        IfxMultican_Message_init(&msg, 0x100, dataLow, dataHigh, IfxMultican_DataLengthCode_8);             //init msg

        while (IfxMultican_Can_MsgObj_sendMessage(&g_MulticanBasic.drivers.canSrcMsgObj, &msg) == IfxMultican_Status_notSentBusy)           //메세지 객체 송신 후 완료까지 대기
        {}
    }
}


void CAN_send(CanRxMsg *message)            //TX msg send
{


    IfxMultican_Message msg;                //can TX 메세지 객체 생성

    //수신받은 RX msg를 이용해 dataLow, dataHigh 생성
    const unsigned dataLow = message->Data[0]|(message->Data[1]<<8)|(message->Data[2]<<16)|(message->Data[3]<<24);
    const unsigned dataHigh = message->Data[4]|(message->Data[5]<<8)|(message->Data[6]<<16)|(message->Data[7]<<24);


    IfxMultican_Message_init(&msg,message->ID,dataLow,dataHigh,message->DLC);           //CAN msg 초기화

    while (IfxMultican_Can_MsgObj_sendMessage(&g_MulticanBasic.drivers.canSrcMsgObj, &msg) == IfxMultican_Status_notSentBusy)       //메세지 객체 송신 후 완료까지 대기
    {

    }
}


void CAN_TEST(void)             //CAN Test를 위한 함수
{
    CanRxMsg MES;               //msg 생성
    int i=0;
    MES.ID=0x890;               //ID = 0x890
    MES.IDE=0;                  //IDE(Identifier Extension)(확장자) = 0
    MES.DLC=8;                  //DLC(Data Length Code) = 8
    for(i=0; i<8; i++)          //MES.Data에 a[i] 저장
    {
        MES.Data[i]=a[i];
    }
    CAN_send(&MES);             //can send
}


/* Interrupt Service Routine (ISR) called once the TX interrupt has been generated.
 * Turns on the LED1 to indicate successful CAN message transmission.
 */
void canIsrTxHandler(void)          //인터럽트 서비스 루틴으로 호출될 때 blinkLED1() 실행
{
    /* Just to indicate that the CAN message has been transmitted by turning on LED1 */
    blinkLED1();

    //IfxPort_setPinLow(g_led.led1.port, g_led.led1.pinIndex);
}


void initLed(void)              //LED init
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
    g_led.led1.port      = &MODULE_P00;                             //LED1 GPIO 포트          GPIO(General Purpose Input/Output)
    g_led.led1.pinIndex  = PIN5;                                    //LED1의 핀번호 05
    g_led.led1.mode      = IfxPort_OutputIdx_general;               //GPIO핀 사용
    g_led.led1.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;  //LED1 패드 드라이버 강도 설정        //패드 드라이버 : 핀의 전기적 특성을 제어

    g_led.led2.port      = &MODULE_P00;                             //LED2 GPIO포트
    g_led.led2.pinIndex  = PIN6;                                    //LED2의 핀번호 06
    g_led.led2.mode      = IfxPort_OutputIdx_general;               //GPIO핀 사용
    g_led.led2.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;  //LED2 패드 드라이버 강도 설정

    /* Initialize the pins connected to LEDs to level "HIGH"; will keep the LEDs turned off as default state */
    IfxPort_setPinHigh(g_led.led1.port, g_led.led1.pinIndex);       //LED1 꺼진 상태로 초기화
    IfxPort_setPinHigh(g_led.led2.port, g_led.led2.pinIndex);       //LED2 꺼진 상태로 초기화

    /* Set the pin input/output mode for both pins connected to the LEDs */
    IfxPort_setPinModeOutput(g_led.led1.port, g_led.led1.pinIndex, IfxPort_OutputMode_pushPull, g_led.led1.mode);       //LED1핀 출력모드(Output Mode)로 설정
    IfxPort_setPinModeOutput(g_led.led2.port, g_led.led2.pinIndex, IfxPort_OutputMode_pushPull, g_led.led2.mode);       //LED2핀 출력모드(Output Mode)로 설정

    /* Set the pad driver mode for both pins connected to the LEDs */
    IfxPort_setPinPadDriver(g_led.led1.port, g_led.led1.pinIndex, g_led.led1.padDriver);                                //LED1핀 패드 드라이버 설정
    IfxPort_setPinPadDriver(g_led.led2.port, g_led.led2.pinIndex, g_led.led2.padDriver);                                //LED2핀 패드 드라이버 설정
}


void blinkLED1(void)
{
    //IfxPort_togglePin(LED1);                                                     /* Toggle the state of the LED      */

    IfxPort_setPinHigh(LED1);                               //LED1 High(켜짐)
    waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME_1ms));    /* Wait 1 milliseconds            */
    IfxPort_setPinLow(LED1);                                //LED1 Low(꺼짐)
}       //껏다켰다 반복하여 깜빡임
void blinkLED2(void)
{
    IfxPort_togglePin(LED2);                                                     /* Toggle the state of the LED      */
    waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME_10ms));    /* Wait 10 milliseconds            */
}
