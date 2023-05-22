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

    canNodeConfig.baudrate = 500000UL;                                                                      // CAN �ӵ� ���� 500kbps
    {
        canNodeConfig.nodeId    = IfxMultican_NodeId_0;//(IfxMultican_NodeId)((int)IfxMultican_NodeId_0);   // CAN�� Node ID ���� 0������ ������
        canNodeConfig.rxPin     = &IfxMultican_RXD0B_P20_7_IN;                                              // �Է��� ����
        canNodeConfig.rxPinMode = IfxPort_InputMode_pullUp;                                                 //set rxPin pullup
        canNodeConfig.txPin     = &IfxMultican_TXD0_P20_8_OUT;                                              // ����� ����
        canNodeConfig.txPinMode = IfxPort_OutputMode_pushPull;                                              //set txPin pushpull

        IfxMultican_Can_Node_init(&g_MulticanBasic.drivers.canSrcNode, &canNodeConfig);                     // CAN node �ʱ�ȭ
    }

    /* Create message object config */
    IfxMultican_Can_MsgObjConfig canMsgObjConfig;                                                           // CAN message object configuration
    IfxMultican_Can_MsgObj_initConfig(&canMsgObjConfig, &g_MulticanBasic.drivers.canSrcNode);               //Can msg object init

    canMsgObjConfig.msgObjId              = 0;                                                              //256���� message object�� ����
    canMsgObjConfig.messageId             = 0x100;                                                          //set messageId = 0x100
    canMsgObjConfig.acceptanceMask        = 0x7FFFFFFFUL;                                                   //set acceptanceMask (���� ��ġ�ϸ� �޼��� ����)
    canMsgObjConfig.frame                 = IfxMultican_Frame_transmit;                                     // CAN TX�� ����
    canMsgObjConfig.control.messageLen    = IfxMultican_DataLengthCode_8;                                   // Data ���̴� 8
    canMsgObjConfig.control.extendedFrame = FALSE;                                                          // Extended ID ��� ����
    canMsgObjConfig.control.matchingId    = TRUE;                                                           //ID ��ġ Ȯ��

    canMsgObjConfig.txInterrupt.enabled = TRUE;                                                             //���� ���ͷ�Ʈ Ȱ��ȭ
    canMsgObjConfig.txInterrupt.srcId = TX_INTERRUPT_SRC_ID;                                                //���� ���ͷ�Ʈ �ҽ� ID ����

    /* initialize message object */
    IfxMultican_Can_MsgObj_init(&g_MulticanBasic.drivers.canSrcMsgObj, &canMsgObjConfig);                   //can msg init

    /* IO Port */
    IfxPort_setPinModeOutput(STB, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);                  //��� ��带 ����
    /* Set STB Pin of CAN chip (low-level active) */
    IfxPort_setPinHigh(STB);                                                                                //STB set Pin High
}


void Driver_Can_TxTest(void)                                //driver can TX test
{
    const uint32 dataLow  = 0x12340000;                     //TX data ���� 32bit
    const uint32 dataHigh = 0x9abc0000;                     //TX data ���� 332bit

    /* Transmit Data */
    {
        IfxMultican_Message msg;                            //can �޼��� ��ü ����
        IfxMultican_Message_init(&msg, 0x100, dataLow, dataHigh, IfxMultican_DataLengthCode_8);             //init msg

        while (IfxMultican_Can_MsgObj_sendMessage(&g_MulticanBasic.drivers.canSrcMsgObj, &msg) == IfxMultican_Status_notSentBusy)           //�޼��� ��ü �۽� �� �Ϸ���� ���
        {}
    }
}


void CAN_send(CanRxMsg *message)            //TX msg send
{


    IfxMultican_Message msg;                //can TX �޼��� ��ü ����

    //���Ź��� RX msg�� �̿��� dataLow, dataHigh ����
    const unsigned dataLow = message->Data[0]|(message->Data[1]<<8)|(message->Data[2]<<16)|(message->Data[3]<<24);
    const unsigned dataHigh = message->Data[4]|(message->Data[5]<<8)|(message->Data[6]<<16)|(message->Data[7]<<24);


    IfxMultican_Message_init(&msg,message->ID,dataLow,dataHigh,message->DLC);           //CAN msg �ʱ�ȭ

    while (IfxMultican_Can_MsgObj_sendMessage(&g_MulticanBasic.drivers.canSrcMsgObj, &msg) == IfxMultican_Status_notSentBusy)       //�޼��� ��ü �۽� �� �Ϸ���� ���
    {

    }
}


void CAN_TEST(void)             //CAN Test�� ���� �Լ�
{
    CanRxMsg MES;               //msg ����
    int i=0;
    MES.ID=0x890;               //ID = 0x890
    MES.IDE=0;                  //IDE(Identifier Extension)(Ȯ����) = 0
    MES.DLC=8;                  //DLC(Data Length Code) = 8
    for(i=0; i<8; i++)          //MES.Data�� a[i] ����
    {
        MES.Data[i]=a[i];
    }
    CAN_send(&MES);             //can send
}


/* Interrupt Service Routine (ISR) called once the TX interrupt has been generated.
 * Turns on the LED1 to indicate successful CAN message transmission.
 */
void canIsrTxHandler(void)          //���ͷ�Ʈ ���� ��ƾ���� ȣ��� �� blinkLED1() ����
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
    g_led.led1.port      = &MODULE_P00;                             //LED1 GPIO ��Ʈ          GPIO(General Purpose Input/Output)
    g_led.led1.pinIndex  = PIN5;                                    //LED1�� �ɹ�ȣ 05
    g_led.led1.mode      = IfxPort_OutputIdx_general;               //GPIO�� ���
    g_led.led1.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;  //LED1 �е� ����̹� ���� ����        //�е� ����̹� : ���� ������ Ư���� ����

    g_led.led2.port      = &MODULE_P00;                             //LED2 GPIO��Ʈ
    g_led.led2.pinIndex  = PIN6;                                    //LED2�� �ɹ�ȣ 06
    g_led.led2.mode      = IfxPort_OutputIdx_general;               //GPIO�� ���
    g_led.led2.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;  //LED2 �е� ����̹� ���� ����

    /* Initialize the pins connected to LEDs to level "HIGH"; will keep the LEDs turned off as default state */
    IfxPort_setPinHigh(g_led.led1.port, g_led.led1.pinIndex);       //LED1 ���� ���·� �ʱ�ȭ
    IfxPort_setPinHigh(g_led.led2.port, g_led.led2.pinIndex);       //LED2 ���� ���·� �ʱ�ȭ

    /* Set the pin input/output mode for both pins connected to the LEDs */
    IfxPort_setPinModeOutput(g_led.led1.port, g_led.led1.pinIndex, IfxPort_OutputMode_pushPull, g_led.led1.mode);       //LED1�� ��¸��(Output Mode)�� ����
    IfxPort_setPinModeOutput(g_led.led2.port, g_led.led2.pinIndex, IfxPort_OutputMode_pushPull, g_led.led2.mode);       //LED2�� ��¸��(Output Mode)�� ����

    /* Set the pad driver mode for both pins connected to the LEDs */
    IfxPort_setPinPadDriver(g_led.led1.port, g_led.led1.pinIndex, g_led.led1.padDriver);                                //LED1�� �е� ����̹� ����
    IfxPort_setPinPadDriver(g_led.led2.port, g_led.led2.pinIndex, g_led.led2.padDriver);                                //LED2�� �е� ����̹� ����
}


void blinkLED1(void)
{
    //IfxPort_togglePin(LED1);                                                     /* Toggle the state of the LED      */

    IfxPort_setPinHigh(LED1);                               //LED1 High(����)
    waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME_1ms));    /* Wait 1 milliseconds            */
    IfxPort_setPinLow(LED1);                                //LED1 Low(����)
}       //�����״� �ݺ��Ͽ� ������
void blinkLED2(void)
{
    IfxPort_togglePin(LED2);                                                     /* Toggle the state of the LED      */
    waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME_10ms));    /* Wait 10 milliseconds            */
}
