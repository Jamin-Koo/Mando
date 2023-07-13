/**********************************************************************************************************************
 * \file I2C_Read_Ext_Device.c
 * \copyright Copyright (C) Infineon Technologies AG 2019
 *
 * Use of this file is subject to the terms of use agreed between (i) you or the company in which ordinary course of
 * business you are acting and (ii) Infineon Technologies AG or its licensees. If and as long as no such terms of use
 * are agreed, use of this file is subject to following:
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization obtaining a copy of the software and
 * accompanying documentation covered by this license (the "Software") to use, reproduce, display, distribute, execute,
 * and transmit the Software, and to prepare derivative works of the Software, and to permit third-parties to whom the
 * Software is furnished to do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including the above license grant, this restriction
 * and the following disclaimer, must be included in all copies of the Software, in whole or in part, and all
 * derivative works of the Software, unless such copies or derivative works are solely in the form of
 * machine-executable object code generated by a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxI2c_I2c.h"


/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
/* MCP79411 I2C PINS */
#define MCP_SCL_PIN             IfxI2c0_SCL_P13_1_INOUT         /* SCL PIN                                          */
#define MCP_SDA_PIN             IfxI2c0_SDA_P13_2_INOUT         /* SDA PIN                                          */
#define I2C_BAUDRATE            100000                          /* 400 kHz baud rate                                */

#define MCP79411_EEPROM_ADDRESS 0x57                            /* 7 bit slave device address for reading from EEPROM
                                                                   of MCP79411 is 0b1010111 which is 0x57.          */
#define Address_Of_Mac_Address  0x57                            /* Location of EUI-48 node address (MAC address)    */
#define Length_Of_Address       8                               /* Length of address of the register, in which the
                                                                   requested MAC address is stored in bytes         */
#define Length_Of_Mac_Address   8                               /* Length of the MAC address in bytes               */
#define PIN                 &MODULE_P00,5
/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
IfxI2c_I2c          g_i2cHandle;                                /* I2C handle                                       */
IfxI2c_I2c_Device   g_i2cDevEeprom;                             /* I2C Slave device handle to EEPROM of MC79411     */

uint8 G_Mac_Addr[Length_Of_Mac_Address] = {0,};    /* Global parameter for 6-byte EUI-48 MAC address   */
uint8 Receive_Buff[Length_Of_Address] = { 0, };
uint8 Receive_Buff2[5] = { 0, };
uint8 Mode_Select = 0;

int cnt1 = 0;
int cnt2 = 0;
int cnt0 = 0;
union
{
    uint16 check_sum;
    uint8 b[2];
} Crc16_Value;


union
{
        uint16 data_value;
        uint8 data[2];
}Input_Data;

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
/* This function initializes the I2C in master mode and configures the MCP79411 (real time clock) as an I2C slave */
void INIT_I2C_MODULE(void)
{
    /* Initialize module */
    IfxI2c_I2c_Config i2cConfig;                                                /* Create configuration structure   */
    IfxI2c_I2c_initConfig(&i2cConfig, &MODULE_I2C0);                            /* Fill structure with default values
                                                                                   and Module address               */
    /* I2c pin configuration */
    const IfxI2c_Pins MCP_PINS =
    {
            &MCP_SCL_PIN,                                                       /* SCL port pin                     */
            &MCP_SDA_PIN,                                                       /* SDA port pin                     */
            IfxPort_PadDriver_ttlSpeed1                                         /* Pad driver mode                  */
    };
    i2cConfig.pins = &MCP_PINS;                                                 /* Configure port pins              */
    i2cConfig.baudrate = I2C_BAUDRATE;                                          /* Configure baud rate with 400kHz  */

    IfxI2c_I2c_initModule(&g_i2cHandle, &i2cConfig);                            /* Initialize module                */

    /* Initialize device */
    IfxI2c_I2c_deviceConfig i2cDeviceConfig;                                    /* Create device configuration      */
    IfxI2c_I2c_initDeviceConfig(&i2cDeviceConfig, &g_i2cHandle);                /* Fill structure with default values
                                                                                   and I2C Handler                  */
    /* Because it is 7 bit long and bit 0 is R/W bit, the device address has to be shifted by 1 */
    i2cDeviceConfig.deviceAddress = MCP79411_EEPROM_ADDRESS << 1;
    IfxI2c_I2c_initDevice(&g_i2cDevEeprom, &i2cDeviceConfig);                   /* Initialize the I2C device handle */
    IfxPort_setPinMode(PIN , IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinState(PIN , IfxPort_State_high);
}

/* This function executes the I2C data transfer.
 * The location of the MAC address is transmitted, then the MAC address is received.
 */
void READ_EXT_DEVICE_ADDRESS(void)
{
    /* Address of 6-byte EUI-48 MAC address location */
    uint8 i2cTxBuffer[Length_Of_Address] = {Address_Of_Mac_Address};

    /* Communication via I2C starts by transmitting the address of the specific I2C slave until the slave
     * is ready and confirms the reception via the acknowledge bit (IfxI2c_I2c_Status_nak = not acknowledge).
     * This procedure is done for both the write and read process.
     */
    /* Write data to device */
    //while(IfxI2c_I2c_write(&g_i2cDevEeprom, &i2cTxBuffer[0], Length_Of_Address) == IfxI2c_I2c_Status_nak);
    /* Read the unique MAC address */
    while(IfxI2c_I2c_read(&g_i2cDevEeprom, &G_Mac_Addr[0], Length_Of_Mac_Address) == IfxI2c_I2c_Status_nak);

}

int CRC16_MODBUS(const uint8* nData, uint16 wLength) {

   static const uint16 wCRCTable[] = {                             // <<== 2022.02.05 �߰� (�˼��մϴ�.~�Ф�)

   0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241, 0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440, 0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40, 0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841, 0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40, 0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41, 0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641, 0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040, 0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240, 0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441, 0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41, 0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840, 0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41, 0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40, 0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640, 0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041, 0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240, 0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441, 0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41, 0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840, 0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41, 0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40, 0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640, 0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041, 0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241, 0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440, 0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40, 0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841, 0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40, 0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41, 0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641, 0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };



   uint8 nTemp;

   uint16 wCRCWord = 0xFFFF;



   while (wLength--)

   {

      nTemp = *nData++ ^ wCRCWord;

      wCRCWord >>= 8;

      wCRCWord ^= wCRCTable[nTemp];

   }

   return wCRCWord;

}


int RX_BUFF(void)
{

    for(int i = 0; i<8; i++){
        Receive_Buff[i] = G_Mac_Addr[i];

    }

    for(int i = 0; i<5; i++){
            Receive_Buff2[i] = Receive_Buff[i];

     }

    cnt0++;
    if((Receive_Buff[0] == 0x73) && (Receive_Buff[7] == 0x65) )
    {
        cnt1++;

            Crc16_Value.check_sum = CRC16_MODBUS(Receive_Buff2, 5);

            if((  Crc16_Value.b[0] == Receive_Buff[5]) && (  Crc16_Value.b[1] == Receive_Buff[6]))
            {
                cnt2++;
                Mode_Select = Receive_Buff[1];
                Input_Data.data[0] = Receive_Buff[2];
                Input_Data.data[1] = Receive_Buff[3];
            }
    }
    return Input_Data.data_value;
}