/*****************************************************************************
 *
 * Filename:
 * ---------
 *   catc24c16.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of CAM_CAL driver
 *
 *
 * Author:
 * -------
 *   LukeHu
 *
 *============================================================================*/
#ifndef __IMX298_EEPROM_H
#define __IMX298_EEPROM_H

#define CAM_CAL_DEV_MAJOR_NUMBER 226

/* CAM_CAL READ/WRITE ID */
#define imx298_EEPROM_READ_ID							0xA0
#define imx298_EEPROM_WRITE_ID                                          0xA1
/*#define I2C_UNIT_SIZE                                  1 //in byte*/
/*#define OTP_START_ADDR                            0x0A04*/
/*#define OTP_SIZE                                      24*/

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iBurstReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadReg(u16 a_u2Addr , u8 *a_puBuff , u16 i2cId);


#endif /* __CAM_CAL_H */

