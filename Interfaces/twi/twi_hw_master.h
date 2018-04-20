#ifndef TWI_HW_MASTER_H
#define TWI_HW_MASTER_H


extern bool I2C_DataWrite(uint8_t devAddr, uint8_t *pWriteBuff, uint8_t writeLen);
extern bool I2C_DataRead(uint8_t devAddr, uint8_t *pReadBuff, uint8_t readLen);







#endif