/* FileName: TMP103Defines
// All Defines related to TMP103
*/

#ifndef TMP103DEFINES_H
#define TMP103DEFINES_H

#define CONVERSION_DELAY  (pdMS_TO_TICKS(10*1000))
#define TMP103_TASK_DELAY (pdMS_TO_TICKS(20))

// TI TMP103A Sensor Registers and settings
#define TMP103_I2C_ADDR                 ((uint8_t)(0x70))      // 7-bit I2C slave address in 7 LSB's.
#define TMP103_TEMP_REG_SEL             ((uint8_t)(0x00))      // P1,P0 : 00
#define TMP103_CONFIG_REG_SEL           ((uint8_t)(0x01))      // P1,P0 : 01
#define TMP103_TEMP_LOW_LEVEL_REG_SEL   ((uint8_t)(0x02))      // P1,P0 : 10
#define TMP103_TEMP_HIGH_LEVEL_REG_SEL  ((uint8_t)(0x03))      // P1,P0 : 11
// Conversion modes
#define TMP103_SHUTDOWN_MODE            ((uint8_t)(0x00))
#define TMP103_ONESHOT_MODE             ((uint8_t)(0x01)) 



#define TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER      (7u)	//[P0.07       : Pin #  7] I2C_SCL signal GPIO pin.
#define TWI_MASTER_CONFIG_DATA_PIN_NUMBER       (12u)	//[P0.12       : Pin #  12] I2C_SDA signal GPIO pin.



#endif