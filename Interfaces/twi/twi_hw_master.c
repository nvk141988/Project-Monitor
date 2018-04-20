/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

#include "twi_master.h"
//#include "collar_defines.h"
#include "TMP103Defines.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

//#if defined(TEMP_SENSOR_ENABLE)
/* Max cycles approximately to wait on RXDREADY and TXDREADY event, 
 * This is optimized way instead of using timers, this is not power aware. */
#define MAX_TIMEOUT_LOOPS             (20000UL)        /**< MAX while loops to wait for RXD/TXD event */

static bool twi_master_write(uint8_t *data, uint8_t data_length, bool issue_stop_condition)
{
    uint32_t timeout = MAX_TIMEOUT_LOOPS;   /* max loops to wait for EVENTS_TXDSENT event*/

    if (data_length == 0)
    {
        /* Return false for requesting data of size 0 */
        return false;
    }

    NRF_TWI0->TXD           = *data++;
    NRF_TWI0->TASKS_STARTTX = 1;
    
//    uint32_t status;

    /** @snippet [TWI HW master write] */            
    while (true)
    {
        while(NRF_TWI0->EVENTS_TXDSENT == 0 && NRF_TWI0->EVENTS_ERROR == 0 && (--timeout))
        {
            // Do nothing.
        }

        if (timeout == 0 || NRF_TWI0->EVENTS_ERROR != 0)
        {
          // Recover the peripheral as indicated by PAN 56: "TWI: TWI module lock-up." found at 
          // Product Anomaly Notification document found at 
          // https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
          NRF_TWI0->EVENTS_ERROR = 0;
          NRF_TWI0->ENABLE       = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos; 
//          NRF_TWI0->POWER        = 0; 
          nrf_delay_us(5); 
//          NRF_TWI0->POWER        = 1; 
          NRF_TWI0->ENABLE       = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

          (void)twi_master_init();          

          return false;
        }
        NRF_TWI0->EVENTS_TXDSENT = 0;
        if (--data_length == 0)
        {
            break;
        }

        //status = 
        NRF_TWI0->INTENSET;
        NRF_TWI0->TXD = *data++;
    }
    /** @snippet [TWI HW master write] */            
    
    if (issue_stop_condition) 
    { 
        NRF_TWI0->EVENTS_STOPPED = 0; 
        NRF_TWI0->TASKS_STOP     = 1; 
        /* Wait until stop sequence is sent */ 
        while(NRF_TWI0->EVENTS_STOPPED == 0) 
        { 
            // Do nothing.
        } 
        NRF_TWI0->EVENTS_STOPPED = 0;
    }
    return true;
}


/** @brief Function for read by twi_master. 
 */
static bool twi_master_read(uint8_t *data, uint8_t data_length, bool issue_stop_condition)
{
    uint32_t timeout = MAX_TIMEOUT_LOOPS;   /* max loops to wait for RXDREADY event*/

    if (data_length == 0)
    {
        /* Return false for requesting data of size 0 */
        return false;
    }
    else if (data_length == 1)
    {
        NRF_TWI0->SHORTS = TWI_SHORTS_BB_STOP_Enabled << TWI_SHORTS_BB_STOP_Pos;
    }
    else
    {
        NRF_TWI0->SHORTS = TWI_SHORTS_BB_SUSPEND_Enabled << TWI_SHORTS_BB_SUSPEND_Pos;
    }
    
    NRF_TWI0->EVENTS_RXDREADY = 0;
    NRF_TWI0->TASKS_STARTRX   = 1;
    
    /** @snippet [TWI HW master read] */                
    while (true)
    {
        while(NRF_TWI0->EVENTS_RXDREADY == 0 && NRF_TWI0->EVENTS_ERROR == 0 && (--timeout))
        {    
            // Do nothing.
        }
        NRF_TWI0->EVENTS_RXDREADY = 0;

        if (timeout == 0 || NRF_TWI0->EVENTS_ERROR != 0)
        {
          // Recover the peripheral as indicated by PAN 56: "TWI: TWI module lock-up." found at 
          // Product Anomaly Notification document found at 
          // https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
          NRF_TWI0->EVENTS_ERROR = 0;
          NRF_TWI0->ENABLE       = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos; 
//          NRF_TWI0->POWER        = 0; 
          nrf_delay_us(5); 
//          NRF_TWI0->POWER        = 1; 
          NRF_TWI0->ENABLE       = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

          (void)twi_master_init();          
          
          return false;
        }

        *data++ = NRF_TWI0->RXD;

        if (--data_length == 1)
        {
            NRF_TWI0->SHORTS = TWI_SHORTS_BB_STOP_Enabled << TWI_SHORTS_BB_STOP_Pos;
        }

        if (data_length == 0)
        {
            break;
        }

        // Recover the peripheral as indicated by PAN 56: "TWI: TWI module lock-up." found at 
        // Product Anomaly Notification document found at 
        // https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
        nrf_delay_us(20);      
        NRF_TWI0->TASKS_RESUME = 1;
    }
    /** @snippet [TWI HW master read] */                    

    /* Wait until stop sequence is sent */
    while(NRF_TWI0->EVENTS_STOPPED == 0)
    {
        // Do nothing.
    }
    NRF_TWI0->EVENTS_STOPPED = 0;

    // AA Disable shorts after the read
    NRF_TWI0->SHORTS = TWI_SHORTS_BB_STOP_Disabled << TWI_SHORTS_BB_STOP_Pos;

    return true;
}


/**
 * @brief Function for detecting stuck slaves (SDA = 0 and SCL = 1) and tries to clear the bus.
 *
 * @return
 * @retval false Bus is stuck.
 * @retval true Bus is clear.
 */
static bool twi_master_clear_bus(void)
{
    uint32_t twi_state;
    bool bus_clear;
    uint32_t clk_pin_config;
    uint32_t data_pin_config;
        
    // Save and disable TWI hardware so software can take control over the pins.
    twi_state        = NRF_TWI0->ENABLE;
    NRF_TWI0->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
    
    clk_pin_config                                        =  \
            NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER];    
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER] =   \
            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
          | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
          | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
          | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
          | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);    

    data_pin_config                                      = \
        NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER];
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER] = \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);    
      
    TWI_SDA_HIGH();
    TWI_SCL_HIGH();
    TWI_DELAY();

    if ((TWI_SDA_READ() == 1) && (TWI_SCL_READ() == 1))
    {
        bus_clear = true;
    }
    else
    {
        uint_fast8_t i;
        bus_clear = false;

        // Clock max 18 pulses worst case scenario(9 for master to send the rest of command and 9 
        // for slave to respond) to SCL line and wait for SDA come high.
        for (i=18; i--;)
        {
            TWI_SCL_LOW();
            TWI_DELAY();
            TWI_SCL_HIGH();
            TWI_DELAY();

            if (TWI_SDA_READ() == 1)
            {
                bus_clear = true;
                break;
            }
        }
    }
    
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER] = clk_pin_config;
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER]  = data_pin_config;

    NRF_TWI0->ENABLE = twi_state;

    return bus_clear;
}


/** @brief Function for initializing the twi_master.
 */
bool twi_master_init(void)
{
    /* To secure correct signal levels on the pins used by the TWI
       master when the system is in OFF mode, and when the TWI master is 
       disabled, these pins must be configured in the GPIO peripheral.
    */
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER] =     \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);   

    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER] =      \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);    

    NRF_TWI0->EVENTS_RXDREADY = 0;
    NRF_TWI0->EVENTS_TXDSENT  = 0;
    NRF_TWI0->PSELSCL         = TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER;
    NRF_TWI0->PSELSDA         = TWI_MASTER_CONFIG_DATA_PIN_NUMBER;
    NRF_TWI0->FREQUENCY       = TWI_FREQUENCY_FREQUENCY_K400 << TWI_FREQUENCY_FREQUENCY_Pos;
    NRF_TWI0->ENABLE          = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

    return twi_master_clear_bus();
}

bool twi_master_enable(bool state)
{
  if (state)
      NRF_TWI0->ENABLE          = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;
  else
      NRF_TWI0->ENABLE          = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
  
  return(state);        // Just send the state back for function return value
}



/*
 *
 * subAddr : register offset address value to write
 */
// devAddr  : 7-bit I2C slave device address 
bool I2C_DataRead(uint8_t devAddr, uint8_t *pReadBuff, uint8_t readLen)
{
    bool xferSuccess = false;

    twi_master_init();
    // This is already done at end of twi_master_init().
    // twi_master_clear_bus();             // ensure a clean bus for the transfer

    NRF_TWI0->ADDRESS      = devAddr;
    // last parm : true = issue a stop condition at end of the read.
    // Note: This needs to be evaluated to possibly do a more efficient I2C transfer. TODO: 07/24/15 - MN
    xferSuccess = twi_master_read(pReadBuff,  readLen, true);

    twi_master_clear_bus();             // ensure a clean bus, since we do not need the twi bus now

    return(xferSuccess);
}

/*
 *
 * subAddr : register offset address value to write
 */
// devAddr  : 7-bit I2C slave device address 
bool I2C_DataWrite(uint8_t devAddr, uint8_t *pWriteBuff, uint8_t writeLen)
{
    bool xferSuccess = false;

    twi_master_init();
    // This is already done at end of twi_master_init().
    // twi_master_clear_bus();             // ensure a clean bus for the transfer
    NRF_TWI0->ADDRESS      = devAddr;
    // last parm : true = issue a stop condition at end of the write.
    // Note: This needs to be evaluated to possibly do a more efficient I2C transfer. TODO: 07/24/15 - MN
    xferSuccess = twi_master_write(pWriteBuff,  writeLen, true);

    twi_master_clear_bus();             // ensure a clean bus, since we do not need the twi bus now

    return(xferSuccess);
}
//#endif


#if 0

/** @brief  Function for transfer by twi_master.
 */ 
bool twi_master_transfer(uint8_t   address, 
                         uint8_t * data, 
                         uint8_t   data_length, 
                         bool      issue_stop_condition)
{
    bool transfer_succeeded = false;
    if (data_length > 0 && twi_master_clear_bus())
    {
        NRF_TWI0->ADDRESS = (address >> 1);

        if ((address & TWI_READ_BIT))
        {
            transfer_succeeded = twi_master_read(data, data_length, issue_stop_condition);
        }
        else
        {
            transfer_succeeded = twi_master_write(data, data_length, issue_stop_condition);
        }
    }
    return transfer_succeeded;
}

bool twi_data_write(uint8_t dev_addr, uint8_t sub_addr, uint8_t *data, uint8_t length)
{
  bool success;
  uint8_t* p_combined;
  if (length > 0)
  {
    p_combined = malloc(sizeof(uint8_t) + length * sizeof(uint8_t)); // Addr + Length
    if (p_combined == NULL)
      D(printf("TWI: NULL POINT RETURNED FROM MALLOC\n"));
    memcpy(p_combined, &sub_addr, 1);
    memcpy(p_combined + 1, data, length);
  }
  else
    p_combined = &sub_addr;
  
  dev_addr = dev_addr & 0xFE;
  success = twi_master_transfer(dev_addr, p_combined, length + 1, true);
  
  if (length > 0)
    free(p_combined);
  
  return success;
}

bool twi_data_read(uint8_t dev_addr, uint8_t sub_addr, uint8_t *data, uint8_t length)
{
  bool success;
  success = twi_data_write(dev_addr, sub_addr, (uint8_t) NULL, 0);
  
  if (success)
  {
    dev_addr = dev_addr | 0x01;
    success = twi_master_transfer(dev_addr, data, length, true);
  }
  return success;
}
#endif

/*lint --flb "Leave library region" */
