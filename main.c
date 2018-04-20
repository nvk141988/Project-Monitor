/*
/ Project: Monitoring Device
/ By: NVK
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "nordic_common.h"
#include "nrf_gpio.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "SEGGER_RTT.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "string.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "semphr.h"
#include "CellDefines.h"
#include "twi_master.h"
#include "TMP103Defines.h"
#include "twi_hw_master.h"

#define TASK_DELAY        200    /**< Task delay. Delays a LED0 task for 200 ms */
#define TIMER_PERIOD      1000   /**< Timer period. LED1 timer will expire after 1000 ms */

#define LEDW    (15)
#define LEDR    (19)

bool            gbDataRecvd = false;
bool            gbUrcRecvd  = false;
bool            gbResponseRcvd = false;

TimerHandle_t tmp103_oneshot_convstart_handle;
TaskHandle_t  tmp103_readvalue_handle;
SemaphoreHandle_t       tempDataReady;
bool          oneShotModeSet = false;
uint8_t    i2cWriteBuff[3];

BaseType_t ret = 0;

//Function for handling HardFault
void HardFault_Handler(void)
{
    uint32_t *sp = (uint32_t *) __get_MSP(); // Get stack pointer
    uint32_t ia = sp[12]; // Get instruction address from stack
    char Buff[50];
    sprintf(Buff,"Hard Fault at address: 0x%08x\r\n", (unsigned int)ia);
    SEGGER_RTT_TerminalOut(0,Buff);
    while(1)
        ;
}
// Function for Initializing Temperature Sensor
void  Temp103SensorInit(void)
{
      uint8_t    i2cWriteBuff[3];

    memset(i2cWriteBuff, 0x00, sizeof(i2cWriteBuff));
    i2cWriteBuff[0] = TMP103_CONFIG_REG_SEL;        // Register to write to
    i2cWriteBuff[1] = TMP103_SHUTDOWN_MODE;         // Value to write to the register

    // Put TMP103 into shutdown mode
    if (I2C_DataWrite(TMP103_I2C_ADDR, i2cWriteBuff, 2))
    {    
        SEGGER_RTT_TerminalOut(0,"TMP103 Sensor Init Success!\n");
    }
    else
    {
        SEGGER_RTT_TerminalOut(0,"TMP103 Sensor Init Failed!\n"); 
    }
}

//Call Back Function for Starting Temperature Read by TMP103 module
static void config_oneshot_timer_callback (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    SEGGER_RTT_TerminalOut(0,"Starting TMP103 Temperature Conversion using OneShot Mode\n");


    // Put TMP103 into conversion mode
    memset(i2cWriteBuff, 0x00, sizeof(i2cWriteBuff));
    i2cWriteBuff[0] = TMP103_CONFIG_REG_SEL;        // Register to write to
    i2cWriteBuff[1] = TMP103_ONESHOT_MODE;         // Value to write to the register

    I2C_DataWrite(TMP103_I2C_ADDR, i2cWriteBuff, 2);
    oneShotModeSet = true;
    xSemaphoreGive(tempDataReady);

}

// Function for Reading Temperature from TMP103
static void tmp103_read_tempature (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    uint8_t   data;
    char       str[20];
    uint8_t         err_code;
    while(true)
    { 
      xSemaphoreTake(tempDataReady,portMAX_DELAY);//WAIT_FOR_CELL_RX);

      if(oneShotModeSet)
      {
        vTaskDelay(TMP103_TASK_DELAY);
        oneShotModeSet = false;
        i2cWriteBuff[0] = TMP103_TEMP_REG_SEL;        // Register to write to
        
        I2C_DataWrite(TMP103_I2C_ADDR, i2cWriteBuff, 1);
        
        // Now read the Temperature Reg value
        err_code = I2C_DataRead(TMP103_I2C_ADDR, (uint8_t *)&data, 1);
        if (err_code)    
        {
          sprintf(str,"Temperaure = %d\n",data);
          SEGGER_RTT_TerminalOut(1,str);
        }
        else
          SEGGER_RTT_TerminalOut(1,"Error Reading Data\n");
      }
      taskYIELD();
    }
}
  
// Function for Toggling White LED
static void vLed0Function (void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    for( ;; )
    {
#if (HARDFAULT_CHECK)
        SEGGER_RTT_TerminalOut(0,"In vLed0Function\n");
#endif
        nrf_gpio_pin_toggle(LEDW);
        vTaskDelay(TASK_DELAY); // Delay a task for a given number of ticks

        // Tasks must be implemented to never return...
    }
}

//Function for Sending Command to UBlox
void sendCellCmd (uint8_t * cellCmd)
{
      //int8_t    ch = cellCmd[0];
      uint32_t  i = 0;
      uint32_t  ret;
      uint8_t   ch;
      SEGGER_RTT_TerminalOut(0,"In sendCellCmd\n");
      if(currentStep == 9)
      {
        SEGGER_RTT_TerminalOut(0,"Finding Problem Lcation");
      }
      //taskENTER_CRITICAL();
      while(cellCmd[i] != '\0')
      {
        ch = cellCmd[i];
        ret = app_uart_put(ch);
        if(ret != NRF_SUCCESS)
        {
          SEGGER_RTT_TerminalOut(0,"Error Tx Uart\n");
        }
        i++;
      }
      //taskEXIT_CRITICAL();
      
}

//Event Handler for UART

void uartEventHandler(app_uart_evt_t * p_event)
{
  char Buff[10];
  uint8_t rxByte;
  uint32_t rxStatus = NRF_SUCCESS;
  //BaseType_t xHigherPriorityTaskWoken;
  //xHigherPriorityTaskWoken = pdFALSE;
#if (HARDFAULT_CHECK)
    SEGGER_RTT_TerminalOut(0,"In uartEventHandler\n");
#endif
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        SEGGER_RTT_TerminalOut(0,"Error while Cell Tx/Rx\n");
        //APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
      SEGGER_RTT_TerminalOut(0,"App UART Fifo Error\n");
        //APP_ERROR_HANDLER(p_event->data.error_code);
    }
    else if (p_event->evt_type == APP_UART_DATA)
    {
    SEGGER_RTT_TerminalOut(0,"Here\n");
        //APP_ERROR_HANDLER(p_event->data.error_code);
    }
    else if (p_event->evt_type == APP_UART_TX_EMPTY)
    {
//    SEGGER_RTT_TerminalOut(0,"Cmd Transmit Done\n");
        //APP_ERROR_HANDLER(p_event->data.error_code);
    }
    
    else if (p_event->evt_type == APP_UART_DATA_READY)
    {
      while (1)
      {
        rxStatus = app_uart_get(&rxByte);
        if( rxStatus== NRF_ERROR_NOT_FOUND)
        {
          break;
        }
        gRxBuffer[gNumOfRcvdCh] = rxByte;
        gNumOfRcvdCh++;
        //U260_RxPacket[sRxBytesCnt++] = rxByte;
      }
      sprintf(Buff,"%c",rxByte);
      SEGGER_RTT_TerminalOut(0,Buff);
      //gbResponseRcvd = true;
      if(cellWaitForResponseToCmd)
      {
        gbDataRecvd     = true;
        gbUrcRecvd      = false;
      }
      else
      {
        gbUrcRecvd      = true;
        gbDataRecvd     = false;
      }
       xSemaphoreGive(dataAvailableForProcessing);
    }
}

//Extract Data Received from Server
void    extractData(void)
{
}

const char gcOkString[] = "OK\r\n";
const char gcUploadSuccessString[] = "+UUHTTPCR: 0,5,1\r\n";

// Process response received from Cell Module
static void vProcessCellData(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    uint8_t* err_code;
    while(1)
    {
        xSemaphoreTake(dataAvailableForProcessing,portMAX_DELAY);
        SEGGER_RTT_TerminalOut(0,"In Process Data\n");
        if(gbDataRecvd)
        {
          switch(currentStep)
          {
              case  AT_GET_SERVER_TIME_STEP:
                              err_code = (uint8_t*)strstr((char*)(gRxBuffer),gcUploadSuccessString);
              if(err_code != '\0')
              {
                SEGGER_RTT_TerminalOut(0,"Ok String Found\n");
                //gIndexChecked = err_code - gRxBuffer;
                extractData();
                gNumOfRcvdCh = 0;
                memset(gRxBuffer,0,sizeof(gRxBuffer));
                xSemaphoreGive(waitForRespHandle);
              }  
                
              break;         
              default:  
              gbDataRecvd = false;
              err_code = (uint8_t*)strstr((char*)(gRxBuffer),gcOkString);
              if(err_code != '\0')
              {
                SEGGER_RTT_TerminalOut(0,"Ok String Found\n");
                //gIndexChecked = err_code - gRxBuffer;
                extractData();
                gNumOfRcvdCh = 0;
                memset(gRxBuffer,0,sizeof(gRxBuffer));
                xSemaphoreGive(waitForRespHandle);
              }
            }
        }
        if(gbUrcRecvd)
        {
            //Logic to process URC's
            SEGGER_RTT_TerminalOut(0,"URC Received\n");
        }
        vTaskDelay(TASK_DELAY);
    }
}

//Function for Configuring Cell Module.
static void vConfigCell (void* pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    char Buff[75];
    while(1)
    {   
        while(currentStep < NUM_OF_STEPS)
        {
            
            sendCellCmd(cellConfigCmds[currentStep]);
            cellWaitForResponseToCmd = true;
            sprintf(Buff,"Current Step : %d and Cmd:%s\n ",currentStep,cellConfigCmds[currentStep]);
            SEGGER_RTT_TerminalOut(0,Buff);
            if(currentStep == 9)
            {
              SEGGER_RTT_TerminalOut(0,"Current Step\n ");  
            }
            //Semaphore Take
            xSemaphoreTake(waitForRespHandle,portMAX_DELAY);//WAIT_FOR_CELL_RX);

            SEGGER_RTT_TerminalOut(0,"After Semaphore in ConfigCell\n");
            
             currentStep++;           
                        
        }
        taskYIELD();
    }

}

//Function for Initialization Cell Module UBLOX
  static void vInitCell (void *pvParameter)
  {
    UNUSED_PARAMETER(pvParameter);
    uint32_t err_code;
    SEGGER_RTT_TerminalOut(0,"I am Here in INitCell\n");
    //char Buff[50];
    //uint8_t cr[] = "AT\r\n" ;

    //int i=0;
    //uint8_t *  ptr;
    while(1)
    {

      nrf_gpio_pin_clear(CELL_POWER_ON_OFF_PIN);
      nrf_delay_us(70);
      nrf_gpio_pin_set(CELL_POWER_ON_OFF_PIN);
      nrf_delay_us(100000);
      const app_uart_comm_params_t comm_params =
      {
        CELL_UART_RXD_PIN,
        CELL_UART_TXD_PIN,
        CELL_UART_RTS_PIN,
        CELL_UART_CTS_PIN,
        APP_UART_FLOW_CONTROL_ENABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud19200
      };

      APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uartEventHandler,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

      APP_ERROR_CHECK(err_code);
      nrf_delay_us(1000);

      //Create task for Cell Configuration
      ret = xTaskCreate( vConfigCell, "CELLCONFIG", configMINIMAL_STACK_SIZE + 50, NULL, 2, &xConfigCellHandle );
      if(ret == -1)
      {
        SEGGER_RTT_TerminalOut(0,"Error Creating Task:vConfigCell");
      }
      //Create Task for Processing Cell Data
      ret = xTaskCreate( vProcessCellData, "PROCESSDATA", configMINIMAL_STACK_SIZE, NULL, 1, &xProcessCellDataHandle );
      if(ret == -1)
      {
        SEGGER_RTT_TerminalOut(0,"Error Creating Task:vProcessCellData");
      }
      //Delete Task ofr Init Cell Module
      vTaskDelete(xInitCellHandle);
      
    }
  }

int main(void)
{
  
    nrf_delay_ms(1000);
    TaskHandle_t  xLed0Handle;       /**< Reference to LED0 toggling FreeRTOS task. */
    //TimerHandle_t xLed1Handle;       /**< Reference to LED1 toggling FreeRTOS timer. */
    ret_code_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    // Configure LED-pins as outputs
    nrf_gpio_cfg_output(LEDW);
    nrf_gpio_cfg_output(LEDR);

    nrf_gpio_pin_set(LEDW);
    nrf_gpio_pin_set(LEDR);
    
    Temp103SensorInit();
    tmp103_oneshot_convstart_handle = xTimerCreate( "TMP103_ONESHOT", CONVERSION_DELAY, pdTRUE, NULL, config_oneshot_timer_callback);
    UNUSED_VARIABLE(xTimerStart(tmp103_oneshot_convstart_handle, 0));


    //Create task for reading value after conversion time has ended
    
    UNUSED_VARIABLE(xTaskCreate(tmp103_read_tempature, "TMP103_READ", configMINIMAL_STACK_SIZE /*+ 200*/, NULL, 2, &tmp103_readvalue_handle));    
    tempDataReady = xSemaphoreCreateBinary();           
    if(tempDataReady == NULL)
    {
        SEGGER_RTT_TerminalOut(0,"Error Creating Semaphore\n");
    }    
    UNUSED_VARIABLE(xTaskCreate( vLed0Function, "L0", configMINIMAL_STACK_SIZE, NULL, 2, &xLed0Handle ));    // LED0 task creation

    UNUSED_VARIABLE(xTaskCreate( vInitCell, "CELLINIT", configMINIMAL_STACK_SIZE, NULL, 2, &xInitCellHandle )); //Cell Init Task Create
    //Semaphores for gettiing and processing data from Cell Module
    waitForRespHandle = xSemaphoreCreateBinary();
    dataAvailableForProcessing = xSemaphoreCreateBinary();
    

    if((waitForRespHandle == NULL) || (dataAvailableForProcessing ==NULL))
    {
        SEGGER_RTT_TerminalOut(0,"Error Creating Semaphore\n");
    }
    
    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    while (true)
    {
        // FreeRTOS should not be here...
    }
}

/* Used in debug mode for assertions */
void assert_nrf_callback(uint16_t line_num, const uint8_t *file_name)
{
  while(1)
  {
    /* Loop forever */
  }
}

/**
 *@}
 **/
