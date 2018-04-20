/*
// File Name: CellDefines.h
// All Defines related to Cell 
*/

#ifndef CELLDEFINES_H
#define CELLDEFINES_H




#define CELL_POWER_ON_OFF_PIN 					(26u)	//[P0.26           : Pin #38] Cell module On/Off control signal GPIO pin.
#define CELL_RESET_L_PIN 						(25u)	//[P0.25           : Pin #37] Cell module Reset (active low) control signal GPIO pin.

// UART Port 0 GPIO signal pins
#define CELL_UART_TXD_PIN 						(28u)	//[P0.28           : Pin #40] Cell module UART TXD signal GPIO pin.
#define CELL_UART_RXD_PIN 						(27u)	//[P0.27           : Pin #39] Cell module UART RXD signal GPIO pin.
#define CELL_UART_RTS_PIN 						(30u)	//[P0.30           : Pin #42] Cell module UART RTS signal GPIO pin.
#define CELL_UART_CTS_PIN 						(29u)	//[P0.29           : Pin #41] Cell module UART CTS signal GPIO pin.

// DTR is used for power saving entry/exit
#define CELL_UART_DTR_PIN 						(3u)	 //[P0.03           : Pin #5] Cell module UART DTR signal GPIO pin.

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 128                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 128                           /**< UART RX buffer size. */

TaskHandle_t    xInitCellHandle;
TaskHandle_t    xConfigCellHandle;
TaskHandle_t    xProcessCellDataHandle;
#define CELL_RX_BUFF_SIZE               (128)
uint8_t         gRxBuffer[CELL_RX_BUFF_SIZE];
uint32_t        gNumOfRcvdCh = 0;
#define         AT_CMD                  ((uint8_t *)"AT\r\n")
#define         AT_DISBALE_ECHO         ((uint8_t *)"ATE0\r\n")
#define         AT_VERBOSE_ERR          ((uint8_t *)"AT+CMEE=2\r\n")
#define         AT_SIM_CARD_ID          ((uint8_t *)"AT+CCID\r\n")
#define         AT_GET_QUALITY          ((uint8_t *)"AT+CSQ\r\n")
#define         AT_DEREGISTER_NET       ((uint8_t *)"AT+COPS=2\r\n")
#define         AT_AUTO_SEL_OPERATOR    ((uint8_t *)"AT+COPS=0\r\n")
#define         AT_REGISTER_WITH_NET    ((uint8_t *)"AT+CREG=1\r\n")
#define         AT_ENABLE_GPRS          ((uint8_t *)"AT+CGREG=1\r\n")
#define         AT_SET_PSD_CONTEXT      ((uint8_t *)"AT+UPSD=0,1,\"Enter PSD Context Here"\r\n")
#define         AT_SET_IP               ((uint8_t *)"AT+UPSD=0,7,\"0.0.0.0\"\r\n")
#define         AT_GPRS_PROFILE_IN_NVM  ((uint8_t *)"AT+UPSDA=0,1\r\n")
#define         AT_START_GPRS           ((uint8_t *)"AT+UPSDA=0,3\r\n")
#define         AT_GET_IP               ((uint8_t *)"AT+UPSND=0,0\r\n")
#define         AT_RESET_PROFILE        ((uint8_t *)"AT+UHTTP=0\r\n")
#define         AT_SET_SERVER_IP        ((uint8_t *)"AT+UHTTP=0,0,\"Enter Server IP Here"\r\n")
#define         AT_SET_SERVER_PORT      ((uint8_t *)"AT+UHTTP=0,5,80\r\n")
#define         AT_GET_SERVER_TIME      ((uint8_t *)"AT+UHTTPC=0,5,"Enter Server Link Here",\"time.ffs\",\"TestString\",2\r")
#define         AT_READ_TIME_FILE       ((uint8_t *)"AT+URDFILE=\"time.ffs\"\r")

SemaphoreHandle_t       waitForRespHandle;
SemaphoreHandle_t       dataAvailableForProcessing;


#define         WAIT_FOR_CELL_RX        (pdMS_TO_TICKS(1000*5))
enum  cellConfigSteps
{
  AT_CMD_STEP,
  AT_DISBALE_ECHO_STEP,
  AT_VERBOSE_ERR_STEP,
  AT_SIM_CARD_ID_STEP,
  AT_GET_QUALITY_STEP,
  AT_DEREGISTER_NET_STEP,
  AT_AUTO_SEL_OPERATOR_STEP,
  AT_REGISTER_WITH_NET_STEP,
  AT_ENABLE_GPRS_STEP,
  AT_SET_PSD_CONTEXT_STEP,
  AT_SET_IP_STEP,
  AT_GPRS_PROFILE_IN_NVM_STEP,
  AT_START_GPRS_STEP,
  AT_GET_IP_STEP,
  AT_RESET_PROFILE_STEP,
  AT_SET_SERVER_IP_STEP,
  AT_SET_SERVER_PORT_STEP,
  AT_GET_SERVER_TIME_STEP,
  AT_READ_TIME_FILE_STEP,
  NUM_OF_STEPS
} cellStepsEn;


volatile uint8_t     currentStep = AT_CMD_STEP;
uint8_t*         cellConfigCmds[NUM_OF_STEPS] =  {AT_CMD,
                                                  AT_DISBALE_ECHO,
                                                  AT_VERBOSE_ERR,
                                                  AT_SIM_CARD_ID,
                                                  AT_GET_QUALITY,	
                                                  AT_DEREGISTER_NET,      
                                                  AT_AUTO_SEL_OPERATOR,
                                                  AT_REGISTER_WITH_NET,
                                                  AT_ENABLE_GPRS,
                                                  AT_SET_PSD_CONTEXT,
                                                  AT_SET_IP,
                                                  AT_GPRS_PROFILE_IN_NVM,
                                                  AT_START_GPRS,
                                                  AT_GET_IP,
                                                  AT_RESET_PROFILE,
                                                  AT_SET_SERVER_IP,
                                                  AT_SET_SERVER_PORT,
                                                  AT_GET_SERVER_TIME,
                                                  AT_READ_TIME_FILE,
                                                          };

volatile bool    cellWaitForResponseToCmd     = false;






#endif