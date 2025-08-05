/****************************************************
 * File Name: main.c
 * Description: [ESP32 - STPM34 Interface over UART]. CT mode
 *
 * Author Information:
 * --------------------
 * Name        : 
 * Email       : 
 * Organization: 
 * Created On  : 
 * Last Modified: 
 *
 * Version History:
 * -----------------
 * Version     : [v1.2]
 * Date        : [Feb 25, 2025]
 * Description : [VRMS validated with Minimum Threshold for Phase2 including zero outing invalid values]
 * Added dynamic configurations for choosing between Configurations (CT MODE or SHUNT MODE)
 * Added dynamic calculations to determine the IFACTOR and POWERFACTOR for 2 types of CTs (Current output and Voltage output)
 * This code parses Electrical Parameters from Dual Phase using STPM34 with ESP32 as host
 * Params : Phase1
 *  1. Vrms
 *  2. Irms
 *  3. Active Power
 *  4. Line Frequency
 *  5. Phase angle
 *  6. Apparent Power
 * Params : Phase2
 *  1. Vrms
 *  2. Irms
 *  3. Active Power
 *  4. Line Frequency
 *  5. Phase angle
 *  6. Apparent Power
 * Configuration Tested: SCT-013-030 CT on Both phases
 * Notes:
 * ------
 * For both Current measurements, the gain needs to be set to 2
 *
 ****************************************************/

 #include "string.h"
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <stdint.h>
 #include <stdbool.h>
 #include "nvs_flash.h"
 #include "driver/uart.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_system.h"
 #include "driver/spi_master.h"
 #include "soc/gpio_struct.h"
 #include "driver/gpio.h"
 #include "sdkconfig.h"
 #include "esp_log.h"

 


 #include "i2c_util.h"
 #include "tmp117.h"
 



 int read_temperature_TMP117(float *temperature);
 int read_accel_kx134(int16_t *x, int16_t *y, int16_t *z);
 #define KX134_CNTL1 0x1B
 #define KX134_ODCNTL 0x21
 #define KX134_I2C_ADDR 0x1F
 #define KX134_XOUT_L 0x08

 
 // UART configuration
 #define STPM_UART              UART_NUM_1
 #define RPI4_UART              UART_NUM_2
 #define RPI_UART_BUFF_SIZE     1024
 #define BUF_SIZE               256  // Define a large enough buffer size to handle all incoming data
 #define FRAME_SIZE             5   // Fixed frame size
 #define COMMAND_SIZE           5    // Size of the command (first 5 bytes)
 
 
 /*UART Configuration*/
 #define DEBUG_DEEP 1
 /*BitRate*/
 #define STPM3_BITRATE   9600
 #define RPI4_BITRATE    115200
 // Polling Duration to Query from STPM34
 #define SLEEP_INTERVAL  5000
 
 /*UART Tx and Rx Pin defintion*/

 /*STPM34 UART*/
 const int StpmRxPin = 16; 
 const int StpmTxPin = 17; 
/*RPi UART*/
 const int RpiTxPin = 33;
 const int RpiRxPin = 25;

 static const char *STPM_TAG = "UART STPM34";




 static const char *SENSOR_TAG = "SENSOR_TEST";
 static i2c_master_dev_handle_t tmp117_dev_handle;
 static i2c_master_dev_handle_t kx134_dev_handle;
 static tmp117_dev_t tmp117_device;
 static float g_current_temperature = 0.0f;
 static int16_t g_accel_x = 0, g_accel_y = 0, g_accel_z = 0;
 
 /* Register Address Mapping */
 #define START_REG_ADDR         0x00
 #define UART_SPI_STS_CTRLREG   0x28
 #define DUMMY_BYTE             0xFF

 /*Registers of our interest*/
 // 0x04
 // 0x05
 // 0x18
 // 0x19
 // 0x1A
 // 0x1B
 // 0x2E
 // 0x48
 // 0x4E
 // 0x54
 // 0x5C

// Constants from the datasheet for frequency calculation
#define PERIOD_LSB_US 8         // Period LSB in microseconds
#define FCLK_KHZ 125            // FCLK frequency in kHz
#define MIN_PERIOD_REG 0x600    // Minimum period register value
#define MAX_PERIOD_REG 0xF00    // Maximum period register value
 
 // Constants from the STPM34 datasheet for the calculation 
 #define CRC_8                 0x07 // Define the CRC-8 polynomial
 #define FRAME_LEN             5    // Standard Frame length with CRC - 5 bytes
 #define RESP_READ_TIMEOUT     2000 // Read timeut in ms for response from STPM34
 #define BLOCKS_TO_READ        69   // Read 69 blocks from STPM34. Each block is 4 bytes + 1 byte CRC
 
 #define BIT_MASK_STPM_DATA_VRMS             (0x00007FFF)
 #define BIT_MASK_STPM_DATA_C1_RMS           (0xFFFF8000)
 #define BIT_MASK_STPM_PRIM_CURR_ACTIVE_POW  (0x1FFFFFFF)
 #define BIT_MASK_STPM_PERIOD_VALUE          (0xFFF)

 /* In our case VFactor doesn't change at least for now*/
 

 // Hardware Constants 
 #define VREF   1.18
 #define R1     810000.0
 #define R2     470.0
 #define AV     2.0
 #define AI     2.0
 #define CAL_V  0.875
 #define CAL_I  0.875
 #define KINT   1.0
 
 /* Dynamic Configuration for CT integration
  * User chooses the configuration. Either a CT confugration or a SHUNT configuration 
  */
 #define CONFIG_SHUNT_MODE      0
 #define CONFIG_CT_MODE         1
 #define CONFIG_MODE            CONFIG_CT_MODE

 //Define the type of CTs.
 // We have CTs that produce either voltage or current on the secondary side, proportional to the current through the primary side
 #define CT_VOLTAGE_OUTPUT      0
 #define CT_CURRENT_OUTPUT      1

 // User selects CT type here
 #define CT_TYPE                CT_VOLTAGE_OUTPUT

 // Confiugure the selected CT Output type
 #if CT_TYPE == CT_VOLTAGE_OUTPUT
    // Example for CT with voltage output
    // User defines CT sensitivity here (in mV/A for voltage output, or ratio for current output)
    // Below CT config is for SCT-013-030 which produces 1V per 30 Amperes measured on the secondary of the CT from the line to which the CT is clamped
    //#define CT_MAX_RATING_CURRENT_AMPS     5.0
    //#define CT_MAX_RATING_MILLI_VOLTS      333
    #define CT_MAX_RATING_CURRENT_AMPS     60.0
    #define CT_MAX_RATING_MILLI_VOLTS      1000
    
 #elif CT_TYPE == CT_CURRENT_OUTPUT
    // Example for CT with current output
    // User defines CT Ratio (Primary:Secondary) here for current output
    // Below CT config is for unknown CT which produces 60mV per 1 Ampere measured on the secondary of the CT from the line to which the CT is clamped
    #define CT_MAX_RATING_CURRENT_AMPS     1 // 1:1 for RSHUNT
    #define CT_MAX_RATING_MILLI_VOLTS      60 // 30mOhs + 30 mOhm in series

 #endif
 
// Define the CT sensitivity here based on the CT's output mode
#define CT_SENSITIVITY ((float)CT_MAX_RATING_MILLI_VOLTS / CT_MAX_RATING_CURRENT_AMPS)

/* Calculations for VFACTOR, IFACTOR and POWERFACTOR Multipliers*/
/*Calculate CONFIG_VOLTAGE_MULTI_FACTOR alias Voltage Multiplication Factor*/
 #define CALC_VOLTAGE_MULTI_FACTOR() \
    ((uint32_t)((VREF * (1 + (R1 / R2)) / (AV * CAL_V)) * 100.0))

 /* Calculate CONFIG_CURRENT_MULTI_FACTOR alias Current Multiplication Factor */
 #define CALC_CURRENT_MULTI_FACTOR(sensitivity) \
    ((uint32_t)((VREF / (AI * CAL_I * (sensitivity / 1000.0) * KINT)) * 100.0))

 /* Calculate the CONFIG_POWER_FACTOR_MULTIPLE alias Power Multiplication Factor */
 #define CALC_POWER_MULTI_FACTOR(sensitivity) \
    ((uint32_t)((VREF * VREF * (1 + (R1 / R2)) / (KINT * AV * AI * sensitivity * CAL_V * CAL_I) * 100000)))    

// TO-DO validate the else part (CT_VOLTAGE_OUTPUT)

 #define PHASE_ONE_NUMBER       1
 #define PHASE_TWO_NUMBER       2

 #define CHANNEL_ONE            1
 #define CHANNEL_TWO            2

 #define FACTOR_PH_ANGLE        360.0


 /* Arrival on the below calculations are done
 * based on the mandate that the Current Gain for 
 * both the channels are set to 2
 * CT Sensitivity for shunt is nothing but the resistors installed
 * -i.e 30mOhms + 30mOhms inseries = 60 mOhms. Hence sensitivity is 60 mV/A
 * CT Sensitivity for CT configuration is the actual sensitivity of the CT
 * Which is 1V/30A = 33.33 mV/A
 */
 //Now that the decision on the modes are based on the sensitivity calculation, we need not validate the expected mode as it was previously.
 // Refer to #define CT_SENSITIVITY
 #define CONFIG_VOLTAGE_MULTI_FACTOR                CALC_VOLTAGE_MULTI_FACTOR() // Will output 116274
 #define CONFIG_CURRENT_MULTI_FACTOR                CALC_CURRENT_MULTI_FACTOR(CT_SENSITIVITY)
 #define CONFIG_POWER_FACTOR_MULTIPLE               CALC_POWER_MULTI_FACTOR(CT_SENSITIVITY)
                 
 /*#define CONFIG_VOLTAGE_MULTI_FACTOR                116274 // Will output 116274
 #define CONFIG_CURRENT_MULTI_FACTOR                4214
 #define CONFIG_POWER_FACTOR_MULTIPLE               4900123
*/
 
 #define FACTOR_POWER_ON_ENERGY                         (858)
 #define ENERGY_FACTOR_MULTIPLE                         (uint32_t)CONFIG_POWER_FACTOR_MULTIPLE / FACTOR_POWER_ON_ENERGY

 // If the VRMS value parsed from Phase2 is < VRMS_PHASE2_MIN_THRESHOLD, Zeroout that parameter and other params for Phase2
 #define VRMS_PHASE2_MIN_THRESHOLD                      10.0 //10V
 
 // Index in the rx buffer for respective params.
 // This is valid in the context of reading all registers in oneshot
 /* The reception of data, is as below
 * Set gain values for both channels (Phases) with 1 second intermittent delay
 * Set Auto latch [0xFF, 0x04, 0xE0, 0x04, 0x98, 0xFF, 0x05, 0x80, 0x00, 0x31]
 * Query the parameters from 49 uint32_t registers in one shot
 * Total Transactions needed are totally 52
 * send the read command for reg 00 [0x00, 0xff, 0xff, 0xff, 0xF0] followed by 69 dummy frames [0xFF, 0xFF, 0xFF, 0xFF, 0x7B]
 * In the response, ignore the first 3 frames (15 bytes).
 * Then start parsing from the 4th frame upto the last frame.
 * These frames are reversed in nature and usually for both the tx and rx, the 5th bytes is the CRC for the 4 bytes data.
 * During reception, the CRC is now omitted and the 4 data bytes are parsed and gathered in an array of size (69 + 3) * 4 elements
 * Below index numbers are cosidered assuming that the frame size would be 4 bytes and CRC is ignored
 */
#define PERIOD_REG_STA_INDEX    104
#define PERIOD_REG_END_INDEX    107     
#define RMSDATASTARTIND_PH1     156
#define RMSDATAENDIND_PH1       159
#define RMSDATASTARTIND_PH2     160
#define RMSDATAENDIND_PH2       163
#define PH_ANG_STA_INDEX_PH1    168
#define PH_ANG_END_INDEX_PH1    171
#define PH_ANG_STA_INDEX_PH2    176
#define PH_ANG_END_INDEX_PH2    179
#define ACTPWRDATASTARTIND_PH1  196
#define ACTPWRDATAENDIND_PH1    199
#define ACTPWRDATASTARTIND_PH2  244
#define ACTPWRDATAENDIND_PH2    247
#define MAX_STRING_LENGTH       250
//Clean up below
#define APPPWRDATASTARTIND_PH1  208
#define APPPWRDATAENDIND_PH1    211
#define APPPWRDATASTARTIND_PH2  256
#define APPPWRDATAENDIND_PH2    259

//Macro to configure DEBUG message related to CT Config
#define DEBUG_CONFIG_MACROS  0
uint8_t rpi_uart_buff_size = 0;
 
//Below struct is used in STPM34 Uart init
 const uart_config_t stpm_uart_config = 
{
    .baud_rate = STPM3_BITRATE,
    .data_bits = UART_DATA_8_BITS,
    .parity     = UART_PARITY_DISABLE,
    .stop_bits  = UART_STOP_BITS_1,
    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE
};

//Below struct is used in RPi Uart init
const uart_config_t rpi_uart_config = 
{
    .baud_rate  = RPI4_BITRATE,
    .data_bits  = UART_DATA_8_BITS,
    .parity     = UART_PARITY_DISABLE,
    .stop_bits  = UART_STOP_BITS_1,
    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE
};

/* Structure to maintain the parameters to be measured from STPM34*/
typedef struct 
{
    float rmsVoltagePhase1;
    float rmsVoltagePhase2;
    float rmsCurrentPhase1;
    float rmsCurrentPhase2;
    double frequencyPhase1;
    double frequencyPhase2;
    float phaseAnglePhase1;
    float phaseAnglePhase2;
    int32_t activePowerPhase1;
    int32_t activePowerPhase2;
    //Clean up below
    int32_t apparentPowerPhase1;
    int32_t apparentPowerPhase2;
    int32_t activeEnergy;

    // New Fields
    float temperatureC;
    float accelX;
    float accelY;
    float accelZ;

}STPM34Params;

 /*Function Prototypes*/
int formatSTPMParams(STPM34Params stpmParams, char* formattedString);
 void stpm_uart_init(void);
 uint8_t* readRegister (uint8_t regAddr, bool dummyRead);
 void writeRegister (uint8_t regAddr, uint8_t * data);
 uint8_t metroHalCrc8Calc(uint8_t inData, uint8_t crcChecksum);
 uint8_t metroHalCalcCrc8(const uint8_t *frame, size_t frameLength);
 uint8_t metroHalByteReverse(uint8_t inByte);
 void reverseFrame(uint8_t *frame, size_t frameLength, uint8_t *reversedFrame);
 void send_command(uint8_t *cmd, size_t cmd_len);
 void sendDummyFrame(void);
 void setLatchRegister(void);
 void performDummyRead(void);
 void setGainValue(uint8_t channel);
 void read_response(uint8_t *response, size_t len);
 void rearrangeBytes(const uint8_t* buffer, uint8_t* reversed, size_t length, uint16_t start_index);
 void flush_uart_tx_rx_buff(void);
 void readAll_Params(void);
 void readConsecutiveRegistersToBuffer(uint8_t startRegister, uint8_t count, uint8_t *rxBuffer);
 void parseRMS(uint8_t* rmsBuff, uint8_t phase_number);
 //Clean up below
 void parseAppPower(uint8_t* appPowBuff, uint8_t phase_number);
 void parseActPower(uint8_t* actPowBuff, uint8_t phase_number);
 double calculate_period_seconds(const uint8_t *reg_ptr, uint8_t phase_number);
 void updateData(void);
 void logData(void);
 void log_to_rpi(uint8_t *cmd, size_t cmd_len);
 float calculate_phase_angle(uint8_t phase_number, uint8_t* cx_pha_register_value, double frequency_hz);
 void debug_calculation_macros(void);
  void read_electrical_params_task(void *param);
 /* Buffers for Storing Tx and Rx Frames*/
 uint8_t STPM34_TxBuffer[FRAME_LEN];
 uint8_t STPM34_RxBuffer[FRAME_LEN];
 
 /* This is what the dummy frame looks like */
 /* |FF|FF|FF|FF|7B| - 7B - Checksum */
 uint8_t STPM34_DummyFrame[FRAME_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0x7B};
 
 /*Manual Latch Command*/
 
 /* The convention of writing to the latch register followed from STM's software library 
 * //uint8_t latchReg_SW[10] = {0xFF, 0x04, 0xE0, 0x04, 0x98, 0xFF, 0x05, 0x60, 0x00, 0xE7}; */
 
 /* Auto Latch command Phase1 - Autolatch with LED enabled (Verfied) */
 uint8_t latchReg_SW[10] = {0xFF, 0x04, 0xE0, 0x04, 0x98, 0xFF, 0x05, 0x80, 0x00, 0x31};
 
 
 /* Autolatch with LED disabled (Verfied) */
 //uint8_t latchReg_SW[10] = {0xFF, 0x04, 0xE0, 0x04, 0x98, 0xFF, 0x05, 0x80, 0x03, 0x43};
 
 
 /* Phase 1 current gain (Ai) set to 2*/
 uint8_t ch1_gainReg[10] = {0xFF, 0x18, 0x27, 0x03, 0xDE, 0xFF, 0x19, 0x27, 0x00, 0x7C};
 
 /* Phase 2 current gain (Ai) set to 2*/
 uint8_t ch2_gainReg[10] = {0xFF, 0x1A, 0x27, 0x03, 0xBF, 0xFF, 0x1B, 0x27, 0x00, 0x1D};

 /*Write default values to DSP_CR3. So that frequency chosen is 50Hz*/
 uint8_t dsp_cr3[10] = {0xFF, 0x04, 0xE0, 0x04, 0x98, 0xFF, 0x05, 0x00, 0x09, 0x99};

 /* Select 60Hz as REF_FREQ - U.S Uncomment the below sections if 60Hz is the reference frequency */
 //uint8_t dsp_cr3[10] = {0xFF, 0x04, 0xE0, 0x04, 0x98, 0xFF, 0x05, 0x00, 0x08, 0x97};
 
 /* Global Variables to store Values */
 uint64_t rmsvoltage;
 uint64_t rmscurrent;
 int32_t powerActive;
 // Tight buffer to hold the Rx bytes
 // Response buffer would only have 4 byte values for each dummy frame.
 // Total would be 3(Prev responses) + 69(responses) for dummy frames
 // Total bytes = (69 + 3) (4) = 288 bytes
 uint8_t RxBuff[288];
 
 // Create an instance of the structure
 STPM34Params stpmParams;
 
uint32_t reverse_bytes(const uint8_t *reg_ptr)
{
    uint32_t reversed = 0;
    uint8_t i,j;

    for(i = 0, j=0; i < 4 && j < 32; i++, j+=8)
    {
        reversed |= reg_ptr[i] << (32 - 8 - j);
    }
    return reversed;
}

 /**
 * @brief Calculates the period in seconds and frequency in Hz from a register value.
 *
 * @param reg_ptr Pointer to the start of the 32-bit register value.
 * @param phase_number The phase number (PHASE_ONE_NUMBER or PHASE_TWO_NUMBER).
 *
 * @return The calculated frequency in Hz. Returns -1.0 if the period is out of range.
 *
 * @details This function performs the following steps:
 *          1. Reads a 32-bit register value from the provided pointer.
 *          2. Extracts the period value based on the specified phase number.
 *          3. Validates the period value is within the allowed range.
 *          4. Calculates the period in seconds and the frequency in Hz.
 *          5. Stores the calculated frequency in the appropriate global variable.
 *
 * @note The function uses bit manipulation to extract the period value and assumes
 *       the period is stored in the lower 12 bits of the register.
 *
 * @warning This function logs debug information using ESP_LOGI. Ensure the STPM_TAG
 *          is defined before calling this function.
 */

 double calculate_period_seconds(const uint8_t *reg_ptr, uint8_t phase_number) {
    uint32_t dsp_reg1_value=0;

    dsp_reg1_value = reverse_bytes(reg_ptr);

    ESP_LOGI(STPM_TAG,"Period Register Value: 0x%" PRIX32, dsp_reg1_value);

    // Option 2: Using memcpy (for unaligned access and potential endianness issues)
    // memcpy(&dsp_reg1_value, reg_ptr, sizeof(dsp_reg1_value));

    // Extract the period value from the register.
    // Assuming the period is in the lower 12 bits (bits 0-11)
    uint16_t period_register_value = 0;
    if(phase_number == PHASE_ONE_NUMBER)
    {
        ESP_LOGI(STPM_TAG, "Calculating Period for Phase-1");
        period_register_value = (uint16_t)dsp_reg1_value & BIT_MASK_STPM_PERIOD_VALUE;
        ESP_LOGI(STPM_TAG, "Period Register Phase-1: 0x%" PRIu16, period_register_value);
    }
    else if(phase_number == PHASE_TWO_NUMBER)
    {
        ESP_LOGI(STPM_TAG, "Calculating Period for Phase-2");
        period_register_value = (uint16_t)(dsp_reg1_value >> 16) & BIT_MASK_STPM_PERIOD_VALUE;
        ESP_LOGI(STPM_TAG, "Period Register Phase-2: 0x%" PRIu16, period_register_value);
    }

    // Validate the period is in the allowed range
    if (period_register_value < MIN_PERIOD_REG || period_register_value > MAX_PERIOD_REG) {
        printf("Error: Period out of range!\n");
        return -1.0; // Indicate an error
    }

    // Calculate the period in seconds:
    double period_in_seconds = (double)period_register_value * PERIOD_LSB_US / 1000000.0;
    // Calculate the frequency in Hz:
    double frequency_hz = 1.0 / period_in_seconds;
    ESP_LOGI(STPM_TAG, "Frequency: %.2f Hz", frequency_hz);
    if(phase_number == PHASE_ONE_NUMBER)
        stpmParams.frequencyPhase1 = frequency_hz;
    else if(phase_number == PHASE_TWO_NUMBER)
        stpmParams.frequencyPhase2 = frequency_hz;
    return frequency_hz;
}
 
 
 /**
 * @brief Calculates the phase angle for a given phase using STPM34 register values.
 *
 * This function implements the phase angle calculation as per Equation 13 in the STPM34 datasheet.
 * It constructs the register value from individual bytes, extracts the relevant bits,
 * and applies the formula: φ = (Cx_PHA[11:0] / FClk) * f * 360°
 *
 * @param phase_number The number of the phase (1 or 2)
 * @param cx_pha_register_value Pointer to the 4-byte array containing the register value
 * @param frequency_hz The frequency of the measured signal in Hz
 *
 * @return The calculated phase angle in degrees
 *
 * @note The result is also stored in the global stpmParams structure for the respective phase.
 */

float calculate_phase_angle(uint8_t phase_number, uint8_t* cx_pha_register_value, double frequency_hz) {
    // Phase-angle φ in degrees can be calculated from the register value as follows:
    // φ = (Cx_PHA[11:0] / FClk) * f * 360°  (Datasheet Equation 13)
    uint32_t cx_pha_reg_value = 0;

    /* cx_pha_reg_value |= (uint32_t)cx_pha_register_value[0] << 24;  // First byte to the highest byte of uint32_t
    cx_pha_reg_value |= (uint32_t)cx_pha_register_value[1] << 16;  // Second byte to the second highest byte
    cx_pha_reg_value |= (uint32_t)cx_pha_register_value[2] << 8;   // Third byte to the second lowest byte
    cx_pha_reg_value |= (uint32_t)cx_pha_register_value[3];        // Fourth byte to the lowest byte
    */
    cx_pha_reg_value = reverse_bytes(cx_pha_register_value);

    cx_pha_reg_value = (uint16_t)(cx_pha_reg_value >> 16) & BIT_MASK_STPM_PERIOD_VALUE;
    float phase_angle = ((float)cx_pha_reg_value / (float)(FCLK_KHZ * 1000.0)) * frequency_hz * FACTOR_PH_ANGLE;
    //phase_angle = 360 - phase_angle;
    ESP_LOGI(STPM_TAG, "Phase Angle for Phase-%d is: %.3f", phase_number, phase_angle);
    
    if(phase_number == PHASE_ONE_NUMBER)(stpmParams.phaseAnglePhase1 = phase_angle);
    else if(phase_number == PHASE_TWO_NUMBER)(stpmParams.phaseAnglePhase2 = phase_angle);
    return phase_angle;
}

 /**
  * @brief Sends a command to the UART interface.
  * 
  * This function sends a command (a sequence of bytes) through the UART interface. 
  * The `cmd` array holds the command bytes, and the function sends these bytes over UART. 
  * After sending the command, the function logs the bytes that were sent for debugging purposes.
  * 
  * @param cmd A pointer to the array containing the command to be sent.
  * @param cmd_len The length of the command (number of bytes to send).
  * 
  * @note This function uses `uart_write_bytes` to send the command over UART.
  * 
  * @warning Ensure that the `cmd` array is properly initialized and contains valid command data before calling this function.
  */
 
 void send_command(uint8_t *cmd, size_t cmd_len) 
 {
     uart_write_bytes(STPM_UART, (const char *)cmd, cmd_len);
     ESP_LOGD(STPM_TAG, "Sent command: ");
     for (int i = 0; i < cmd_len; i++) 
     {
         ESP_LOGD(STPM_TAG, "0x%02X", cmd[i]);
     }
 }


/**
 * @brief Logs a command to a Raspberry Pi via UART, with error checking.
 *
 * This function writes a byte array to the specified UART port (RPI4_UART).
 * It includes error checking to ensure that all bytes are written to the UART FIFO
 * and that the transmission completes successfully.  If errors are detected, they are logged
 * using ESP_LOGE.
 *
 * @param cmd       Pointer to the byte array containing the command to be sent.
 * @param cmd_len   The length (number of bytes) of the command to be sent.
 *
 * @note This function uses `uart_write_bytes` to write data to the UART FIFO, then calls
 *       `uart_wait_tx_done` to ensure the transmission completes.  It's important to
 *       choose an appropriate timeout value for `uart_wait_tx_done` based on the baud rate
 *       and the amount of data being sent. Also, be aware of potential memory constraints when handling cmd.
 */

 void log_to_rpi(uint8_t *cmd, size_t cmd_len) {
        int bytes_written = uart_write_bytes(RPI4_UART, (const char *)cmd, cmd_len);
    
        if (bytes_written != cmd_len) {
            ESP_LOGE(STPM_TAG, "Error: Only %d bytes written to UART FIFO instead of %d", bytes_written, cmd_len);
            return; // Or handle the partial write appropriately
        }
    
        // Wait for transmission to complete (with timeout)
        esp_err_t err = uart_wait_tx_done(RPI4_UART, pdMS_TO_TICKS(100)); // 100ms timeout
        if (err != ESP_OK) {
            ESP_LOGE(STPM_TAG, "UART transmission error: %s", esp_err_to_name(err));
            return; // Or handle the error
        }
        ESP_LOGI(STPM_TAG,"Data sent to RPi UART");
 }
 
 
 /**
  * @brief Reads data from the UART interface into a buffer.
  * 
  * This function attempts to read a specified number of bytes from the UART interface into the `response` buffer. 
  * It will keep trying until the desired number of bytes have been read or a timeout occurs. 
  * The function allows up to 5 attempts (with 1-second timeouts) to read the data, making it robust against transient communication issues.
  * 
  * If the function successfully reads data, it updates the `response` buffer with the received bytes. 
  * The number of bytes successfully read in each attempt is logged for debugging purposes.
  * 
  * @param response The buffer where the received data will be stored.
  * @param len The total number of bytes to read into the `response` buffer.
  * 
  * @note The function makes a maximum of 5 attempts to read the required number of bytes. 
  * If the required number of bytes is not read within these attempts, the function exits without completing the read operation.
  * 
  * @warning Ensure that the `response` buffer is large enough to store the required number of bytes (`len`).
  */
 
 void read_response(uint8_t *response, size_t len) 
 {
     int total_bytes_read = 0;
     int attempts = 0;
     int bytes_read = 0;
     while (total_bytes_read < len) 
     {
         attempts++;
         bytes_read = uart_read_bytes(STPM_UART, response + total_bytes_read, len - total_bytes_read, pdMS_TO_TICKS(1000));
         if (bytes_read > 0) 
         {
             total_bytes_read += bytes_read;
             ESP_LOGD(STPM_TAG, "Read %d bytes", bytes_read);
         }
         else 
         {
             ESP_LOGW(STPM_TAG, "Timeout or no data");
         }
         if(attempts >= 5)break;
     }
 }
 
 
 /**
  * @brief Calculate the CRC-8 for a single byte.
  *
  * @param inData Input byte for CRC calculation
  * @param crcChecksum Current checksum value
  * @return Updated CRC checksum
  */
 uint8_t metroHalCrc8Calc(uint8_t inData, uint8_t crcChecksum) 
 {
    uint8_t temp;
     for (int i = 0; i < 8; i++) // Process each bit
     { 
         temp = inData ^ crcChecksum;
         crcChecksum <<= 1;
         if (temp & 0x80)  // Check MSB
         { 
             crcChecksum ^= CRC_8;
         }
         inData <<= 1;
         crcChecksum &= 0xFF; // Ensure 8-bit result
     }
     return crcChecksum;
 }
 
 /**
  * @brief Calculate the CRC-8 for a frame.
  *
  * @param frame Pointer to the data frame (array)
  * @param frameLength Length of the data frame
  * @return CRC-8 checksum
  */
 uint8_t metroHalCalcCrc8(const uint8_t *frame, size_t frameLength) 
 {
     uint8_t crcChecksum = 0x00; // Initialize checksum
     for (size_t i = 0; i < frameLength; i++) {
         crcChecksum = metroHalCrc8Calc(frame[i], crcChecksum);
     }
     return crcChecksum;
 }
 
 /**
  * @brief Reverse the bits of an 8-bit byte.
  *
  * @param inByte Input byte
  * @return Byte with reversed bit order
  */
 uint8_t metroHalByteReverse(uint8_t inByte) 
 {
     inByte = ((inByte >> 1) & 0x55) | ((inByte << 1) & 0xAA);
     inByte = ((inByte >> 2) & 0x33) | ((inByte << 2) & 0xCC);
     inByte = ((inByte >> 4) & 0x0F) | ((inByte << 4) & 0xF0);
     return inByte;
 }
 
 /**
  * @brief Reverse the bits of each byte in the frame.
  *
  * @param frame Pointer to the data frame (array)
  * @param frameLength Length of the data frame
  * @param reversedFrame Pointer to store the reversed frame
  */
 void reverseFrame(uint8_t *frame, size_t frameLength, uint8_t *reversedFrame) 
 {
     for (size_t i = 0; i < frameLength; i++) 
     {
         reversedFrame[i] = metroHalByteReverse(frame[i]);
     }
 }
 
 /**
  * @brief Reverses a portion of the input buffer starting from a given index.
  * 
  * This function rearranges (reverses) the bytes of the input buffer, starting from the specified `start_index` and reversing the first 4 bytes from that point onward. 
  * The reversed bytes are stored in the `reversed` output buffer. 
  * This operation is useful for transforming data into a different format or endianness, such as when dealing with multi-byte data that needs to be swapped or reordered.
  * 
  * The function performs a check to ensure that the length of the data is sufficient (at least 4 bytes), as reversing fewer bytes might not provide meaningful results.
  * 
  * @param buffer The input data buffer that contains the bytes to be reversed.
  * @param reversed The output buffer where the reversed bytes will be stored.
  * @param length The total length of the data buffer.
  * @param start_index The index in the `buffer` from where the reversal should begin.
  * 
  * @note The function reverses exactly 4 bytes starting from the `start_index`. If the length is less than 4, it returns an error message and does nothing.
  * 
  * @warning Ensure that the `reversed` buffer is large enough to hold the rearranged bytes.
  */
 
 void rearrangeBytes(const uint8_t* buffer, uint8_t* reversed, size_t length, uint16_t start_index) 
 {
     if (length < 4) {
         printf("Buffer must have at least 4 bytes.\n");
         return;
     }
     // Reverse the first 4 bytes
     for (size_t i = 0; i < length; i++) {
         reversed[i] = buffer[start_index + 3 - i];
     }
 }
 
/**
 * @brief Formats STPM parameter values into a string.
 *
 * This function takes an STPMParams struct as input and formats its members
 * into a string enclosed in curly braces. The string contains the RMS voltage and current
 * for both phases, and the active power for both phases (divided by 1000.0 to convert to kW).
 * The function allocates memory for the string, which the caller is responsible for freeing
 * using `free()`.
 *
 * @param stpmParams A struct containing the STPM parameter values to be formatted.
 * @return A pointer to a dynamically allocated string containing the formatted STPM
 *         parameters, or NULL if memory allocation fails or if there is an error during string
 *         formatting.  It is the *caller's responsibility* to `free()` the returned memory.
 *
 * @note The function uses `snprintf` to prevent buffer overflows. If `snprintf` encounters
 *       an error or the output is truncated due to the buffer being too small, the function frees
 *       any allocated memory and returns NULL.  Make sure `MAX_STRING_LENGTH` is large
 *       enough to accommodate the formatted string. activePowerPhase1 and activePowerPhase2 assumed to be in Milliwatts, and converted to Watts inside the function.
 */


int formatSTPMParams(STPM34Params stpmParams, char* formattedString) {

    /*
    //Allocate memory for the string.  Caller is responsible for freeing it.
    char *buffer = (char*) malloc(MAX_STRING_LENGTH);
    if (buffer == NULL) {
        fprintf(stderr, "Memory allocation failed!\n");
        return NULL; // Indicate failure
    }
    */

    //char buffer[MAX_STRING_LENGTH];
    if (!formattedString) {
        fprintf(stderr, "Null string passed to formatSTPMParams!\n");
        return -1;
    }

    // Use snprintf to safely format the string
    int snprintf_result = snprintf(
        formattedString,
        MAX_STRING_LENGTH,
        "{V1: %.3f, I1: %.3f, P1: %.3f, S1: %.3f, F1: %.2f, PH1: %.2f, V2: %.3f, I2: %.3f, "
        "P2: %.3f, S2: %.3f, F2: %.2f, PH2: %.2f, "
        "Temp(C): %.2f, AccelX: %.2f, AccelY: %.2f, AccelZ: %.2f}\r\n",
        stpmParams.rmsVoltagePhase1,
        stpmParams.rmsCurrentPhase1,
        (float)stpmParams.activePowerPhase1 / 1000.0f,  //Corrected the division and cast
        (float)stpmParams.apparentPowerPhase1 / 1000.0f,
        stpmParams.frequencyPhase1,
        stpmParams.phaseAnglePhase1,
        stpmParams.rmsVoltagePhase2,
        stpmParams.rmsCurrentPhase2,
        (float)stpmParams.activePowerPhase2 / 1000.0f,   //Corrected the division and cast
        (float)stpmParams.apparentPowerPhase2 / 1000.0f,
        stpmParams.frequencyPhase2,
        stpmParams.phaseAnglePhase2,

        stpmParams.temperatureC,
        stpmParams.accelX,
        stpmParams.accelY,
        stpmParams.accelZ
    );

    //Check for `snprintf` errors.
    if (snprintf_result < 0 || snprintf_result >= MAX_STRING_LENGTH) {
        fprintf(stderr, "Formatting string failed with %d!\n", snprintf_result);
        return -1;    // Indicate failure
    }
    rpi_uart_buff_size = snprintf_result;
    return 0;
}

 /**
  * @brief Sends a dummy frame to the STPM34 energy meter.
  * 
  * This function transmits a predefined dummy frame to the STPM34 
  * device. It is typically used to ensure communication is active 
  * and to trigger any required internal processing within the STPM34.
  * 
  * @return void
  * 
  * @note The function assumes that `STPM34_DummyFrame` is defined elsewhere in the code.
  * 
  * @see send_command
  */
 
 void sendDummyFrame(void)
 {
     ESP_LOGD(STPM_TAG, "Sending Dummy Frame to STPM34");
     send_command(STPM34_DummyFrame, sizeof(STPM34_DummyFrame));
 }
 
 /**
  * @brief Latches the register to obtain the latest value from the device.
  * 
  * This function sends a command to the device to latch the register, ensuring that the latest values are captured. 
  * The latch operation is typically used in devices that require an explicit command to capture or store the current register values. 
  * This is often used to ensure that the device's latest state is reflected when performing subsequent operations or reads.
  * 
  * The function uses a predefined command buffer (`latchReg_SW`) to send the latch command to the device. 
  * After executing this function, the device is expected to hold or lock the current register values for later retrieval.
  * 
  * @note This function assumes that the latch register command (`latchReg_SW`) and its associated frame length (`FRAME_LEN`) are correctly defined elsewhere in the code.
  * 
  * @see send_command
  */
 
 void setLatchRegister(void)
 {
   ESP_LOGD(STPM_TAG,"Latching Register to obtain the latest value..");
   send_command((uint8_t*)latchReg_SW, (FRAME_LEN*2));
   // Allow STPM34 to update values
   vTaskDelay(pdMS_TO_TICKS(2));
 }
 
 
 /**
  * @brief Sets the gain value for the specified channel.
  * 
  * This function configures the gain for the selected STPM34 measurement channel 
  * by sending the corresponding gain register values. A delay is introduced 
  * to allow the STPM34 to update the gain settings.
  * 
  * @param channel The measurement channel to configure:
  *                - `1` for Channel-1
  *                - `2` for Channel-2
  * 
  * @return void
  * 
  * @note The function assumes that the following variables are defined elsewhere:
  *       - `ch1_gainReg`: The gain register values for Channel-1.
  *       - `ch2_gainReg`: The gain register values for Channel-2.
  *       - `FRAME_LEN`: The frame length used in communication.
  * 
  * @see send_command
  * @see vTaskDelay
  */
 
 void setGainValue(uint8_t channel)
 {
     if(channel == CHANNEL_ONE)
     {
         ESP_LOGI(STPM_TAG,"Setting Channel-1 Gain ");
         send_command((uint8_t*)ch1_gainReg, (FRAME_LEN*2));
         // Allow STPM34 to update values
         vTaskDelay(pdMS_TO_TICKS(2));
     }
     else if(channel == CHANNEL_TWO)
     {
         ESP_LOGI(STPM_TAG,"Setting Channel-2 Gain ");
         send_command((uint8_t*)ch2_gainReg, (FRAME_LEN*2));
         // Allow STPM34 to update values
         vTaskDelay(pdMS_TO_TICKS(2));
     }
     
 }
 
 
 /**
  * @brief Performs a dummy read operation to capture the default frame.
  * 
  * This function sends a dummy command to the device to trigger a default frame read. It is used to capture the initial frame or register value (usually 0x00) from the device. The function sends a dummy write command, waits for a short period to ensure the device has time to respond, and then reads the response into a buffer.
  * 
  * The frame read is typically used to understand the initial state or configuration of the device.
  * 
  * @note This function is primarily intended for debugging or initialization purposes, allowing the software to sync with the device and capture the initial response. The dummy frame read might capture a default or uninitialized register state (e.g., `0x00`).
  * 
  * The following constants and functions are assumed to be defined elsewhere:
  * - `FRAME_LEN`: The length of the frame being read.
  * - `send_command`: A function that sends a command to the device.
  * - `read_response`: A function that reads the response from the device.
  * - `ESP_LOGD`: A function for logging debug information.
  * - `esp_rom_delay_us`: A function for introducing a delay in microseconds.
  * 
  * @see send_command, read_response
  */
 
 void performDummyRead(void)
 {
     /* perform a dummy read and write at the first time t catch the dafaule frame, which is the value of register: 0x00*/
     uint8_t Cmd[FRAME_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0x7B};
     send_command((uint8_t *)Cmd, FRAME_LEN);
     esp_rom_delay_us(10000);
     ESP_LOGD(STPM_TAG,"Doing a dummy read to capture the sequence: {0x04, 0x00, 0x00, 0x00, 0xA0}");
     uint8_t dummy_Frame[FRAME_LEN];
     read_response((uint8_t*)dummy_Frame, FRAME_LEN);
 }
 
 
 
 /**
  * @brief Reads consecutive registers starting from a specified register and stores the data in a buffer.
  * 
  * This function reads a sequence of consecutive registers starting from a given register address. It first reads the data from the starting register, then reads the specified number of consecutive registers using dummy read frames and stores all the read data into the provided buffer.
  * 
  * The function logs the register data for debugging purposes, and the read data is stored in the provided `rxBuffer`.
  * 
  * @param startRegister The register address to start reading from.
  * @param count The number of consecutive registers to read.
  * @param rxBuffer A pointer to the buffer where the register data will be stored. The buffer should be large enough to hold all the data.
  * 
  * @note The function performs the following operations:
  * 1. Reads the data from the starting register and copies it into the buffer.
  * 2. For each subsequent register, a dummy read is performed, and the data is stored in the buffer.
  * 3. The buffer is populated sequentially with the read data from each register.
  * 
  * The following constants and functions are assumed to be defined elsewhere:
  * - `FRAME_LEN`: The length of each register's data in bytes.
  * - `readRegister`: A function that reads data from a specific register.
  * - `DUMMY_BYTE`: A predefined constant used for dummy reads.
  * - `ESP_LOGD`: A function for logging debug information.
  * 
  * @see readRegister
  */
 
 void readConsecutiveRegistersToBuffer(uint8_t startRegister, uint8_t count, uint8_t *rxBuffer) 
 {
     ESP_LOGD(STPM_TAG, "Reading %d consecutive registers starting from 0x%02X", count, startRegister);
     
     // Read the first register
     uint8_t* firstRegisterData = readRegister(startRegister, false);
     // Copy the first register data to the buffer
     memcpy(rxBuffer, firstRegisterData, FRAME_LEN - 1);
     ESP_LOGI(STPM_TAG, "Register 0x%02X data mapped to buffer:", startRegister);
     for (int i = 0; i < FRAME_LEN - 1; i++) {
         ESP_LOGI(STPM_TAG, "0x%02X ", rxBuffer[i]);
     }
     // Perform consecutive reads using dummy frames
     for (int i = 1; i <= count; i++) {
         // Read the response into the buffer
         uint8_t* registerData = readRegister(DUMMY_BYTE, true);
         // Map the response to the RX buffer
         memcpy(&rxBuffer[i * (FRAME_LEN - 1)], registerData, FRAME_LEN - 1);
         //Log
         ESP_LOGI(STPM_TAG, "Register 0x%02X data mapped to buffer:", startRegister = startRegister + 2);
         for (int j = 0; j < FRAME_LEN - 1; j++) {
             ESP_LOGI(STPM_TAG, "0x%02X ", rxBuffer[i * (FRAME_LEN - 1) + j]);
         }
         // Add an intermittent delay between reads (1-5 ms)
         //vTaskDelay(1 / portTICK_PERIOD_MS);  // Example: 1 ms delay
         esp_rom_delay_us(100);
     }
     ESP_LOGI(STPM_TAG, "All register data mapped to buffer successfully.");
 }
 
 
 
 /*
 Read a u32 register from STPM34 */
 /**
  * @brief Read data from a specific register address.
  * 
  * This function sends a command to the device to read data from a given register address. It provides two modes of operation:
  * 1. **Dummy read mode**: Sends a dummy frame and receives the response into the receive buffer.
  * 2. **Normal read mode**: Constructs a frame with the register address, sends the command to the device, and reads the response.
  * 
  * The function also logs the data received from the device for debugging purposes.
  * 
  * @param regAddr The register address to read from.
  * @param dummyRead A boolean flag to indicate whether a dummy read is requested. If `true`, a dummy frame is sent.
  * 
  * @return uint8_t* Pointer to the received data buffer. If the read operation is successful, it returns a pointer to the `STPM34_RxBuffer`. In case of failure, the buffer will contain `0xFF` values.
  * 
  * @note The following constants and functions are assumed to be defined elsewhere:
  * - `STPM34_RxBuffer`: A buffer for storing the received data.
  * - `STPM34_TxBuffer`: A buffer used for constructing the frame to send.
  * - `DUMMY_BYTE`: A predefined constant used as a placeholder byte.
  * - `FRAME_SIZE`: The size of the frame used for communication.
  * - `reverseFrame`: A function that reverses the byte order of a given buffer.
  * - `metroHalCalcCrc8`: A function that calculates the CRC8 checksum for a data buffer.
  * - `metroHalByteReverse`: A function that reverses the bits of a byte.
  * - `send_command`: A function responsible for sending the command to the device.
  * - `read_response`: A function responsible for reading the response from the device.
  * 
  * @see ESP_LOGD
  */
 
 uint8_t* readRegister (uint8_t regAddr, bool dummyRead)
 {
     #if DEBUG_DEEP == 1 
       ESP_LOGD(STPM_TAG,"Read Address: 0x%02X", regAddr);
     #endif
     if(dummyRead)
     {
         sendDummyFrame();
         memset(STPM34_RxBuffer, 0xFF, sizeof(STPM34_RxBuffer));
         read_response(STPM34_RxBuffer, FRAME_SIZE);
         ESP_LOGD(STPM_TAG, "Response Inside readRegister Fucntion: ");
         for (uint8_t i = 0; i < sizeof(STPM34_RxBuffer); i++) 
         {
             ESP_LOGD(STPM_TAG, "0x%02X ", STPM34_RxBuffer[i]);
         }
         return (uint8_t *)STPM34_RxBuffer;
     }
     else
     {
         // Clear the Tx buffer before a transaction with STPM34
         memset(STPM34_TxBuffer, 0, sizeof(STPM34_TxBuffer));
         STPM34_TxBuffer[0] = regAddr;
         STPM34_TxBuffer[1] = DUMMY_BYTE;
         STPM34_TxBuffer[2] = DUMMY_BYTE;
         STPM34_TxBuffer[3] = DUMMY_BYTE;
     
         uint8_t reversedFrame[FRAME_LEN-1];
         reverseFrame(STPM34_TxBuffer, FRAME_LEN-1, reversedFrame);
         uint8_t calculatedCrc = metroHalCalcCrc8(reversedFrame, FRAME_LEN-1);
         calculatedCrc = metroHalByteReverse(calculatedCrc);
         ESP_LOGD(STPM_TAG,"Read Register Says: Calculated CRC: 0x");
         ESP_LOGD(STPM_TAG, "%d", calculatedCrc);
         STPM34_TxBuffer[4] = calculatedCrc;
 
         // Query STPM34
         ESP_LOGD(STPM_TAG,"Sending Command to STPM34");
         send_command(STPM34_TxBuffer, sizeof(STPM34_TxBuffer));
         // Read response from STPM34
         ESP_LOGD(STPM_TAG,"Reading STPM for data");
 
         // Set all bytes to 0xFF. In case of a Rx Failure, the caller can know this
         memset(STPM34_RxBuffer, 0xFF, sizeof(STPM34_RxBuffer));
         read_response(STPM34_RxBuffer, FRAME_SIZE);
         ESP_LOGD(STPM_TAG, "Response Inside readRegister Fucntion: ");
         for (uint8_t i = 0; i < sizeof(STPM34_RxBuffer); i++) 
         {
             ESP_LOGD(STPM_TAG, "0x%02X ", STPM34_RxBuffer[i]);
         }
         return (uint8_t *)STPM34_RxBuffer;
     }
 }
 
 
 /*Write to a u16 register in STPM34 */
 /**
  * @brief Write data to a specific register address.
  * 
  * This function prepares and sends data to a specific register address on a device. The process involves:
  * 1. Constructing a frame containing the dummy byte, register address, and the provided data.
  * 2. Reversing the byte order of the frame for transmission.
  * 3. Calculating the CRC8 for the reversed frame and appending it to the buffer.
  * 4. Sending the constructed frame with CRC8 to the device via the `send_command` function.
  * 
  * The function also logs the address and data being written for debugging purposes.
  * 
  * @param regAddr The register address to which the data is to be written.
  * @param data Pointer to a 2-byte data array that contains the data to be written.
  * 
  * @return void
  * 
  * @note The following constants and functions are assumed to be defined elsewhere:
  * - `STPM34_TxBuffer`: A buffer used for constructing the frame to send.
  * - `DUMMY_BYTE`: A predefined constant used as a placeholder byte in the frame.
  * - `FRAME_LEN`: The length of the data frame.
  * - `reverseFrame`: A function that reverses the byte order of a given buffer.
  * - `metroHalCalcCrc8`: A function that calculates the CRC8 checksum for a data buffer.
  * - `metroHalByteReverse`: A function that reverses the bits of a byte.
  * - `send_command`: A function responsible for sending the command to the device.
  * 
  * @see ESP_LOGD
  */
 
 void writeRegister(uint8_t regAddr, uint8_t* data)
 {
     
     ESP_LOGD(STPM_TAG,"Write Address: 0x%02X", regAddr);
     memset(STPM34_TxBuffer, 0, sizeof(STPM34_TxBuffer));
     STPM34_TxBuffer[0] = DUMMY_BYTE;
     STPM34_TxBuffer[1] = regAddr;
     STPM34_TxBuffer[2] = data[0];
     STPM34_TxBuffer[3] = data[1];
 
     uint8_t reversedFrame[FRAME_LEN-1];
     reverseFrame(STPM34_TxBuffer, FRAME_LEN-1, reversedFrame);
     uint8_t calculatedCrc = metroHalCalcCrc8(reversedFrame, FRAME_LEN-1);
     calculatedCrc = metroHalByteReverse(calculatedCrc);
     STPM34_TxBuffer[4] = calculatedCrc;
 
     ESP_LOGD(STPM_TAG,"Writing data : ");
     ESP_LOGD(STPM_TAG,"0x%02X, 0x%02X", data[0], data[1]);
     ESP_LOGD(STPM_TAG," , to register: ");
     ESP_LOGD(STPM_TAG, "0x%02X", regAddr);
     send_command((uint8_t*)STPM34_TxBuffer, sizeof(STPM34_TxBuffer));
     ESP_LOGD(STPM_TAG,"Data Written to register, Successfully");
 
 }
 
 /**
  * @brief Flushes both the transmit and receive buffers of the UART.
  * 
  * This function clears any pending data in the UART input and output buffers 
  * to ensure clean communication. It helps prevent data corruption due to 
  * stale or residual bytes in the buffers.
  * 
  * @return void
  * 
  * @note The function assumes that `STPM_UART` is defined elsewhere in the code.
  * 
  * @see uart_flush_input
  * @see uart_flush
  */
 
 
 void flush_uart_tx_rx_buff(void)
 {
     /* Flush the input and output buffers of the UART*/
     uart_flush_input(STPM_UART);
     uart_flush(STPM_UART);
 }
 
 
 
 /**
  * @brief Parse and calculate the RMS voltage and current from the raw data buffer for a given phase.
  * 
  * This function processes the raw RMS data (received in 4 bytes) by:
  * 1. Combining the 4 bytes from the provided buffer into a 32-bit raw value.
  * 2. Extracting the RMS voltage and current values using bitmasking and bit shifting.
  * 3. Scaling the RMS voltage and current values by multiplying them by predefined factors 
  *    (`CONFIG_VOLTAGE_MULTI_FACTOR` and `CONFIG_CURRENT_MULTI_FACTOR` respectively), followed by additional scaling.
  * 4. Performing bit shifts to adjust the scaled values.
  * 5. Storing the final RMS voltage and current values in the `stpmParams` structure 
  *    for the corresponding phase (`rmsVoltagePhase1`, `rmsCurrentPhase1`, 
  *    `rmsVoltagePhase2`, `rmsCurrentPhase2`).
  * 
  * Additionally:
  * - For Phase-2 RMS voltage, if the calculated value is below a defined threshold (`VRMS_PHASE2_MIN_THRESHOLD`), 
  *   it is set to `0.000f` and logged accordingly.
  * - For Phase-2 RMS current, if the Phase-2 RMS voltage has been set to zero, the RMS current is also set to zero.
  *
  * @param[in] rmsBuff Pointer to a buffer containing the 4 raw bytes of RMS data.
  * @param[in] phase_number The phase for which RMS values are being calculated (1 or 2).
  *
  * @return void
  *
  * @note The function assumes that the following constants and variables are defined elsewhere in the code:
  * - `BIT_MASK_STPM_DATA_VRMS`: The bitmask used to extract the RMS voltage value.
  * - `CONFIG_VOLTAGE_MULTI_FACTOR`: The factor by which the raw RMS voltage is multiplied to scale it.
  * - `CONFIG_CURRENT_MULTI_FACTOR`: The factor by which the raw RMS current is multiplied to scale it.
  * - `VRMS_PHASE2_MIN_THRESHOLD`: The minimum threshold for valid Phase-2 RMS voltage.
  * - `stpmParams.rmsVoltagePhase1`: The global variable where the calculated RMS voltage for Phase-1 is stored.
  * - `stpmParams.rmsCurrentPhase1`: The global variable where the calculated RMS current for Phase-1 is stored.
  * - `stpmParams.rmsVoltagePhase2`: The global variable where the calculated RMS voltage for Phase-2 is stored.
  * - `stpmParams.rmsCurrentPhase2`: The global variable where the calculated RMS current for Phase-2 is stored.
  *
  * @see ESP_LOGI, ESP_LOGD
 */

 
 
 void parseRMS(uint8_t* rmsBuff, uint8_t phase_number)
 {
     uint32_t rawRMS = 0;
     uint64_t tempRMS =0;
     //rmsvoltage = (uint32_t)0;
     //rmscurrent = (uint32_t)0;
     
     uint32_t rmsvoltage = 0;
     uint32_t rmscurrent = 0;
 
     // Extract raw RMS data from the buffer
     /* rawRMS |= (uint32_t)rmsBuff[0] << 24;  // First byte to the highest byte of uint32_t
     rawRMS |= (uint32_t)rmsBuff[1] << 16;  // Second byte to the second highest byte
     rawRMS |= (uint32_t)rmsBuff[2] << 8;   // Third byte to the second lowest byte
     rawRMS |= (uint32_t)rmsBuff[3];        // Fourth byte to the lowest byte
    */
    rawRMS = reverse_bytes(rmsBuff);

     ESP_LOGI(STPM_TAG, "Raw RMS from phase %d:", phase_number);
     ESP_LOGI(STPM_TAG, "%lu", rawRMS);
 
     // RMS Voltage Calculation
     rmsvoltage = rawRMS & (uint32_t)(BIT_MASK_STPM_DATA_VRMS);
     //ESP_LOGD(STPM_TAG, "RMSVoltage After BitMasking: %llu", rmsvoltage);
     ESP_LOGD(STPM_TAG, "RMSVoltage After BitMasking: %lu", rmsvoltage);
 
     tempRMS = (uint64_t)(rmsvoltage * CONFIG_VOLTAGE_MULTI_FACTOR);
     tempRMS *= 10;
     ESP_LOGD(STPM_TAG, "RMSVoltage After Multiplying factor: %llu", tempRMS);
 
     tempRMS >>= 15;
     ESP_LOGD(STPM_TAG, "RMSVoltage After Bitshifting: %llu", tempRMS);
 
     // Update stpmParams based on phase_number
     if (phase_number == 1)
     {
         ESP_LOGI(STPM_TAG, "RMS Voltage for Phase 1: %3.3f V", (float)(tempRMS) / 1000.0f);
         stpmParams.rmsVoltagePhase1 = (float)(tempRMS) / 1000.0f;
     }
     else if (phase_number == 2)
     {
         float calc_RMS_Volt_Phase2 = (float)(tempRMS) / 1000.0f;
         ESP_LOGI(STPM_TAG, "RMS Voltage for Phase 2: %3.3f V", calc_RMS_Volt_Phase2);
         if(calc_RMS_Volt_Phase2 < VRMS_PHASE2_MIN_THRESHOLD)
         {
            calc_RMS_Volt_Phase2 = 0.000f;
            ESP_LOGI(STPM_TAG, "RMS Voltage measured for Phase-2 less than RMS_THRESHOLD");
            
         }
         stpmParams.rmsVoltagePhase2 = calc_RMS_Volt_Phase2;
     }
 
     // RMS Current Calculation
     rmscurrent = (uint32_t)((rawRMS & 0xFFFF8000) >> 15);
     //ESP_LOGD(STPM_TAG, "RMSCurrent After BitMasking: %llu", rmscurrent);
     ESP_LOGD(STPM_TAG, "RMSCurrent After BitMasking: %lu", rmscurrent);
 
     tempRMS = (uint64_t)(rmscurrent * CONFIG_CURRENT_MULTI_FACTOR);
     tempRMS *= 10;
     ESP_LOGD(STPM_TAG, "RMSCurrent After Multiplying factor: %llu", tempRMS);
     //tempRMS = (uint32_t)(tempRMS >> 17);
     tempRMS = tempRMS >> 17;
 
     // Update stpmParams based on phase_number
     if (phase_number == 1)
     {
         stpmParams.rmsCurrentPhase1 = (float)(tempRMS) / 1000.0f;
         ESP_LOGI(STPM_TAG, "RMS Current for Phase 1: %3.3f A", stpmParams.rmsCurrentPhase1);
     }
     else if (phase_number == 2)
     {
         // Validate the Vrms based on the defined lower threshold limit
         stpmParams.rmsCurrentPhase2 = (float)(tempRMS) / 1000.0f;
         if(stpmParams.rmsVoltagePhase2 == 0)
         {
            stpmParams.rmsCurrentPhase2 = 0;
         }
         ESP_LOGI(STPM_TAG, "RMS Current for Phase 2: %3.3f A", stpmParams.rmsCurrentPhase2);
     }
 }
 
 /**
 * @brief Parses and calculates the active power for a given phase from raw STPM34 register data.
 *
 * This function processes the raw active power data (received in 4 bytes) by:
 * 1. Combining the 4 bytes from the provided buffer into a 32-bit raw value.
 * 2. Extracting the active power value using bitmasking (BIT_MASK_STPM_PRIM_CURR_ACTIVE_POW).
 * 3. Performing sign extension as the power data is 28-bit signed.
 * 4. Scaling the active power value by multiplying it by a predefined factor (CONFIG_POWER_FACTOR_MULTIPLE).
 * 5. Performing bit shifts to adjust the scaled value.
 * 6. Storing the final active power value in the `stpmParams` structure 
 *    for the corresponding phase (`activePowerPhase1` or `activePowerPhase2`).
 *
 * Additional features:
 * - Negative power values are clamped to zero.
 * - For Phase-2, active power is set to zero if either RMS voltage or RMS current for Phase-2 is zero.
 *
 * @param[in] actPowBuff Pointer to a buffer containing the 4 raw bytes of active power data.
 * @param[in] phase_number The phase for which active power is being calculated (1 or 2).
 *
 * @return void
 *
 * @note The function assumes that the following constants and variables are defined elsewhere in the code:
 * - `BIT_MASK_STPM_PRIM_CURR_ACTIVE_POW`: The bitmask used to extract the active power value.
 * - `CONFIG_POWER_FACTOR_MULTIPLE`: The factor by which the raw active power is multiplied to scale it.
 * - `stpmParams.activePowerPhase1`: The global variable where the calculated active power for Phase-1 is stored.
 * - `stpmParams.activePowerPhase2`: The global variable where the calculated active power for Phase-2 is stored.
 * - `stpmParams.rmsVoltagePhase2`: The global variable storing the RMS voltage for Phase-2.
 * - `stpmParams.rmsCurrentPhase2`: The global variable storing the RMS current for Phase-2.
 *
 * @see ESP_LOGI, ESP_LOGD
 */

 
 void parseActPower(uint8_t* actPowBuff, uint8_t phase_number)
 {
     int32_t result=0;
     uint32_t rawActpower = 0;
     int32_t powerActive = 0;
     int64_t tempActPow;
 
    /* rawActpower |= (uint32_t)actPowBuff[0] << 24;  // First byte to the highest byte of uint32_t
     rawActpower |= (uint32_t)actPowBuff[1] << 16;  // Second byte to the second highest byte
     rawActpower |= (uint32_t)actPowBuff[2] << 8;   // Third byte to the second lowest byte
     rawActpower |= (uint32_t)actPowBuff[3];
    */
    rawActpower = reverse_bytes(actPowBuff);
     ESP_LOGI(STPM_TAG,"Raw Active power for Phase %d: ", phase_number);
     ESP_LOGI(STPM_TAG, "%lu", rawActpower);
 
     powerActive = rawActpower & BIT_MASK_STPM_PRIM_CURR_ACTIVE_POW;
     ESP_LOGD(STPM_TAG, "Active Power Before BitMasking: %lu", powerActive);
     // Handle sign extension as power is 28 bits
     powerActive <<= 4;
     powerActive >>= 4;
     ESP_LOGD(STPM_TAG, "Active Power After BitMasking: %lu", powerActive);
 
     tempActPow = (int64_t)powerActive * CONFIG_POWER_FACTOR_MULTIPLE;
     tempActPow *= 10;
     ESP_LOGD(STPM_TAG, "Active Power After Multiplying factor: %llu", tempActPow);
 
     tempActPow >>= 28;
     result = (int32_t)tempActPow;
     ESP_LOGD(STPM_TAG, "Active Power After Bitshifting: %lu", result);
     if (result < 0)
     {
         result = 0;
     }
     if (phase_number == 1)
     {
         stpmParams.activePowerPhase1 = result;
     }
     else if(phase_number == 2)
     {
         stpmParams.activePowerPhase2 = result;
     }

     //Make active power zero to avoid noise being falsely read as active power when there is no supply on Phase-2
     if((stpmParams.rmsVoltagePhase2 == 0) || (stpmParams.rmsCurrentPhase2 == 0))
     {
        stpmParams.activePowerPhase2 = 0;
     }
     //stpmParams.activePower = result;
     //ESP_LOGD(STPM_TAG,"Active Power: %.3f W", (float)result/1000.0f);
 }
 
/**
 * @brief Parses and calculates the apparent power for a given phase from raw STPM34 register data.
 *
 * This function processes the raw apparent power data (received in 4 bytes) by:
 * 1. Combining the 4 bytes from the provided buffer into a 32-bit raw value.
 * 2. Extracting the apparent power value using bitmasking (BIT_MASK_STPM_PRIM_CURR_ACTIVE_POW).
 * 3. Performing sign extension as the power data is 28-bit signed.
 * 4. Scaling the apparent power value by multiplying it by a predefined factor (CONFIG_POWER_FACTOR_MULTIPLE).
 * 5. Performing bit shifts to adjust the scaled value.
 * 6. Storing the final apparent power value in the `stpmParams` structure 
 *    for the corresponding phase (`apparentPowerPhase1` or `apparentPowerPhase2`).
 *
 * Additional features:
 * - Negative power values are clamped to zero.
 * - For Phase-2, apparent power is set to zero if either RMS voltage or RMS current for Phase-2 is zero.
 *
 * @param[in] appPowBuff Pointer to a buffer containing the 4 raw bytes of apparent power data.
 * @param[in] phase_number The phase for which apparent power is being calculated (1 or 2).
 *
 * @return void
 *
 * @note The function assumes that the following constants and variables are defined elsewhere in the code:
 * - `BIT_MASK_STPM_PRIM_CURR_ACTIVE_POW`: The bitmask used to extract the apparent power value.
 * - `CONFIG_POWER_FACTOR_MULTIPLE`: The factor by which the raw apparent power is multiplied to scale it.
 * - `stpmParams.apparentPowerPhase1`: The global variable where the calculated apparent power for Phase-1 is stored.
 * - `stpmParams.apparentPowerPhase2`: The global variable where the calculated apparent power for Phase-2 is stored.
 * - `stpmParams.rmsVoltagePhase2`: The global variable storing the RMS voltage for Phase-2.
 * - `stpmParams.rmsCurrentPhase2`: The global variable storing the RMS current for Phase-2.
 *
 * @see ESP_LOGI, ESP_LOGD
 */



 void parseAppPower(uint8_t* appPowBuff, uint8_t phase_number)
 {
     int32_t result=0;
     uint32_t rawApppower = 0;
     int32_t powerApparent = 0;
     int64_t tempAppPow;
 
     /* rawApppower |= (uint32_t)appPowBuff[0] << 24;  // First byte to the highest byte of uint32_t
     rawApppower |= (uint32_t)appPowBuff[1] << 16;  // Second byte to the second highest byte
     rawApppower |= (uint32_t)appPowBuff[2] << 8;   // Third byte to the second lowest byte
     rawApppower |= (uint32_t)appPowBuff[3];
    */

    rawApppower = reverse_bytes(appPowBuff);

     ESP_LOGI(STPM_TAG,"Raw Apparent power for Phase %d: ", phase_number);
     ESP_LOGI(STPM_TAG, "%lu", rawApppower);
 
     powerApparent = rawApppower & BIT_MASK_STPM_PRIM_CURR_ACTIVE_POW;
     ESP_LOGD(STPM_TAG, "Apparent Power Before BitMasking: %lu", powerApparent);
     // Handle sign extension as power is 28 bits
     powerApparent <<= 4;
     powerApparent >>= 4;
     ESP_LOGD(STPM_TAG, "Apparent Power After BitMasking: %lu", powerApparent);
 
     tempAppPow = (int64_t)powerApparent * CONFIG_POWER_FACTOR_MULTIPLE;
     tempAppPow *= 10;
     ESP_LOGD(STPM_TAG, "Apparent Power After Multiplying factor: %llu", tempAppPow);
 
     tempAppPow >>= 28;
     result = (int32_t)tempAppPow;
     ESP_LOGD(STPM_TAG, "Apparent Power After Bitshifting: %lu", result);
     if (result < 0)
     {
         result = 0;
     }
     if (phase_number == 1)
     {
         stpmParams.apparentPowerPhase1 = result;
     }
     else if(phase_number == 2)
     {
         stpmParams.apparentPowerPhase2 = result;
     }

      //Make apparent power zero to avoid noise being falsely read as apparent power when there is no supply on Phase-2
     if((stpmParams.rmsVoltagePhase2 == 0) || (stpmParams.rmsCurrentPhase2 == 0))
     {
        stpmParams.apparentPowerPhase2 = 0;
     }
     //stpmParams.activePower = result;
     //ESP_LOGD(STPM_TAG,"Active Power: %.3f W", (float)result/1000.0f);
 }
 
 // Function to initialize UART
 /**
  * @brief Initialize the UART peripheral for communication.
  * 
  * This function configures and initializes the UART interface with the necessary parameters 
  * for communication with the STPM3 device. It sets up the UART configuration, installs the 
  * UART driver, and assigns the UART pins for transmission and reception.
  * 
  * The following parameters are configured:
  * - Baud rate: Defined by the `STPM3_BITRATE` constant.
  * - Data bits: 8 bits.
  * - Parity: Disabled.
  * - Stop bits: 1 stop bit.
  * - Flow control: Disabled (hardware flow control is turned off).
  * 
  * The UART driver is installed with a buffer size of `BUF_SIZE * 2` (buffer size can be adjusted if needed).
  * The function also sets the UART pins for the TX and RX lines using the `StpmTxPin` and `StpmRxPin`.
  * After configuration, a debug log is generated to indicate successful initialization.
  * 
  * @note This function assumes that the `STPM3_BITRATE`, `StpmTxPin`, and `StpmRxPin` constants 
  *       are properly defined elsewhere in the code.
  * 
  * @return void
  * 
  * @see uart_driver_install, uart_param_config, uart_set_pin
  */
 
 void stpm_uart_init(void) 
 {

     // Set UART parameters
     uart_driver_install(STPM_UART, BUF_SIZE * 2, 0, 0, NULL, 0); // Increase buffer if necessary
     uart_param_config(STPM_UART, &stpm_uart_config);
     uart_set_pin(STPM_UART, StpmTxPin, StpmRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
     ESP_LOGD(STPM_TAG, "STPM34 UART Init Done");
 }


 /**
 * @brief Initializes the UART for communication with Raspberry Pi.
 *
 * This function sets up the UART driver for communication with a Raspberry Pi.
 * It configures the UART parameters, installs the driver, and sets the appropriate pins.
 *
 * @details The function performs the following steps:
 * 1. Installs the UART driver with a specified buffer size.
 * 2. Configures the UART parameters using a predefined configuration.
 * 3. Sets the TX and RX pins for the UART communication.
 *
 * @note The buffer size is set to RPI_UART_BUFF_SIZE * 2. Increase if necessary.
 *
 * @see uart_driver_install()
 * @see uart_param_config()
 * @see uart_set_pin()
 *
 * @return void
 */

 void rpi_uart_init(void)
 {
     uart_driver_install(RPI4_UART, RPI_UART_BUFF_SIZE * 2, 0, 0, NULL, 0); // Increase buffer if necessary
     uart_param_config(RPI4_UART, &rpi_uart_config);
     uart_set_pin(RPI4_UART, RpiTxPin, RpiRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
     ESP_LOGD(STPM_TAG, "Raspberry Pi UART Init Done");
 }
 
 /**
  * @brief Read and process all electrical parameters from registers.
  * 
  * This function reads data from consecutive registers, updates the internal data buffers, 
  * and logs the current electrical parameters. The function performs the following actions:
  * 1. Clears the `RxBuff` buffer by setting all bytes to 0xFF.
  * 2. Sets the latch register to prepare for reading.
  * 3. Reads data from consecutive registers starting from `UART_SPI_STS_CTRLREG` and stores it in the `RxBuff` buffer.
  * 4. Updates the parsed data by calling the `updateData` function, which processes the read data.
  * 5. Logs the current electrical parameters (VRMS, IRMS, and Active Power) using the `logData` function.
  * 
  * @note This function relies on the `setLatchRegister`, `readConsecutiveRegistersToBuffer`, 
  *       `updateData`, and `logData` functions to perform its tasks.
  * 
  * @return void
  * 
  * @see setLatchRegister, readConsecutiveRegistersToBuffer, updateData, logData
  */
 
 void readAll_Params(void)
 {
     memset(RxBuff, 0xFF, sizeof(RxBuff));
     setLatchRegister();
     readConsecutiveRegistersToBuffer(START_REG_ADDR, BLOCKS_TO_READ, RxBuff);
     updateData();
     logData();
 }
 
 /**
 * @brief Logs the current electrical parameters for both phases.
 *
 * This function prints the following parameters for each phase to the log:
 * - RMS Voltage (V)
 * - RMS Current (A)
 * - Active Power (W)
 * - Apparent Power (W)
 *
 * @note The function uses the global stpmParams structure to access the parameter values.
 * @note Active and Apparent Power values are converted from milliwatts to watts for display.
 * @note This function is typically called periodically to log the current state of the electrical system.
 */

 
 void logData(void)
 {
     //ESP_LOGI(STPM_TAG, ": VRMS-P1 :%.3f V, IRMS-P1: %.3f A, ActivePower-P1: %.3f W", stpmParams.rmsVoltagePhase1, stpmParams.rmsCurrentPhase1, (float)stpmParams.activePowerPhase1/1000.0f);
     //ESP_LOGI(STPM_TAG, ": VRMS-P2 :%.3f V, IRMS-P2: %.3f A, ActivePower-P2: %.3f W", stpmParams.rmsVoltagePhase2, stpmParams.rmsCurrentPhase2, (float)stpmParams.activePowerPhase2/1000.0f);
     // clean up below
     ESP_LOGI(STPM_TAG, ": VRMS-P1 :%.3f V, IRMS-P1: %.3f A, ActivePower-P1: %.3f W, ApparentPower-P1: %.3f W", stpmParams.rmsVoltagePhase1, stpmParams.rmsCurrentPhase1, (float)stpmParams.activePowerPhase1/1000.0f, (float)stpmParams.apparentPowerPhase1/1000.0f);
     ESP_LOGI(STPM_TAG, ": VRMS-P2 :%.3f V, IRMS-P2: %.3f A, ActivePower-P2: %.3f W, ApparentPower-P2: %.3f W", stpmParams.rmsVoltagePhase2, stpmParams.rmsCurrentPhase2, (float)stpmParams.activePowerPhase2/1000.0f, (float)stpmParams.apparentPowerPhase2/1000.0f);

 } 
 
 /**
 * @brief Updates all electrical parameters for both phases from STPM34 data.
 *
 * This function processes the raw data buffer from the STPM34 chip and calculates:
 * - Line frequency for both phases
 * - RMS voltage and current for both phases
 * - Active power for both phases
 * - Apparent power for both phases
 * - Phase angle for both phases
 *
 * The function performs the following steps:
 * 1. Calculates line frequency for both phases.
 * 2. Processes Phase-1 parameters: RMS values, active power, apparent power, and phase angle.
 * 3. Processes Phase-2 parameters, with additional checks:
 *    - If Phase-2 RMS voltage is zero, it sets all Phase-2 parameters to zero.
 *    - Otherwise, it calculates active power, apparent power, and phase angle for Phase-2.
 *
 * @note Uses global RxBuff for input data and updates the global stpmParams structure.
 * @note Calls several helper functions for specific calculations (e.g., parseRMS, calculate_phase_angle).
 * @note Energy calculation is currently commented out and not implemented.
 *
 * @return void
 *
 * @see rearrangeBytes, calculate_period_seconds, parseRMS, parseActPower, parseAppPower, calculate_phase_angle
 */


 
 void updateData(void)
 {
     uint8_t res[4];
     uint8_t *dataBuff = RxBuff;
     
     memset(res, 0xFF, sizeof(res));
     double frequency;
     
     /* Calcualate Phase-1 line frequency */
     rearrangeBytes(dataBuff, res, FRAME_LEN-1, PERIOD_REG_STA_INDEX);
     frequency = calculate_period_seconds((uint8_t*)res, PHASE_ONE_NUMBER);
     ESP_LOGI(STPM_TAG, "Phase-1 Frequency: %.3f Hz\n", frequency);
     
     /* Calcualate Phase-2 line frequency */
     frequency = calculate_period_seconds((uint8_t*)res, PHASE_TWO_NUMBER);
     ESP_LOGI(STPM_TAG, "Phase-2 Frequency: %.3f Hz\n", frequency);

     /* Calculate Phase-1 RMS voltage and current */
     memset(res, 0xFF, sizeof(res));
     rearrangeBytes(dataBuff, res, FRAME_LEN-1, RMSDATASTARTIND_PH1);
     parseRMS((uint8_t*)res, 1);

     /* Calculate Phase-1 Active Power */
     memset(res, 0xFF, sizeof(res));
     rearrangeBytes(dataBuff, res, FRAME_LEN-1, ACTPWRDATASTARTIND_PH1);
     parseActPower((uint8_t*)res, 1);

     /* Calculate Phase-1 Apparent Power */
     memset(res, 0xFF, sizeof(res));
     rearrangeBytes(dataBuff, res, FRAME_LEN-1, APPPWRDATASTARTIND_PH1);
     parseAppPower((uint8_t*)res, 1);

     /* Calculate Phase Angle for Phase-1 */
     memset(res, 0xFF, sizeof(res));
     rearrangeBytes(dataBuff, res, FRAME_LEN-1, PH_ANG_STA_INDEX_PH1);
     calculate_phase_angle(PHASE_ONE_NUMBER, (uint8_t*)res, stpmParams.frequencyPhase1);
     
     /* Calculate Phase-2 RMS voltage and current */
     memset(res, 0xFF, sizeof(res));
     rearrangeBytes(dataBuff, res, FRAME_LEN-1, RMSDATASTARTIND_PH2);
     parseRMS((uint8_t*)res, 2);

     // Check if Phase 2 voltage exceeded minimum rms threshold by validating it against 0
    if (stpmParams.rmsVoltagePhase2 == 0.0f)
    {
        // Set other Phase 2 parameters to 0
        stpmParams.frequencyPhase2 = 0.0;
        stpmParams.phaseAnglePhase2 = 0.0f;
        stpmParams.activePowerPhase2 = 0;
        stpmParams.apparentPowerPhase2 = 0;
    }
    else
    {
        /* Calculate Phase Angle for Phase-2 */
        memset(res, 0xFF, sizeof(res));
        rearrangeBytes(dataBuff, res, FRAME_LEN-1, PH_ANG_STA_INDEX_PH2);
        calculate_phase_angle(PHASE_TWO_NUMBER, (uint8_t*)res, stpmParams.frequencyPhase2);
 
        /* Calculate Phase-2 Active Power */
        memset(res, 0xFF, sizeof(res));
        rearrangeBytes(dataBuff, res, FRAME_LEN-1, ACTPWRDATASTARTIND_PH2);
        parseActPower((uint8_t*)res, 2);
    
        // Clean up below
        memset(res, 0xFF, sizeof(res));
        rearrangeBytes(dataBuff, res, FRAME_LEN-1, APPPWRDATASTARTIND_PH2);
        parseAppPower((uint8_t*)res, 2);
    }

     /*
     For now, energy is not needed
     memset(res, 0xFF, sizeof(res));
     rearrangeBytes(dataBuff, res, FRAME_LEN-1, ENEDATASTARTIND);
     parseActEnergy((uint8_t*)res, 1);*/  


     stpmParams.accelX = g_accel_x;
     stpmParams.accelY = g_accel_y;
     stpmParams.accelZ = g_accel_z;
 
 }



















 // === KX134 Functions ===
esp_err_t kx134_init(i2c_master_dev_handle_t dev)
{
    esp_err_t ret;
    //uint8_t data;

    // Set CNTL1 to standby (PC1 = 0) to configure
    
    uint8_t tx1[2] = {KX134_CNTL1, 0x00};
    ret = i2c_master_transmit(dev, tx1, sizeof(tx1), I2C_MASTER_TIMEOUT_MS);

    if (ret != ESP_OK) return ret;

    // Set output data rate
    uint8_t tx2[2] = {KX134_ODCNTL, 0x02};
    ret = i2c_master_transmit(dev, tx2, sizeof(tx2), I2C_MASTER_TIMEOUT_MS);

    if (ret != ESP_OK) return ret;

    // Set CNTL1 to operating mode with 2g range, low noise, high resolution
    uint8_t tx3[2] = {KX134_CNTL1, 0xC1};
    ret = i2c_master_transmit(dev, tx3, sizeof(tx3), I2C_MASTER_TIMEOUT_MS);
    return ret;
}

esp_err_t kx134_read_xyz(i2c_master_dev_handle_t dev, int16_t *x, int16_t *y, int16_t *z)
{
    esp_err_t ret;
    uint8_t reg = KX134_XOUT_L;
    uint8_t data[6];

    ret = i2c_master_transmit(dev, &reg, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) return ret;

    ret = i2c_master_receive(dev, data, sizeof(data), I2C_MASTER_TIMEOUT_MS);
    if(ret != ESP_OK) return ret;

    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
    return ESP_OK;
}

// === Sensor Task ===
static void sensor_task(void *pvParameters)
{

    ESP_LOGI(SENSOR_TAG, "Sensor monitoring task started");
    while(1){
        g_current_temperature = tmp117_read_temp_c(&tmp117_device);

        esp_err_t accel_ret = kx134_read_xyz(kx134_dev_handle, &g_accel_x, &g_accel_y, &g_accel_z);

        if(!isnan(g_current_temperature) && accel_ret == ESP_OK) {
            ESP_LOGI(SENSOR_TAG, "Temp: %.2f | Accel: X = %d, Y= %d, Z = %d", g_current_temperature, g_accel_x, 
            g_accel_y, g_accel_z);
        } else {
            ESP_LOGW(SENSOR_TAG, "Sensor read failed. Temp: %s, Accel: %s", isnan(g_current_temperature) ? "FAILED" : "OK",
                                                                            accel_ret == ESP_OK ? "OK" : "FAILED");
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }

}
    


 /**
 * @brief FreeRTOS task for periodically reading and logging electrical parameters.
 *
 * This task performs the following operations in a continuous loop:
 * 1. Initializes the STPM UART
 * 2. Flushes UART buffers
 * 3. Clears the stpmParams structure
 * 4. Sets gain values for both phases
 * 5. Reads all electrical parameters
 * 6. Disables the UART
 * 7. Formats and logs the parameters to Raspberry Pi
 * 8. Sleeps for a defined interval
 *
 * @param param Pointer to task parameters (unused)
 *
 * @note This task runs indefinitely and is typically created during system initialization.
 * @note The task uses a global stpmParams structure to store the read parameters.
 * @note Communication with the Raspberry Pi is done via UART.
 * @note The task sleeps for SLEEP_INTERVAL between iterations.
 */

 
 void read_electrical_params_task(void *param) 
 {
    char formatted_string[MAX_STRING_LENGTH];
     while (1) 
     {
         // Enable the peripheral
         stpm_uart_init();
         // Flush the Tx and Rx Buffers UART
         flush_uart_tx_rx_buff();
         // Help it to settle
         vTaskDelay(pdMS_TO_TICKS(50));
         // Clear structure before reading new values
         memset(&stpmParams, 0, sizeof(STPM34Params));
         setGainValue(1);
         vTaskDelay(pdMS_TO_TICKS(1000));
         flush_uart_tx_rx_buff();
         setGainValue(2);
         vTaskDelay(pdMS_TO_TICKS(1000));
         flush_uart_tx_rx_buff();
         // Read all parameters. VRms, IRms and ActivePower
         readAll_Params();
         
         // Read temperature
         float temperature;
         if(read_temperature_TMP117(&temperature) == 0){
            stpmParams.temperatureC = temperature;
         } else {
            stpmParams.temperatureC = -999.9f; // error value
         }

         // Read KX134
         int16_t x_raw, y_raw, z_raw;
         if(read_accel_kx134(&x_raw, &y_raw, &z_raw) == 0){
            stpmParams.accelX = x_raw / 16384.0f; // convert to g
            stpmParams.accelY = y_raw / 16384.0f;
            stpmParams.accelZ = z_raw / 16384.0f;
         } else {
            stpmParams.accelX = stpmParams.accelY = stpmParams.accelZ = -999.9f; // error value
         }

         // Disable the peripheral
         uart_driver_delete(STPM_UART);
         //Send data to RPi UART
         int res = formatSTPMParams(stpmParams, formatted_string);
         if (res == 0)
         {
            log_to_rpi((uint8_t*) formatted_string, rpi_uart_buff_size);
         }
         //Delay
         vTaskDelay(pdMS_TO_TICKS(SLEEP_INTERVAL));
     }
 }
 // Add this debug function to your main_to_change.c file
// Call it in app_main() to see what values your macros are producing


/**
 * @brief Debugs and validates STPM34 calculation macros by performing step-by-step calculations.
 *
 * This function validates the mathematical correctness of the STPM34 configuration macros
 * (CALC_VOLTAGE_MULTI_FACTOR, CALC_CURRENT_MULTI_FACTOR, CALC_POWER_MULTI_FACTOR) by:
 * - Displaying all hardware constants and CT configuration parameters
 * - Performing manual step-by-step calculations for voltage, current, and power factors
 * - Comparing macro results against expected hardcoded values
 * - Providing a pass/fail summary for each calculation
 *
 * @note This function is intended for development and debugging purposes only.
 *       It generates extensive log output showing intermediate calculation steps.
 *
 * @note Expected hardcoded values are:
 *       - Voltage factor: 116274
 *       - Current factor: 4214  
 *       - Power factor: 4900123
 *
 * @warning This function should be removed or disabled in production builds
 *          to avoid excessive logging overhead.
 *
 * @see CALC_VOLTAGE_MULTI_FACTOR()
 * @see CALC_CURRENT_MULTI_FACTOR()
 * @see CALC_POWER_MULTI_FACTOR()
 *
 * @return void
 */

void debug_calculation_macros(void) 
{
    ESP_LOGI(STPM_TAG, "=== DEBUGGING CALCULATION MACROS ===");
    
    // Print all constants first
    ESP_LOGI(STPM_TAG, "Hardware Constants:");
    ESP_LOGI(STPM_TAG, "VREF = %.3f", VREF);
    ESP_LOGI(STPM_TAG, "R1 = %.1f", R1);
    ESP_LOGI(STPM_TAG, "R2 = %.1f", R2);
    ESP_LOGI(STPM_TAG, "AV = %.1f", AV);
    ESP_LOGI(STPM_TAG, "AI = %.1f", AI);
    ESP_LOGI(STPM_TAG, "CAL_V = %.3f", CAL_V);
    ESP_LOGI(STPM_TAG, "CAL_I = %.3f", CAL_I);
    ESP_LOGI(STPM_TAG, "KINT = %.1f", KINT);
    
    // Print CT configuration
    ESP_LOGI(STPM_TAG, "CT Configuration:");
    ESP_LOGI(STPM_TAG, "CT_MAX_RATING_CURRENT_AMPS = %.1f", CT_MAX_RATING_CURRENT_AMPS);
    ESP_LOGI(STPM_TAG, "CT_MAX_RATING_MILLI_VOLTS = %d", CT_MAX_RATING_MILLI_VOLTS);
    ESP_LOGI(STPM_TAG, "CT_SENSITIVITY = %.3f", CT_SENSITIVITY);
    
    ESP_LOGI(STPM_TAG, "");
    ESP_LOGI(STPM_TAG, "=== VOLTAGE FACTOR CALCULATION ===");
    
    // Step-by-step voltage calculation
    double r_ratio = 1 + (R1 / R2);
    double av_cal_v = AV * CAL_V;
    double voltage_factor_float = (VREF * r_ratio) / av_cal_v;
    double voltage_factor_scaled = voltage_factor_float * 100.0;
    uint32_t voltage_factor_final = (uint32_t)voltage_factor_scaled;
    
    ESP_LOGI(STPM_TAG, "R1/R2 = %.6f", R1/R2);
    ESP_LOGI(STPM_TAG, "1 + (R1/R2) = %.6f", r_ratio);
    ESP_LOGI(STPM_TAG, "AV * CAL_V = %.6f", av_cal_v);
    ESP_LOGI(STPM_TAG, "VREF * (1 + R1/R2) = %.6f", VREF * r_ratio);
    ESP_LOGI(STPM_TAG, "Voltage factor (before *100) = %.6f", voltage_factor_float);
    ESP_LOGI(STPM_TAG, "Voltage factor (after *100) = %.6f", voltage_factor_scaled);
    ESP_LOGI(STPM_TAG, "Voltage factor (uint32_t) = %lu", voltage_factor_final);
    ESP_LOGI(STPM_TAG, "MACRO result = %lu", CALC_VOLTAGE_MULTI_FACTOR());
    ESP_LOGI(STPM_TAG, "Expected hardcoded = %lu", (uint32_t)116274);
    
    ESP_LOGI(STPM_TAG, "");
    ESP_LOGI(STPM_TAG, "=== CURRENT FACTOR CALCULATION ===");
    
    // Step-by-step current calculation
    double sensitivity_volts = CT_SENSITIVITY / 1000.0;
    double ai_cal_i = AI * CAL_I;
    double denominator = ai_cal_i * sensitivity_volts * KINT;
    double current_factor_float = VREF / denominator;
    double current_factor_scaled = current_factor_float * 100.0;
    uint32_t current_factor_final = (uint32_t)current_factor_scaled;
    
    ESP_LOGI(STPM_TAG, "CT_SENSITIVITY/1000 = %.6f", sensitivity_volts);
    ESP_LOGI(STPM_TAG, "AI * CAL_I = %.6f", ai_cal_i);
    ESP_LOGI(STPM_TAG, "AI * CAL_I * sensitivity * KINT = %.6f", denominator);
    ESP_LOGI(STPM_TAG, "VREF / denominator = %.6f", current_factor_float);
    ESP_LOGI(STPM_TAG, "Current factor (after *100) = %.6f", current_factor_scaled);
    ESP_LOGI(STPM_TAG, "Current factor (uint32_t) = %lu", current_factor_final);
    ESP_LOGI(STPM_TAG, "MACRO result = %lu", CALC_CURRENT_MULTI_FACTOR(CT_SENSITIVITY));
    ESP_LOGI(STPM_TAG, "Expected hardcoded = %lu", (uint32_t)4214);
    
    ESP_LOGI(STPM_TAG, "");
    ESP_LOGI(STPM_TAG, "=== POWER FACTOR CALCULATION ===");
    
    // Step-by-step power calculation
    double vref_squared = VREF * VREF;
    double numerator = vref_squared * r_ratio;
    double power_denominator = KINT * AV * AI * (CT_SENSITIVITY/1000.0) * CAL_V * CAL_I;
    double power_factor_float = numerator / power_denominator;
    double power_factor_scaled = power_factor_float * 100000.0;
    uint32_t power_factor_final = (uint32_t)power_factor_scaled;
    
    ESP_LOGI(STPM_TAG, "VREF^2 = %.6f", vref_squared);
    ESP_LOGI(STPM_TAG, "VREF^2 * (1 + R1/R2) = %.6f", numerator);
    ESP_LOGI(STPM_TAG, "KINT * AV * AI * sensitivity * CAL_V * CAL_I = %.9f", power_denominator);
    ESP_LOGI(STPM_TAG, "Power factor (before *100000) = %.9f", power_factor_float);
    ESP_LOGI(STPM_TAG, "Power factor (after *100000) = %.6f", power_factor_scaled);
    ESP_LOGI(STPM_TAG, "Power factor (uint32_t) = %lu", power_factor_final);
    ESP_LOGI(STPM_TAG, "MACRO result = %lu", CALC_POWER_MULTI_FACTOR(CT_SENSITIVITY));
    ESP_LOGI(STPM_TAG, "Expected hardcoded = %lu", (uint32_t)4900123);
    
    ESP_LOGI(STPM_TAG, "");
    ESP_LOGI(STPM_TAG, "=== SUMMARY COMPARISON ===");
    ESP_LOGI(STPM_TAG, "Voltage: Calculated=%lu, Expected=%lu, Match=%s", 
             CALC_VOLTAGE_MULTI_FACTOR(), (uint32_t)116274, 
             (CALC_VOLTAGE_MULTI_FACTOR() == 116274) ? "YES" : "NO");
    ESP_LOGI(STPM_TAG, "Current: Calculated=%lu, Expected=%lu, Match=%s", 
             CALC_CURRENT_MULTI_FACTOR(CT_SENSITIVITY), (uint32_t)4214,
             (CALC_CURRENT_MULTI_FACTOR(CT_SENSITIVITY) == 4214) ? "YES" : "NO");
    ESP_LOGI(STPM_TAG, "Power: Calculated=%lu, Expected=%lu, Match=%s", 
             CALC_POWER_MULTI_FACTOR(CT_SENSITIVITY), (uint32_t)4900123,
             (CALC_POWER_MULTI_FACTOR(CT_SENSITIVITY) == 4900123) ? "YES" : "NO");
}


 // Main application entry point
 void app_main() 
 {
     // Initialize UART
     
     stpm_uart_init();
     rpi_uart_init();
     #if DEBUG_CONFIG_MACROS == 1
        debug_calculation_macros();
        ESP_LOGI(STPM_TAG, "=== DEBUG COMPLETE ===");
     #endif
     /* Flush the input and output buffers before performning a read */
     // Wait for the default reg (0x00) value to be available on the UART
     esp_rom_delay_us(2000000);





    ESP_LOGI(SENSOR_TAG, "Initializing I2C...");
    esp_err_t ret = i2c_util_init();
    if (ret != ESP_OK) {
        ESP_LOGE(SENSOR_TAG, "I2C init failed: %s", esp_err_to_name(ret));
        return;
    }

    // TMP117 Init
    ret = i2c_util_add_device(TMP117_I2C_ADDR, &tmp117_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(SENSOR_TAG, "TMP117 add failed");
        return;
    }
    ret = tmp117_init(&tmp117_device, tmp117_dev_handle, TMP117_I2C_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(SENSOR_TAG, "TMP117 init failed");
        return;
    }

    // KX134 Init
    ret = i2c_util_add_device(KX134_I2C_ADDR, &kx134_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(SENSOR_TAG, "KX134 add failed");
        return;
    }
    ret = kx134_init(kx134_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(SENSOR_TAG, "KX134 init failed");
        return;
    }

    // Start sensor task
    BaseType_t task_ret = xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(SENSOR_TAG, "Failed to create sensor task");
        return;
    }





     performDummyRead();
     flush_uart_tx_rx_buff();
     ESP_LOGD(STPM_TAG,"Register before writing:");
     // Perform a sample read to ensure that a register can be read without an issue
     readRegister(0x18, false);
     esp_rom_delay_us(2000000);
     flush_uart_tx_rx_buff();
     uart_driver_delete(STPM_UART);
     ESP_LOGI(STPM_TAG,"Starting read_electrical_params_task");
     
     xTaskCreate(read_electrical_params_task, "params_update_task", 4096, NULL, 5, NULL);
 }