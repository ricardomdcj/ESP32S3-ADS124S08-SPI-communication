#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "math.h"
#include "main.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "esp_log.h"
#include "esp_check.h"

#define HIGH  1
#define LOW   0

static const char SPI[] = "SPI2";
static const char REG[] = "REG";
static const char MAIN[] = "MAIN";
static const char RES[] = "RESISTANCE";

spi_device_handle_t spi;

double  VREF = 2.5;                   // Internal reference of 2.5V
double  RESOLUTION  = 8388608;        // 24 bits
double  GAIN = 1;                     // PGA gain

static Main my_main;

extern "C" void app_main(void)
{
    uint8_t channel;

    ESP_ERROR_CHECK(my_main.setupGPIO());
    ESP_ERROR_CHECK(my_main.setupSPI());
    ESP_ERROR_CHECK(my_main.ADS124S08_Init());

    ESP_LOGI(MAIN, "SPI Reading Started");

    while(true)
    {
      float resValue = 0.0;
      float aux[2] = {0.0, 0.0};

      for (channel = 1; channel < 3; channel++)
      {
        my_main.setChannel(channel);
        vTaskDelay(pdMS_TO_TICKS(50));
        if(gpio_get_level((gpio_num_t) ADS124S08_DRDY_PIN) == LOW)  // monitor Data ready(DRDY pin)
        {
          float data = my_main.ADS124S08_Read_Data();      // read 6 bytes conversion register
          aux[channel] = data;
        }
      }
      resValue = aux[1] - aux[2];
      ESP_LOGI(RES, "%f", resValue);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(MAIN, "SPI Reading Finished");
}

esp_err_t Main::setupGPIO(void)
{
    gpio_config_t gpio;

    // DRDY pin config
    gpio.pin_bit_mask = (1 << ADS124S08_DRDY_PIN);
    gpio.mode = GPIO_MODE_INPUT;
    gpio.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio);

    // START pin config
    gpio.pin_bit_mask = (1 << ADS124S08_START_PIN);
    gpio.mode = GPIO_MODE_OUTPUT;
    gpio.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio);

    // POWER DOWN pin config
    gpio.pin_bit_mask = (1 << ADS124S08_PWDN_PIN);
    gpio.mode = GPIO_MODE_OUTPUT;
    gpio.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio);

    // CS pin config
    gpio.pin_bit_mask = (1 << ADS124S08_CS_PIN);
    gpio.mode = GPIO_MODE_OUTPUT;
    gpio.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio);

    // MISO pin config
    gpio.pin_bit_mask = (1 << ADS124S08_DO_MISO_PIN);
    gpio.mode = GPIO_MODE_INPUT;
    gpio.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio);

    // MOSI pin config
    gpio.pin_bit_mask = (1 << ADS124S08_DI_MOSI_PIN);
    gpio.mode = GPIO_MODE_OUTPUT;
    gpio.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio);

    // CLK pin config
    gpio.pin_bit_mask = (1 << ADS124S08_CLK_SCLK_PIN);
    gpio.mode = GPIO_MODE_OUTPUT;
    gpio.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio);

    gpio_set_level((gpio_num_t) ADS124S08_CS_PIN, HIGH);

    vTaskDelay(pdMS_TO_TICKS(1000));

    return ESP_OK;
}

//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
static void my_pre_setup_cb(spi_transaction_t* t)
{
    gpio_set_level((gpio_num_t) ADS124S08_CS_PIN, 0);
    gpio_set_level((gpio_num_t) ADS124S08_START_PIN, 1);
}

//Called after transaction is sent/received. We use this to set the handshake line low.
static void my_post_trans_cb(spi_transaction_t* t)
{
    gpio_set_level((gpio_num_t) ADS124S08_START_PIN, 0);
    gpio_set_level((gpio_num_t) ADS124S08_CS_PIN, 1);
}

esp_err_t Main::setupSPI(void)
{
    ESP_LOGI(SPI, "Initializing SPI2");
    spi_bus_config_t buscfg = {
        .mosi_io_num = ADS124S08_DI_MOSI_PIN,
        .miso_io_num = ADS124S08_DO_MISO_PIN,
        .sclk_io_num = ADS124S08_CLK_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 24
    };

    spi_device_interface_config_t devcfg = {
        .mode = 1,
        //.duty_cycle_pos = 128,
        .clock_speed_hz = SPI_SPEED,
        .spics_io_num = ADS124S08_CS_PIN,
        .queue_size = 8,
        .pre_cb = my_pre_setup_cb,
        .post_cb = my_post_trans_cb
    };

    // Initialize the SPI
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_LOGI(SPI, "DMA Channel: %d", SPI_DMA_CH_AUTO);

    // Define SPI handle
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);

    ESP_LOGI(SPI, "SPI Setup complete");

    return ESP_OK;
}

//--------------------------------------------------------

float Main::ADS124S08_Read_Data() {
    float value = -1;
    uint32_t iData = 0;

    if(gpio_get_level((gpio_num_t) ADS124S08_DRDY_PIN) == 0) {
        while(gpio_get_level((gpio_num_t) ADS124S08_DRDY_PIN) == 0);

        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.flags = SPI_TRANS_USE_RXDATA;
        t.length = 24; // 24 bits
        t.rxlength = 24; // 24 bits

        spi_device_transmit(spi, &t);

        iData = (t.rx_data[0] << 16) | (t.rx_data[1] << 8) | t.rx_data[2];
        //iData = *t.rx_data;
        vTaskDelay(pdMS_TO_TICKS(5));

        //value = (820 * 2.5 * (float)iData) / (RESOLUTION * GAIN);
        value = iData;
        return value;
    } else {
        return value;
    }
}

uint8_t Main::readRegister(uint8_t address)
{
    uint8_t data;

    uint8_t dataToSend = (REGRD_OPCODE_MASK|(address & 0x1f));

    spi_transaction_t transRegRead;
    memset(&transRegRead, 0, sizeof(transRegRead));
    transRegRead.flags = (SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA) ;
    transRegRead.length = 1;
    transRegRead.rxlength = 1;
    transRegRead.tx_data[0] = dataToSend;

    spi_device_transmit(spi, &transRegRead);
/*
    transRegRead.tx_buffer = CONFIG_SPI_MASTER_DUMMY;
    spi_device_transmit(spi, &transRegRead); */
    data = transRegRead.rx_data[0];

    return data;
}

void Main::ADS124S08_Reg_Write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA)
{
    // now combine the register address and the command into one byte:
    uint8_t dataToSend = READ_WRITE_ADDRESS + REGWR_OPCODE_MASK;

    spi_transaction_t transRegWrite;
    memset(&transRegWrite, 0, sizeof(transRegWrite));
    transRegWrite.flags = SPI_TRANS_USE_TXDATA;
    transRegWrite.length = 2;
    transRegWrite.tx_data[0] = dataToSend;
    //transRegWrite.tx_data[1] = DATA;

    spi_device_transmit(spi, &transRegWrite); //Send register location

    transRegWrite.tx_data[0]= 0x00;
    spi_device_transmit(spi, &transRegWrite); //number of register to wr

    transRegWrite.tx_data[0] = DATA;
    spi_device_transmit(spi, &transRegWrite); //Send value to record into register
}

esp_err_t Main::ADS124S08_Init()
{
  // Log.info("ADS Init");
  my_main.ADS124S08_Reset();
  //vTaskDelay(pdMS_TO_TICKS(20));
  my_main.ADS124S08_Reset_Data_Conv_Command();
  //vTaskDelay(pdMS_TO_TICKS(1000));

  ADS124S08_Reg_Write(ID_ADDR_MASK, ADS_124S08);
  ESP_LOGI(SPI, "Adress %d, value %d", ID_ADDR_MASK, my_main.readRegister(ID_ADDR_MASK));

  //vTaskDelay(pdMS_TO_TICKS(10));
  ADS124S08_Reg_Write(INPMUX_ADDR_MASK, (ADS_P_AIN2 | ADS_N_AIN1));	//AINP  = AN2, AINN  = AN1
  ESP_LOGI(SPI, "Adress %d, value %d", INPMUX_ADDR_MASK, my_main.readRegister(INPMUX_ADDR_MASK));

  //vTaskDelay(pdMS_TO_TICKS(10));
  ADS124S08_Reg_Write(PGA_ADDR_MASK, (ADS_PGA_BYPASS | ADS_GAIN_1)); // PGA bypassed, gain 1
  ESP_LOGI(SPI, "Adress %d, value %d", PGA_ADDR_MASK, my_main.readRegister(PGA_ADDR_MASK));

  //vTaskDelay(pdMS_TO_TICKS(10));
  ADS124S08_Reg_Write(DATARATE_ADDR_MASK, (ADS_FILTERTYPE_LL | ADS_DR_50));  //continuous conversion mode, low-latency filter,  50-sps
  ESP_LOGI(SPI, "Adress %d, value %d", DATARATE_ADDR_MASK, my_main.readRegister(DATARATE_ADDR_MASK));

  //vTaskDelay(pdMS_TO_TICKS(10));
  ADS124S08_Reg_Write(REF_ADDR_MASK, (ADS_REFSEL_P0 | ADS_REFINT_ON_ALWAYS | ADS_REFN_BYP_DISABLE));	//monitor off, positive and negative reference enabled, REFP0 AND REFN0 selected, internal reference always on
  ESP_LOGI(SPI, "Adress %d, value %d", REF_ADDR_MASK, my_main.readRegister(REF_ADDR_MASK));

  //vTaskDelay(pdMS_TO_TICKS(10));
  ADS124S08_Reg_Write(IDACMAG_ADDR_MASK, ADS_IDACMAG_750);  //IDAC magnitude set to 750uA
  ESP_LOGI(SPI, "Adress %d, value %d", IDACMAG_ADDR_MASK, my_main.readRegister(IDACMAG_ADDR_MASK));

  //vTaskDelay(pdMS_TO_TICKS(10));
  ADS124S08_Reg_Write(IDACMUX_ADDR_MASK, (ADS_IDAC2_A3 | ADS_IDAC2_A0));  //IDAC1 = AN3, IDAC2 = AN0
  ESP_LOGI(SPI, "Adress %d, value %d", IDACMUX_ADDR_MASK, my_main.readRegister(IDACMUX_ADDR_MASK));

  //vTaskDelay(pdMS_TO_TICKS(10));
  ADS124S08_Reg_Write(VBIAS_ADDR_MASK, ADS_VBIAS_LVL_DIV2);  //no vbias
  ESP_LOGI(SPI, "Adress %d, value %d", VBIAS_ADDR_MASK, my_main.readRegister(VBIAS_ADDR_MASK));

  //vTaskDelay(pdMS_TO_TICKS(10));
  ADS124S08_Reg_Write(SYS_ADDR_MASK, ADS_CALSAMPLE_4);  //calibration: 4 samples
  ESP_LOGI(SPI, "Adress %d, value %d", SYS_ADDR_MASK, my_main.readRegister(SYS_ADDR_MASK));


  //vTaskDelay(pdMS_TO_TICKS(10));
  my_main.ADS124S08_Enable_Start();
  //vTaskDelay(pdMS_TO_TICKS(10));
  my_main.ADS124S08_Disable_Start();
  //vTaskDelay(pdMS_TO_TICKS(50));

  my_main.ADS124S08_Start_Data_Conv_Command();
  //vTaskDelay(pdMS_TO_TICKS(50));

  ESP_LOGI(REG, "Registers OK!");

  return ESP_OK;
}

void Main::setChannel(uint8_t channel)
{
  channel = (channel << 4) + 0x0C;
  ADS124S08_Reg_Write(INPMUX_ADDR_MASK, channel);
  vTaskDelay(pdMS_TO_TICKS(10));
}

void Main::ADS124S08_Reset()
{
  gpio_set_level((gpio_num_t) ADS124S08_PWDN_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));		// Wait 100 mSec
  gpio_set_level((gpio_num_t) ADS124S08_PWDN_PIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level((gpio_num_t) ADS124S08_PWDN_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));
}

void Main::ADS124S08_Disable_Start()
{
  gpio_set_level((gpio_num_t) ADS124S08_START_PIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(10));
}

void Main::ADS124S08_Enable_Start()
{
  gpio_set_level((gpio_num_t) ADS124S08_START_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));
}

void Main::ADS124S08_Hard_Stop (void)
{
  gpio_set_level((gpio_num_t) ADS124S08_START_PIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(10));
}

void Main::ADS124S08_Reset_Data_Conv_Command (void)
{
  my_main.ADS124S08_SPI_Command_Data(RESET_OPCODE_MASK);    // Send 0x08 to the ADS1x9x
}

void Main::ADS124S08_Start_Data_Conv_Command (void)
{
  my_main.ADS124S08_SPI_Command_Data(START_OPCODE_MASK);		// Send 0x08 to the ADS1x9x
}

void Main::ADS124S08_Wakeup_Data_Conv_Command (void)
{
  my_main.ADS124S08_SPI_Command_Data(WAKE_OPCODE_MASK);			// Send 0x08 to the ADS1x9x
}

void Main::ADS124S08_Soft_Stop (void)
{
  my_main.ADS124S08_SPI_Command_Data(STOP_OPCODE_MASK);     // Send 0x0A to the ADS1x9x
}

void Main::ADS124S08_SPI_Command_Data(unsigned char data_in)
{
  spi_transaction_t transCommandData;
  memset(&transCommandData, 0, sizeof(transCommandData));
  transCommandData.flags = SPI_TRANS_USE_TXDATA;
  transCommandData.length = 24;
  transCommandData.tx_data[0] = data_in;
  spi_device_transmit(spi, &transCommandData);
}
