#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MCP4725_DAC_ADDR1           0x62        /*!< Slave address of the MCP4725 DAC (1100 00 [A0])*/
#define MCP4725_DAC_ADDR2           0x63        /*!< Slave address of the MCP4725 DAC (1100 00 [A0])*/

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t mcp4725_register_write_byte(uint8_t reg_addr, uint8_t data, bool second_dac)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, 
        second_dac ? MCP4725_DAC_ADDR2 : MCP4725_DAC_ADDR1, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

int mcp4725_set_voltage(int voltage, bool second_dac)
{
    int ret;
    uint16_t dac_value = 0;
    uint8_t data[2];

    if (voltage > 4095) {
        voltage = 4095;
    } else if (voltage < 0) {
        voltage = 0;
    }

    dac_value = voltage;

    data[0] = (dac_value >> 8) & 0x0F;
    data[1] = dac_value & 0xFF;

    ret = mcp4725_register_write_byte(0x40, data[0], second_dac);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = mcp4725_register_write_byte(0x00, data[1], second_dac);
    if (ret != ESP_OK) {
        return ret;
    }

    return ret;
}

// esp_err_t mcp4725_set_voltage(mcp4725_config_t dac,uint16_t value) {
//   i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // make a command
//   i2c_master_start(cmd); // start command
//   i2c_master_write_byte(cmd,(dac.address << 1) | I2C_MASTER_WRITE,1); // write to address
  
//   i2c_master_write_byte(cmd,(value >> 8) | MCP4725_WRITE_FAST,1); // write upper 4 bits
//   i2c_master_write_byte(cmd,value & MCP4725_MASK,1); // write lower 8 bits
  
//   i2c_master_stop(cmd);
//   esp_err_t ret = i2c_master_cmd_begin(dac.i2c_bus,cmd,dac.ticks_to_wait); // begin sending command
//   i2c_cmd_link_delete(cmd);
//   return ret;
// }

int mcp4725_init()
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // test the presence of the MCP4725 on the I2C bus 
    // uint8_t data;
    // int ret = mcp4725_register_write_byte(0x00, 0x00, false);
    return 0;
}