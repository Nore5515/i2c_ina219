/**
 * @file i2c_ina219.c
 * @author noah reed (nore5515@gmail.com)
 * @brief slimmed down i2c master library for the ina219 current sensor.
 * @version 0.2
 * @date 2023-05-16
 *
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

static const char *TAG = "i2c-ina219";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 5000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< Slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

// Prototypes
int get_current(uint8_t *data_wr, uint8_t *data_rd);
int get_power(uint8_t *data_wr, uint8_t *data_rd);
int get_shunt(uint8_t *data_wr, uint8_t *data_rd);
int get_bus(uint8_t *data_wr, uint8_t *data_rd);

/**
 * @brief Read from slave. Fills 'data_rd'.
 *
 * @param i2c_num
 * @param data_rd
 * @param size
 * @return esp_err_t
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1)
    {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Writing to slave.
 *
 * @param i2c_num
 * @param data_wr
 * @param size
 * @return esp_err_t
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK)
    {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0)
        {
            printf("\n");
        }
    }
    printf("\n");
}

/**
 * @brief Shifts an array of bytes to the right, 'shift' number of times.
 *
 * @param array
 * @param length
 * @param shift
 */
void shiftArrayRight(uint8_t *array, size_t length, uint8_t shift)
{
    size_t i;
    size_t j;
    uint8_t carry = 0;

    for (j = 0; j < shift; j++)
    {
        for (i = 0; i < length; i++)
        {
            uint8_t temp = array[i] & 0x01;
            array[i] >>= 1;      // Right bit shift by 1
            array[i] |= carry;   // Apply carry from the previous element
            carry = (temp << 7); // Store the carried bit for the next element
        }
        carry = 0;
    }
}

/**
 * @brief Passed in a read bufer, return the decimal value.
 *
 * @param data_rd
 * @return int
 */
int master_read_func(uint8_t *data_rd)
{
    // the response from the master_read_slave func
    int master_read_ret;
    size_t d_size = 2;

    // Read the i2c slave data.
    if (d_size == 0)
    {
        ESP_LOGW(TAG, "i2c slave tx buffer full");
        master_read_ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, DATA_LENGTH);
    }
    else
    {
        master_read_ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, RW_TEST_LENGTH);
    }

    // Status Handling.
    if (master_read_ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGE(TAG, "I2C Timeout");
    }
    else if (master_read_ret == ESP_OK)
    {
        uint8_t *extractedData = (uint8_t *)malloc(DATA_LENGTH);
        extractedData[0] = data_rd[0];
        extractedData[1] = data_rd[1];
        shiftArrayRight(extractedData, 2, 3);
        int little_endian = (extractedData[0] << 8) | extractedData[1];
        free(extractedData);
        return little_endian;
    }
    else
    {
        ESP_LOGW(TAG, " %s: Master read slave error, IO not connected...\n", esp_err_to_name(master_read_ret));
    }

    return -1;
}

void handle_master_write_slave(uint8_t *data_wr, int len)
{
    int ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, len);
    if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGE(TAG, "I2C Timeout");
    }
    else if (ret == ESP_OK)
    {
        // Commented for verbosity.
        // ESP_LOGI(TAG, "Write successful!");
    }
    else
    {
        ESP_LOGW(TAG, "%s: Master write slave error, IO not connected....\n", esp_err_to_name(ret));
    }
}

static void i2c_demo_task(void *arg)
{
    uint32_t task_idx = (uint32_t)arg;
    // int i = 0;
    uint8_t *data_wr = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t *data_rd = (uint8_t *)malloc(DATA_LENGTH);
    int cnt = 0;
    while (1)
    {
        ESP_LOGI(TAG, "test cnt: %d", cnt++);

        // Shunt Voltage
        vTaskDelay(100 / portTICK_RATE_MS);
        printf("Shunt Voltage: %d\n", get_shunt(data_wr, data_rd));

        // Bus Voltage
        vTaskDelay(100 / portTICK_RATE_MS);
        printf("Bus Voltage: %d\n", get_bus(data_wr, data_rd));

        // Power
        vTaskDelay(100 / portTICK_RATE_MS);
        printf("Power: %d\n", get_power(data_wr, data_rd));

        // Current
        vTaskDelay(100 / portTICK_RATE_MS);
        printf("Curent: %d\n", get_current(data_wr, data_rd));

        // Delay for some time.
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

//
//   ╔══════════════════════════════════════════════╗
// ╔══════════════════════════════════════════════════╗
// ║                                                  ║
// ║  Four main get functions                         ║
// ║                                                  ║
// ╚══════════════════════════════════════════════════╝
//   ╚══════════════════════════════════════════════╝
//
int get_shunt(uint8_t *data_wr, uint8_t *data_rd)
{
    data_wr[0] = 0x01;
    handle_master_write_slave(data_wr, 1);
    return master_read_func(data_rd);
}
int get_bus(uint8_t *data_wr, uint8_t *data_rd)
{
    data_wr[0] = 0x02;
    handle_master_write_slave(data_wr, 1);
    return master_read_func(data_rd);
}
int get_power(uint8_t *data_wr, uint8_t *data_rd)
{
    data_wr[0] = 0x03;
    handle_master_write_slave(data_wr, 1);
    return master_read_func(data_rd);
}
int get_current(uint8_t *data_wr, uint8_t *data_rd)
{
    data_wr[0] = 0x04;
    handle_master_write_slave(data_wr, 1);
    return master_read_func(data_rd);
}

// Begin the FreeRTOS task "i2c_demo_task"
void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(i2c_demo_task, "i2c_demo_task_0", 1024 * 2, (void *)0, 10, NULL);
}
