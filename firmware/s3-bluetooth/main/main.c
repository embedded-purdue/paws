#include "bno085.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_SDA_PIN GPIO_NUM_6 // adjust to your board wiring
#define I2C_MASTER_SCL_PIN GPIO_NUM_7 // adjust to your board wiring
#define GPIO_INT_PIN GPIO_NUM_5       // adjust to your board wiring
#define GPIO_RST_PIN GPIO_NUM_4       // adjust to your board wiring
#define BNO085_I2C_ADDR 0x4A          // 0x4B if ADR/SA0 strapped high
#define RAD_TO_DEG 57.29577951308232f

static void sensor_callback(bno085_handle_t handle,
                            const bno085_sensor_value_t *value,
                            void *user_context) {
  if (value->sensor_id == BNO085_SENSOR_ROTATION_VECTOR) {
    float roll, pitch, yaw;
    bno085_quaternion_to_euler(
        value->data.quaternion.i, value->data.quaternion.j,
        value->data.quaternion.k, value->data.quaternion.real, &roll, &pitch,
        &yaw);
    printf("Roll=%.2f  Pitch=%.2f  Yaw=%.2f  (acc=%.1f deg)\n",
           roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG,
           value->data.quaternion.accuracy_rad * RAD_TO_DEG);
  }
}

void app_main(void) {
  // Set up I2C bus
  i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_MASTER_PORT,
      .sda_io_num = I2C_MASTER_SDA_PIN,
      .scl_io_num = I2C_MASTER_SCL_PIN,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = BNO085_I2C_ADDR,
      .scl_speed_hz = 400000, // BNO085 supports Fast Mode (400kHz)
  };
  i2c_master_dev_handle_t dev_handle;
  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

  // Initialize BNO085 with default config
  bno085_config_t config;
  bno085_config_default(&config);

  bno085_handle_t bno085;
  bno085_init(&config, dev_handle, GPIO_INT_PIN, GPIO_RST_PIN, &bno085);

  // Register callback and enable sensor
  bno085_register_sensor_callback(bno085, sensor_callback, NULL);
  bno085_enable_sensor(bno085, BNO085_SENSOR_ROTATION_VECTOR, 100000); // 10Hz

  // Service in main loop (10ms recommended)
  while (1) {
    bno085_service(bno085);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
