#include "imu.h"

#include "bno085.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// ---------------------------------------------------------------------------
// Pin / address config
// ---------------------------------------------------------------------------
#define IMU_I2C_PORT I2C_NUM_0
#define IMU_SDA_PIN GPIO_NUM_6
#define IMU_SCL_PIN GPIO_NUM_7
#define IMU_INT_PIN GPIO_NUM_5
#define IMU_RST_PIN GPIO_NUM_4
#define IMU_I2C_ADDR 0x4A
#define IMU_SAMPLE_US 100000 // 10 Hz

#define RAD_TO_DEG 57.29577951308232f

// ---------------------------------------------------------------------------
// Shared state — written by sensor callback, read via imu_get()
// ---------------------------------------------------------------------------
static imu_state_t g_imu = {0};
static SemaphoreHandle_t g_imu_mutex;

// ---------------------------------------------------------------------------
// BNO085 sensor callback (called from bno085_service on CPU1)
// ---------------------------------------------------------------------------
static void sensor_callback(bno085_handle_t handle,
                            const bno085_sensor_value_t *value,
                            void *user_context) {
  (void)handle;
  (void)user_context;

  if (value->sensor_id != BNO085_SENSOR_ROTATION_VECTOR)
    return;

  float roll, pitch, yaw;
  bno085_quaternion_to_euler(value->data.quaternion.i, value->data.quaternion.j,
                             value->data.quaternion.k,
                             value->data.quaternion.real, &roll, &pitch, &yaw);

  xSemaphoreTake(g_imu_mutex, portMAX_DELAY);
  g_imu.roll = roll;
  g_imu.pitch = pitch;
  g_imu.yaw = yaw;
  g_imu.valid = true;
  xSemaphoreGive(g_imu_mutex);
}

// ---------------------------------------------------------------------------
// BNO085 task (pinned to core 1 by imu_start)
// ---------------------------------------------------------------------------
static void bno085_task(void *arg) {
  i2c_master_bus_config_t bus_cfg = {
      .i2c_port = IMU_I2C_PORT,
      .sda_io_num = IMU_SDA_PIN,
      .scl_io_num = IMU_SCL_PIN,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  i2c_master_bus_handle_t bus;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = IMU_I2C_ADDR,
      .scl_speed_hz = 400000,
  };
  i2c_master_dev_handle_t dev;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev));

  bno085_config_t cfg;
  bno085_config_default(&cfg);

  bno085_handle_t bno;
  bno085_init(&cfg, dev, IMU_INT_PIN, IMU_RST_PIN, &bno);

  bno085_register_sensor_callback(bno, sensor_callback, NULL);
  bno085_enable_sensor(bno, BNO085_SENSOR_ROTATION_VECTOR, IMU_SAMPLE_US);

  while (1) {
    bno085_service(bno);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void imu_start(void) {
  g_imu_mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(bno085_task, "bno085", 4096, NULL,
                          /*priority*/ 5, NULL, /*core*/ 1);
}

bool imu_get(imu_state_t *out) {
  xSemaphoreTake(g_imu_mutex, portMAX_DELAY);
  bool valid = g_imu.valid;
  if (valid)
    *out = g_imu;
  xSemaphoreGive(g_imu_mutex);
  return valid;
}
