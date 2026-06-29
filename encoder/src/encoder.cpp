#include "encoder/encoder.hpp"

#include "adc.h"
#include "drivers/stm32_fdcan/driver_stm32_fdcan.hpp"
#include "encoder/fdcan_driver.hpp"
#include "encoder/vesc_can.hpp"
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/core/fdcan_bus.hpp"
#include "gn10_can/devices/esc_hub_server.hpp"
#include "gn10_can/devices/motor_driver_types.hpp"
#include "stdio.h"
#include "tim.h"

gn10_can::drivers::FDCANDriver fdcan1_driver(&hfdcan1);
VescCAN vesc(&hfdcan2);

gn10_can::FDCANBus fdcan1_bus(fdcan1_driver);
gn10_can::devices::ESCHubServer esc_hub(fdcan1_bus, 0);

// These use get init
gn10_can::devices::MotorConfig motor_config_belt;
uint8_t motor_num;

// Hall sensor limit settings
int32_t motor_stop_count = 6;
int32_t rotate_count     = 0;

float vesc_velo[4]            = {0.0f, 0.0f, 0.0f, 0.0f};
float rpm_conversion_constant = -45000.0f;
float target_rpm              = 0.0f;

// Control flag
bool is_moving   = false;
bool magnet_near = false;
bool homing      = false;

// Voltage threshold for hall sensor
float voltage_threshold_high = 2.0f;
float voltage_threshold_low  = 1.7f;

// LED config
constexpr uint32_t k_heartbeat_toggle_interval_ms = 500;
uint32_t heartbeat_last_toggle_time_ms            = 0;

/**
 * @brief Toggle heartbeat LED at a fixed interval.
 */
void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= k_heartbeat_toggle_interval_ms) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
    }
}

/**
 * @brief Set the initial position of the motor.
 */
void do_homing()
{
    while (HAL_GPIO_ReadPin(LIM1_2_GPIO_Port, LIM1_2_Pin) == GPIO_PIN_SET) {
        vesc.comm_can_set_rpm(45, -2500);
        vesc.comm_can_set_rpm(43, -2500);

        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    }

    vesc.comm_can_set_rpm(45, 0);
    vesc.comm_can_set_rpm(43, 0);

    homing       = false;
    rotate_count = 0;
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
}

void setup()
{
    // encoder settings
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    fdcan1_driver.init();
    vesc.init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    // Wait until a command arrives
    while (!esc_hub.get_init(motor_num, motor_config_belt)) {
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

    do_homing();
}

int32_t encoder_counts_s = 16000;

void loop()
{
    // if get command,we can see light LED
    if (esc_hub.get_angular_velocities(vesc_velo)) {
        is_moving =
            (vesc_velo[0] != 0.0f || vesc_velo[1] != 0.0f || vesc_velo[2] != 0.0f ||
             vesc_velo[3] != 0.0f);
    }
    if (is_moving) {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    }

    // Control motor moving
    target_rpm = vesc_velo[0] * rpm_conversion_constant;
    if (rotate_count < motor_stop_count) {
        vesc.comm_can_set_rpm(45, target_rpm);
        vesc.comm_can_set_rpm(43, target_rpm);
    } else {
        vesc.comm_can_set_rpm(45, 0);
        vesc.comm_can_set_rpm(43, 0);
    }

    // Hall sensor settings
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    int32_t adc_val = HAL_ADC_GetValue(&hadc1);
    float voltage   = (float)adc_val / 4095.0f * 3.3f;

    // Control Hall sensor
    if (voltage > voltage_threshold_high && !magnet_near) {
        rotate_count++;
        magnet_near = true;
    } else if (magnet_near && voltage < voltage_threshold_low) {
        magnet_near = false;
    }

    // Init settings
    if (rotate_count == motor_stop_count) {
        homing = true;
    }
    if (homing) {
        do_homing();
    }

    // encoder test
    int32_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    float rotates = (float)count / encoder_counts_s;

    if (count > 50000 || count < -50000) {
        __HAL_TIM_SET_COUNTER(&htim3, 0);
    }
    float rotates_test[4] = {rotates, 0.0f, 0.0f, 0.0f};
    esc_hub.set_angular_velocity_feedbacks(rotates_test);

    update_heartbeat_led();
    HAL_Delay(10);
}

// canのcallback処理
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == hfdcan1.Instance) {
        fdcan1_bus.update();
    }
    if (hfdcan->Instance == hfdcan2.Instance) {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            vesc.receive_data(rx_header.Identifier, rx_data, 8);
        }
    }
}