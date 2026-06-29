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
gn10_can::devices::MotorConfig motor_config_belt;

constexpr uint32_t k_heartbeat_toggle_interval_ms = 500;

uint32_t heartbeat_last_toggle_time_ms = 0;

uint8_t motor_num;

int32_t motor_stop_count      = 6;
float vesc_velo[4]            = {0.0f, 0.0f, 0.0f, 0.0f};
float rpm_conversion_constant = -45000.0f;
float target_rpm              = 0.0f;
int32_t rotate_count          = 0;
bool is_moving                = false;
bool magnet_near              = false;
bool homing                   = false;
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
    fdcan1_driver.init();
    vesc.init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    /*
        while (!esc_hub.get_init(motor_num, motor_config_belt)) {
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
        }

        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

        do_homing();*/
}

void loop()
{ /*
     if (esc_hub.get_angular_velocities(vesc_velo)) {
         is_moving =
             (vesc_velo[0] != 0.0f || vesc_velo[1] != 0.0f || vesc_velo[2] != 0.0f ||
              vesc_velo[3] != 0.0f);
     }

     if (is_moving) {
         HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
     } else {
         HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
     }*/

    HAL_ADC_Start(&hadc1);                       // 変換開始
    HAL_ADC_PollForConversion(&hadc1, 100);      // 完了待ち
    int32_t adc_val = HAL_ADC_GetValue(&hadc1);  // 値取得
    float voltage   = (float)adc_val / 4095.0f * 3.3f;

    target_rpm = vesc_velo[0] * rpm_conversion_constant;

    if (rotate_count < motor_stop_count) {
        vesc.comm_can_set_rpm(45, target_rpm);
        vesc.comm_can_set_rpm(43, target_rpm);
    } else {
        vesc.comm_can_set_rpm(45, 0);
        vesc.comm_can_set_rpm(43, 0);
    }

    if (voltage > 2.0f && !magnet_near) {
        rotate_count++;
        magnet_near = true;
    } else if (magnet_near && voltage < 1.7f) {
        magnet_near = false;
    }

    if (rotate_count == motor_stop_count) {
        homing = true;
    }

    if (homing) {
        do_homing();
    }

    switch (rotate_count) {
        case 0:
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
            break;
        case 1:
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
            break;
        case 2:
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
            break;
        case 3:
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
            break;
        case 4:
            HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
            break;
        case 5:
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
            break;
        case 6:
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);

            break;
        default:

            break;
    }

    // update_heartbeat_led();
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