#include "encoder/encoder.hpp"

#include "drivers/stm32_fdcan/driver_stm32_fdcan.hpp"
#include "encoder/fdcan_driver.hpp"
#include "encoder/vesc_can.hpp"
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/core/fdcan_bus.hpp"
#include "gn10_can/devices/esc_hub_server.hpp"
#include "stdio.h"
#include "tim.h"

gn10_can::drivers::FDCANDriver fdcan1_driver(&hfdcan1);
VescCAN vesc(&hfdcan2);

gn10_can::FDCANBus fdcan1_bus(fdcan1_driver);
gn10_can::devices::ESCHubServer esc_hub(fdcan1_bus, 0);

constexpr uint32_t k_heartbeat_toggle_interval_ms = 500;

uint32_t heartbeat_last_toggle_time_ms = 0;

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

bool vesc_move;
int32_t origin_taco    = 0;
int32_t last_position  = 0;
int32_t position       = 0;
int32_t prev_position1 = 0;
int32_t prev_position2 = 0;

bool initialized = false;  // グローバルに追加
bool homing_done = false;  // 追加

void do_homing()
{
    // リミットスイッチまで戻る
    while (HAL_GPIO_ReadPin(LIM1_2_GPIO_Port, LIM1_2_Pin) == GPIO_PIN_SET) {
        vesc.comm_can_set_rpm(45, -2500);
        vesc.comm_can_set_rpm(43, -2500);

        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    }

    vesc.comm_can_set_rpm(45, 0);
    vesc.comm_can_set_rpm(43, 0);

    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

    // 機体にリミットスイッチがついていないので無効化
    /*
    for (int i = 0; i < 16; i++) {
        vesc.comm_can_set_rpm(45, -2300);
        vesc.comm_can_set_rpm(43, -2300);

        HAL_Delay(10);
    }
    vesc.comm_can_set_rpm(45, 0);
    vesc.comm_can_set_rpm(43, 0);
    */

    int32_t prev1 = -1, prev2 = -2;
    while (!(vesc.get_taco() == prev1 && prev1 == prev2)) {
        prev2 = prev1;
        prev1 = vesc.get_taco();
        HAL_Delay(5);
    }

    origin_taco    = vesc.get_taco();
    position       = 0;
    prev_position1 = 0;
    prev_position2 = 0;
    last_position  = 0;
    initialized    = false;
    homing_done    = true;
}

void setup()
{
    fdcan1_driver.init();

    vesc.init();
}

// 定義類
float vesc_velo[4]            = {0.0f, 0.0f, 0.0f, 0.0f};
float rpm_conversion_constant = -45000.0f;
float vesc_angular_velocity_command;
bool vesc_init = false;

float current_rpm    = 0.0f;
const float RPM_STEP = 3000.0f;  // 1ステップの加速量

void loop()
{
    int32_t rpm  = vesc.get_rpm();
    int32_t taco = vesc.get_taco();

    // 初期化処理
    if (esc_hub.get_angular_velocities(vesc_velo)) {
        if (!vesc_init && vesc_velo[1] != 0.0f) {
            vesc_init = true;
            do_homing();
        }
    }

    if (!vesc_init) {
        update_heartbeat_led();
        HAL_Delay(10);
        return;
    }

    // position処理
    if (abs(rpm) > 500) {
        int32_t new_position = taco - origin_taco;
        if (abs(new_position - last_position) > 10) {
            prev_position2 = prev_position1;
            prev_position1 = last_position;
            position       = new_position;
            last_position  = new_position;
        }
    } else {
        prev_position2 = prev_position1;
        prev_position1 = position;
    }

    if (abs(position) > 1000) initialized = true;

    // 3回連続同じ値なら再ホーミング
    if (homing_done && initialized && (abs(position) > 110 || abs(position) < 50)) {
        if ((position == prev_position1) && (position == prev_position2)) {
            do_homing();
        }
    }

    // スタック判定
    bool stuck = (abs(position) > 1000);

    if (esc_hub.get_angular_velocities(vesc_velo) && vesc_velo) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
        bool is_moving =
            (vesc_velo[0] != 0.0f || vesc_velo[1] != 0.0f || vesc_velo[2] != 0.0f ||
             vesc_velo[3] != 0.0f);

        if (is_moving) {
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
        }
    }

    // コマンド計算
    vesc_angular_velocity_command = vesc_velo[0] * rpm_conversion_constant;

    if (!stuck) {
        if (vesc_angular_velocity_command < 0.0f) {
            if (current_rpm > vesc_angular_velocity_command) {
                current_rpm -= RPM_STEP;
                if (current_rpm < vesc_angular_velocity_command) {
                    current_rpm = vesc_angular_velocity_command;
                }
            }
        }

        vesc.comm_can_set_rpm(45, current_rpm);
        vesc.comm_can_set_rpm(43, current_rpm);
        HAL_Delay(10);

    } else {
        current_rpm = 0.0f;
        vesc.comm_can_set_rpm(45, 0.0f);
        vesc.comm_can_set_rpm(43, 0.0f);
    }

    float neko[4] = {(float)rpm, 0.0f, 0.0f, 0.0f};
    esc_hub.set_angular_velocity_feedbacks(neko);

    update_heartbeat_led();
    HAL_Delay(10);
}

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