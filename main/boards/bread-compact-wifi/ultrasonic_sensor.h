#ifndef ULTRASONIC_SENSOR_H_
#define ULTRASONIC_SENSOR_H_

// HC-SR04 ultrasonic sensor (ported from Jervice_XiaoZhi.ino senseDistance()).
// - Measures every 120 ms, median of the last 3 readings (400 cm = nothing in range)
// - Presence: someone closer than PRESENCE_DISTANCE_CM for 3 readings -> OnArrived(distance)
// - Hand: closer than HAND_DISTANCE_CM for 1.5 s -> OnHandNear()
// - MCP tool: self.ultrasonic.get_distance
//
// NOTE: ECHO outputs 5 V. Use a 1k + 2k voltage divider before the ESP32-S3 pin.

#include <atomic>
#include <functional>
#include <cstdio>
#include <string>

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "mcp_server.h"

class UltrasonicSensor {
public:
    std::function<void(int distance_cm)> OnArrived;   // runs in the sensor task
    std::function<void()> OnHandNear;
    std::function<void(bool alone)> OnAloneChanged;   // true after nobody was seen for a while

    UltrasonicSensor(gpio_num_t trig, gpio_num_t echo, int presence_cm = 80)
        : trig_(trig), echo_(echo), presence_cm_(presence_cm) {
        gpio_config_t out = {};
        out.pin_bit_mask = 1ULL << trig_;
        out.mode = GPIO_MODE_OUTPUT;
        ESP_ERROR_CHECK(gpio_config(&out));
        gpio_set_level(trig_, 0);

        gpio_config_t in = {};
        in.pin_bit_mask = 1ULL << echo_;
        in.mode = GPIO_MODE_INPUT;
        in.pull_down_en = GPIO_PULLDOWN_ENABLE;
        in.intr_type = GPIO_INTR_DISABLE;   // polling, no ISR (avoids IRAM linker error)
        ESP_ERROR_CHECK(gpio_config(&in));

        xTaskCreate(&UltrasonicSensor::TaskEntry, "ultrasonic", 3072, this, 2, nullptr);
    }

    int distance_cm() const { return (int)dist_; }
    bool present() const { return present_; }

    void RegisterTools() {
        McpServer::GetInstance().AddTool("self.ultrasonic.get_distance",
            "Measure the distance in centimeters to the nearest object in front of the robot "
            "using the ultrasonic sensor. 400 means nothing is within range. "
            "Use when the user asks how far something is, or whether someone is in front of the robot.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                char buf[96];
                snprintf(buf, sizeof(buf), "{\"distance_cm\": %d, \"person_nearby\": %s}",
                         (int)dist_, present_ ? "true" : "false");
                return std::string(buf);
            });
    }

private:
    static constexpr int kHandDistanceCm = 10;
    static constexpr int kHandHoldMs = 1500;
    static constexpr int kLeaveAfterMs = 2000;     // no one for 2 s -> present = false
    static constexpr int kAloneAfterMs = 25000;    // no one for 25 s -> "alone" (head rests)
    static constexpr int kEchoTimeoutUs = 15000;   // same as pulseIn(..., 15000)

    gpio_num_t trig_, echo_;
    int presence_cm_;
    float hist_[3] = {400, 400, 400};
    std::atomic<float> dist_{400};
    std::atomic<bool> present_{false};

    static void TaskEntry(void* arg) { static_cast<UltrasonicSensor*>(arg)->Run(); }

    float ReadCm() {
        gpio_set_level(trig_, 0);
        esp_rom_delay_us(3);
        gpio_set_level(trig_, 1);
        esp_rom_delay_us(10);
        gpio_set_level(trig_, 0);

        // Poll the ECHO pin (like Arduino pulseIn) - no interrupt needed
        int64_t t0 = esp_timer_get_time();
        while (!gpio_get_level(echo_)) {
            if (esp_timer_get_time() - t0 > 6000) return 400;      // no echo started
        }
        int64_t rise = esp_timer_get_time();
        while (gpio_get_level(echo_)) {
            if (esp_timer_get_time() - rise > kEchoTimeoutUs) return 400;
        }
        int64_t width = esp_timer_get_time() - rise;
        return width * 0.0343f / 2;
    }

    void Run() {
        int near_count = 0;
        bool alone = false;
        int64_t last_seen_ms = 0, hand_start_ms = 0;
        bool hand_done = false;

        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(90));            // + echo time = ~100-120 ms period
            int64_t now = esp_timer_get_time() / 1000;

            hist_[0] = hist_[1]; hist_[1] = hist_[2]; hist_[2] = ReadCm();
            float a = hist_[0], b = hist_[1], c = hist_[2];
            float lo = a < b ? a : b, hi = a < b ? b : a;
            float d = c < lo ? lo : (c > hi ? hi : c); // median of 3
            dist_ = d;

            if (d < presence_cm_) {
                last_seen_ms = now;
                if (++near_count >= 3 && !present_) {
                    present_ = true;
                    if (OnArrived) OnArrived((int)d);
                }
                if (alone) { alone = false; if (OnAloneChanged) OnAloneChanged(false); }
            } else {
                near_count = 0;
                if (present_ && now - last_seen_ms > kLeaveAfterMs) present_ = false;
                if (!alone && now - last_seen_ms > kAloneAfterMs) { alone = true; if (OnAloneChanged) OnAloneChanged(true); }
            }

            if (d < kHandDistanceCm) {
                if (!hand_start_ms) hand_start_ms = now;
                else if (now - hand_start_ms > kHandHoldMs && !hand_done) {
                    hand_done = true;
                    if (OnHandNear) OnHandNear();
                }
            } else {
                hand_start_ms = 0;
                hand_done = false;
            }
        }
    }
};

#endif  // ULTRASONIC_SENSOR_H_
