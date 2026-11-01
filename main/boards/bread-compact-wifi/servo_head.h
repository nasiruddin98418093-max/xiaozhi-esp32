#ifndef SERVO_HEAD_H_
#define SERVO_HEAD_H_

// Robot head servo (ported from Jervice_XiaoZhi.ino servoBehavior()).
// - Moves with the robot's mood/emotion, speaking and listening state
// - Can be pointed by the AI through MCP tools (self.servo.*)
// - Signal pulses are stopped 1.5 s after the last move, so it does not buzz or draw current

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "application.h"
#include "mcp_server.h"

class ServoHead {
public:
    enum Behavior { kStill, kLookAround, kSway, kShake, kBigSwing, kTremble };

    explicit ServoHead(gpio_num_t gpio) : gpio_(gpio) {
        ledc_timer_config_t timer = {};
        timer.speed_mode = LEDC_LOW_SPEED_MODE;
        timer.duty_resolution = LEDC_TIMER_13_BIT;
        timer.timer_num = LEDC_TIMER_1;
        timer.freq_hz = 50;
        timer.clk_cfg = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&timer));

        ledc_channel_config_t ch = {};
        ch.gpio_num = gpio_;
        ch.speed_mode = LEDC_LOW_SPEED_MODE;
        ch.channel = LEDC_CHANNEL_1;
        ch.intr_type = LEDC_INTR_DISABLE;
        ch.timer_sel = LEDC_TIMER_1;
        ch.duty = 0;
        ch.hpoint = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&ch));

        xTaskCreate(&ServoHead::TaskEntry, "servo_head", 3072, this, 2, nullptr);
    }

    // Called from the display when the emotion changes (see EmotionOledDisplay)
    void SetEmotion(const char* emotion) {
        behavior_ = BehaviorFor(emotion);
    }

    // The robot is "alone" -> keep the head still (like the sketch's sleep mood)
    void SetAlone(bool alone) { alone_ = alone; }

    void SetEnabled(bool on) { enabled_ = on; }
    bool enabled() const { return enabled_; }
    int angle() const { return (int)cur_; }

    // Point the head at an angle (40..140) and hold it for hold_ms
    void PointTo(int angle, int hold_ms) {
        if (angle < kMinAngle) angle = kMinAngle;
        if (angle > kMaxAngle) angle = kMaxAngle;
        override_angle_ = angle;
        override_until_ms_ = NowMs() + hold_ms;
    }

    void RegisterTools() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.servo.set_angle",
            "Turn the robot's head (servo) to an angle and hold it for a while. "
            "90 is straight ahead, lower turns one way, higher turns the other way. "
            "Use when the user asks the robot to look left, right or ahead.",
            PropertyList({
                Property("angle", kPropertyTypeInteger, 90, kMinAngle, kMaxAngle),
                Property("hold_seconds", kPropertyTypeInteger, 10, 1, 60)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int angle = properties["angle"].value<int>();
                int hold = properties["hold_seconds"].value<int>();
                if (!enabled_) enabled_ = true;
                PointTo(angle, hold * 1000);
                return true;
            });

        mcp.AddTool("self.servo.set_enabled",
            "Turn the robot's head movement on or off. When off the head stays still.",
            PropertyList({Property("enabled", kPropertyTypeBoolean)}),
            [this](const PropertyList& properties) -> ReturnValue {
                enabled_ = properties["enabled"].value<bool>();
                return true;
            });

        mcp.AddTool("self.servo.get_state",
            "Get the head servo state (enabled and current angle).",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                char buf[64];
                snprintf(buf, sizeof(buf), "{\"enabled\": %s, \"angle\": %d}",
                         enabled_ ? "true" : "false", (int)cur_);
                return std::string(buf);
            });
    }

private:
    static constexpr int kMinAngle = 40;
    static constexpr int kMaxAngle = 140;
    static constexpr int kCenter = 90;
    static constexpr int kPulseMinUs = 500;     // same as sv.attach(PIN, 500, 2400)
    static constexpr int kPulseMaxUs = 2400;
    static constexpr int kDetachAfterMs = 1500;
    static constexpr float kMaxStepDeg = 2.0f;  // per 20 ms tick

    gpio_num_t gpio_;
    std::atomic<int> behavior_{kLookAround};
    std::atomic<bool> enabled_{true};
    std::atomic<bool> alone_{false};
    std::atomic<int> override_angle_{kCenter};
    std::atomic<int64_t> override_until_ms_{0};
    float cur_ = kCenter;
    bool attached_ = false;
    int64_t idle_since_ms_ = 0;
    float look_target_ = 0;
    int64_t next_look_ms_ = 0;

    static int64_t NowMs() { return esp_timer_get_time() / 1000; }

    static int BehaviorFor(const char* e) {
        if (!e) return kLookAround;
        auto is = [&](const char* s) { return strcmp(e, s) == 0; };
        if (is("happy") || is("laughing") || is("funny") || is("loving") || is("kissy") ||
            is("delicious") || is("silly") || is("winking") || is("cool") || is("confident") ||
            is("relaxed")) return kSway;
        if (is("angry")) return kShake;
        if (is("confused") || is("embarrassed")) return kBigSwing;
        if (is("surprised") || is("shocked")) return kTremble;
        if (is("sleepy")) return kStill;
        return kLookAround;   // neutral, thinking, sad, crying ...
    }

    static void TaskEntry(void* arg) { static_cast<ServoHead*>(arg)->Run(); }

    void Write(float angle) {
        float us = kPulseMinUs + (angle / 180.0f) * (kPulseMaxUs - kPulseMinUs);
        uint32_t duty = (uint32_t)(us * 8191.0f / 20000.0f);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }

    void Detach() {
        if (!attached_) return;
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        attached_ = false;
    }

    float ComputeBase(int64_t now) {
        float t = (float)now;
        if (now < override_until_ms_) return (float)override_angle_.load();

        auto state = Application::GetInstance().GetDeviceState();
        if (state == kDeviceStateSpeaking) return kCenter + 6 * sinf(t / 130);
        if (state == kDeviceStateListening) return kCenter;
        if (state != kDeviceStateIdle) return kCenter;      // booting, Wi-Fi setup, updating...
        if (alone_) return kCenter;

        switch (behavior_.load()) {
            case kSway:     return kCenter + 12 * sinf(t / 250);
            case kShake:    return kCenter + 8 * sinf(t / 60);
            case kBigSwing: return kCenter + 25 * sinf(t / 300);
            case kTremble:  return kCenter + 4 * sinf(t / 40);
            case kStill:    return kCenter;
            default: {      // look around: random glance every 1.2 - 4 s (same targets as the eyes)
                static const int8_t kLook[9] = {0, 0, -9, 9, 0, -8, 8, -9, 9};
                if (now > next_look_ms_) {
                    look_target_ = kLook[esp_random() % 9] * 2.5f;
                    next_look_ms_ = now + 1200 + (esp_random() % 2800);
                }
                return kCenter + look_target_;
            }
        }
    }

    void Run() {
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(20));
            int64_t now = NowMs();
            if (!enabled_) { Detach(); continue; }

            float tgt = ComputeBase(now);
            if (tgt < kMinAngle) tgt = kMinAngle;
            if (tgt > kMaxAngle) tgt = kMaxAngle;

            if (fabsf(tgt - cur_) > 0.5f) {
                attached_ = true;
                float step = tgt - cur_;
                if (step > kMaxStepDeg) step = kMaxStepDeg;
                if (step < -kMaxStepDeg) step = -kMaxStepDeg;
                cur_ += step;
                Write(cur_);
                idle_since_ms_ = now;
            } else if (attached_ && now - idle_since_ms_ > kDetachAfterMs) {
                Detach();
            }
        }
    }
};

#endif  // SERVO_HEAD_H_
