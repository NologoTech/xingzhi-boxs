#include "dual_network_board.h"
#include "wifi_board.h"
#include "codecs/es8311_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "power_save_timer.h"
#include "power_manager.h"
#include "led/single_led.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "XINGZHI_S3_4G"

class XINGZHI_S3_4G : public DualNetworkBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    PowerSaveTimer* power_save_timer_;
    PowerManager* power_manager_;
    uint8_t boot_multi_click_count_ = 0;

    void PowerOnModem() {
        // 与 xingzhi-cube / metal 一致：拉高 GPIO21 给 4G 模组供电
        rtc_gpio_init(NETWORK_MODULE_POWER_IN);
        rtc_gpio_set_direction(NETWORK_MODULE_POWER_IN, RTC_GPIO_MODE_OUTPUT_ONLY);
        rtc_gpio_set_level(NETWORK_MODULE_POWER_IN, 1);
        ESP_LOGI(TAG, "4G modem power enabled (GPIO%d=1)", (int)NETWORK_MODULE_POWER_IN);
    }

    void InitializePowerManager() {
        power_manager_ = new PowerManager(POWER_USB_IN);
        power_manager_->OnChargingStatusChanged([this](bool is_charging) {
            if (is_charging) {
                power_save_timer_->SetEnabled(false);
            } else {
                power_save_timer_->SetEnabled(true);
            }
        });
        power_manager_->OnLowBatteryStatusChanged([](bool is_low) {
            if (is_low) {
                Application::GetInstance().Schedule([]() {
                    Application::GetInstance().PlaySound(Lang::Sounds::OGG_LOW_BATTERY);
                });
            }
        });
        power_manager_->OnShutdownRequest([]() {
            Application::GetInstance().PlaySound(Lang::Sounds::OGG_SHUTDOWN);
        });
    }

    void InitializePowerSaveTimer() {
        // 无显示：不进入浅睡眠，仅在空闲约 300s 后关机
        power_save_timer_ = new PowerSaveTimer(-1, -1, 300);
        power_save_timer_->OnShutdownRequest([this]() {
            ESP_LOGI(TAG, "Shutting down");
            auto& app = Application::GetInstance();
            app.PlaySound(Lang::Sounds::OGG_SHUTDOWN);
            vTaskDelay(pdMS_TO_TICKS(2000));
            // 仅 ML307 模式下关掉 4G 供电，避免休眠漏电
            if (GetNetworkType() == NetworkType::ML307) {
                rtc_gpio_set_level(NETWORK_MODULE_POWER_IN, 0);
                rtc_gpio_hold_en(NETWORK_MODULE_POWER_IN);
            }
            power_manager_->shutdown();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeCodecI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags =
                {
                    .enable_internal_pullup = 1,
                },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
    }

    void InitializeButtons() {
        boot_button_.OnPressDown([this]() {
            boot_multi_click_count_++;
            ESP_LOGI(TAG, "BOOT multi-click count: %d/5", boot_multi_click_count_);
        });
        boot_button_.OnClick([this]() {
            // Single-click window ended without reaching 5 clicks; reset counter
            boot_multi_click_count_ = 0;
            power_save_timer_->WakeUp();
            auto& app = Application::GetInstance();
            if (GetNetworkType() == NetworkType::WIFI) {
                if (app.GetDeviceState() == kDeviceStateStarting) {
                    auto& wifi_board = static_cast<WifiBoard&>(GetCurrentBoard());
                    wifi_board.EnterWifiConfigMode();
                    return;
                }
            }
            app.ToggleChatState();
        });
        boot_button_.OnMultipleClick([this]() {
            ESP_LOGI(TAG, "BOOT 5-click reached, switching network");
            boot_multi_click_count_ = 0;
            power_save_timer_->WakeUp();
            SwitchNetworkType();
        }, 5);
    }

public:
    XINGZHI_S3_4G()
        : DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, GPIO_NUM_NC, 1),
          boot_button_(BOOT_BUTTON_GPIO) {
        if (GetNetworkType() == NetworkType::ML307) {
            PowerOnModem();
        }
        InitializePowerManager();
        InitializePowerSaveTimer();
        InitializeCodecI2c();
        InitializeButtons();
    }

    virtual void StartNetwork() override {
        if (GetNetworkType() == NetworkType::ML307) {
            PowerOnModem();
            // 对齐旧 Ml307Board：给模组上电后留出启动时间再探测
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        DualNetworkBoard::StartNetwork();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        // WiFi: GPIO21 is speaker PA enable (align xingzhi-ai-395).
        // ML307: keep NC so codec init cannot pull the modem power pin low;
        // PowerOnModem() already drives NETWORK_MODULE_POWER_IN high.
        static const gpio_num_t pa_pin =
            (GetNetworkType() == NetworkType::WIFI) ? NETWORK_MODULE_POWER_IN : GPIO_NUM_NC;
        static Es8311AudioCodec audio_codec(codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE,
                                            AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_GPIO_MCLK,
                                            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS,
                                            AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, pa_pin,
                                            AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = power_manager_->IsCharging();
        discharging = power_manager_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }
        level = power_manager_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        DualNetworkBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(XINGZHI_S3_4G);
