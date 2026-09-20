#include "ml307_udp.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "Ml307Udp"


Ml307Udp::Ml307Udp(std::shared_ptr<AtUart> at_uart, int udp_id) : at_uart_(at_uart), udp_id_(udp_id) {
    event_group_handle_ = xEventGroupCreate();
    local_port_ = 0;

    urc_callback_it_ = at_uart_->RegisterUrcCallback([this](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
        if (command == "MIPOPEN") {
            if (arguments.size() >= 1 && arguments[0].int_value == udp_id_) {
                // ML307 成功=0；美格/XJ2113 成功=1；800/550+ 为失败
                last_error_ = arguments.size() > 1 ? arguments[1].int_value : 0;
                if (last_error_ <= 1) {
                    connected_ = true;
                    instance_active_ = true;
                    xEventGroupClearBits(event_group_handle_, ML307_UDP_DISCONNECTED | ML307_UDP_ERROR);
                    xEventGroupSetBits(event_group_handle_, ML307_UDP_CONNECTED);
                } else {
                    connected_ = false;
                    xEventGroupSetBits(event_group_handle_, ML307_UDP_ERROR);
                    ESP_LOGE(TAG, "UDP socket %d MIPOPEN error=%d", udp_id_, last_error_);
                }
            }
        } else if (command == "MIPCLOSE" && arguments.size() == 1) {
            if (arguments[0].int_value == udp_id_) {
                connected_ = false;
                instance_active_ = false;
                xEventGroupSetBits(event_group_handle_, ML307_UDP_DISCONNECTED);
            }
        } else if (command == "MIPSEND" && arguments.size() == 2) {
            if (arguments[0].int_value == udp_id_) {
                xEventGroupSetBits(event_group_handle_, ML307_UDP_SEND_COMPLETE);
            }
        } else if (command == "MIPURC" && arguments.size() == 4) {
            if (arguments[1].int_value == udp_id_) {
                if (arguments[0].string_value == "rudp") {
                    if (message_callback_) {
                        message_callback_(at_uart_->DecodeHex(arguments[3].string_value));
                    }
                } else if (arguments[0].string_value == "disconn") {
                    connected_ = false;
                    instance_active_ = false;
                    xEventGroupSetBits(event_group_handle_, ML307_UDP_DISCONNECTED);
                } else {
                    ESP_LOGE(TAG, "Unknown MIPURC command: %s", arguments[0].string_value.c_str());
                }
            }
        } else if (command == "MIPSTATE") {
            if (arguments.size() >= 1 && arguments[0].int_value == udp_id_) {
                if (arguments.size() >= 5 && arguments[4].string_value == "INITIAL") {
                    connected_ = false;
                    instance_active_ = false;
                } else if (arguments.size() >= 5) {
                    connected_ = true;
                    instance_active_ = true;
                }
                xEventGroupSetBits(event_group_handle_, ML307_UDP_INITIALIZED);
            }
        } else if (command == "MDNSGIP" || command == "CDNSGIP" || command == "MDNSCFG") {
            resolved_ip_.clear();
            for (const auto& arg : arguments) {
                if (IsIpv4(arg.string_value)) {
                    resolved_ip_ = arg.string_value;
                    break;
                }
            }
            xEventGroupSetBits(event_group_handle_, ML307_UDP_DNS_DONE);
        } else if (command == "FIFO_OVERFLOW") {
            xEventGroupSetBits(event_group_handle_, ML307_UDP_ERROR);
            Disconnect();
        }
    });
}

Ml307Udp::~Ml307Udp() {
    Disconnect();
    at_uart_->UnregisterUrcCallback(urc_callback_it_);
    if (event_group_handle_) {
        vEventGroupDelete(event_group_handle_);
    }
}

bool Ml307Udp::IsIpv4(const std::string& value) {
    if (value.empty() || value.find('.') == std::string::npos) {
        return false;
    }
    int dots = 0;
    for (char c : value) {
        if (c == '.') {
            dots++;
        } else if (c < '0' || c > '9') {
            return false;
        }
    }
    return dots == 3;
}

bool Ml307Udp::ResolveHost(const std::string& host, std::string& ip) {
    if (IsIpv4(host)) {
        ip = host;
        return true;
    }

    resolved_ip_.clear();
    xEventGroupClearBits(event_group_handle_, ML307_UDP_DNS_DONE);
    if (!at_uart_->SendCommand("AT+MDNSGIP=\"" + host + "\"", UDP_DNS_TIMEOUT_MS)) {
        if (!at_uart_->SendCommand("AT+CDNSGIP=\"" + host + "\"", UDP_DNS_TIMEOUT_MS)) {
            ESP_LOGE(TAG, "DNS command failed for %s", host.c_str());
            return false;
        }
    }

    auto bits = xEventGroupWaitBits(event_group_handle_, ML307_UDP_DNS_DONE, pdTRUE, pdFALSE,
                                    pdMS_TO_TICKS(UDP_DNS_TIMEOUT_MS));
    if (!(bits & ML307_UDP_DNS_DONE) || resolved_ip_.empty()) {
        ESP_LOGE(TAG, "DNS URC timeout or no IPv4 for %s", host.c_str());
        return false;
    }
    ip = resolved_ip_;
    ESP_LOGI(TAG, "DNS %s -> %s", host.c_str(), ip.c_str());
    return true;
}

bool Ml307Udp::TryOpen(const std::string& cmd, int id) {
    udp_id_ = id;
    last_error_ = -1;
    connected_ = false;
    instance_active_ = false;
    xEventGroupClearBits(event_group_handle_, ML307_UDP_CONNECTED | ML307_UDP_ERROR);

    ESP_LOGI(TAG, "try %s", cmd.c_str());
    if (!at_uart_->SendCommand(cmd, UDP_CONNECT_TIMEOUT_MS)) {
        last_error_ = at_uart_->GetCmeErrorCode();
        ESP_LOGW(TAG, "MIPOPEN cmd failed cme=%d: %s", last_error_, cmd.c_str());
        at_uart_->SendCommand("AT+MIPCLOSE=" + std::to_string(id), 1000);
        return false;
    }
    xEventGroupWaitBits(event_group_handle_, ML307_UDP_CONNECTED | ML307_UDP_ERROR,
                        pdTRUE, pdFALSE, pdMS_TO_TICKS(5000));
    // ML307 成功=0；美格/XJ2113 成功=1。若 URC 未解析到，仍用 MIPSTATE 兜底。
    if (last_error_ > 1) {
        ESP_LOGW(TAG, "MIPOPEN URC error=%d: %s", last_error_, cmd.c_str());
        at_uart_->SendCommand("AT+MIPCLOSE=" + std::to_string(id), 1000);
        return false;
    }

    xEventGroupClearBits(event_group_handle_, ML307_UDP_INITIALIZED);
    at_uart_->SendCommand("AT+MIPSTATE=" + std::to_string(id));
    xEventGroupWaitBits(event_group_handle_, ML307_UDP_INITIALIZED, pdTRUE, pdFALSE, pdMS_TO_TICKS(3000));
    if (!connected_) {
        ESP_LOGW(TAG, "MIPSTATE not connected after: %s", cmd.c_str());
        at_uart_->SendCommand("AT+MIPCLOSE=" + std::to_string(id), 1000);
        return false;
    }
    instance_active_ = true;
    return true;
}

bool Ml307Udp::Connect(const std::string& host, int port, int local_port) {
    local_port_ = local_port;
    return Connect(host, port);
}

bool Ml307Udp::Connect(const std::string& host, int port) {
    xEventGroupClearBits(event_group_handle_, ML307_UDP_CONNECTED | ML307_UDP_DISCONNECTED | ML307_UDP_ERROR);

    std::string ip;
    if (!ResolveHost(host, ip)) {
        ip = host;
    }

    // MQTT=0，语音 UDP 用 1/2；先关掉残留 socket，避免 error=800
    for (int id : {1, 2, 3}) {
        at_uart_->SendCommand("AT+MIPCLOSE=" + std::to_string(id), 500);
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // 优先请求的 id，再试 1/2（与旧 XJ2113 可用固件一致）
    const int ids[] = {udp_id_ > 0 ? udp_id_ : 1, 1, 2};
    for (int id : ids) {
        if (id <= 0) {
            continue;
        }
        udp_id_ = id;
        connected_ = false;
        instance_active_ = false;

        xEventGroupClearBits(event_group_handle_, ML307_UDP_INITIALIZED);
        at_uart_->SendCommand("AT+MIPSTATE=" + std::to_string(id));
        xEventGroupWaitBits(event_group_handle_, ML307_UDP_INITIALIZED, pdTRUE, pdFALSE,
                            pdMS_TO_TICKS(3000));
        if (instance_active_ || connected_) {
            at_uart_->SendCommand("AT+MIPCLOSE=" + std::to_string(id), 2000);
            xEventGroupWaitBits(event_group_handle_, ML307_UDP_DISCONNECTED, pdTRUE, pdFALSE,
                                pdMS_TO_TICKS(2000));
            connected_ = false;
            instance_active_ = false;
        }

        // XJ/美格常不支持 MIPCFG；失败时仍继续 MIPOPEN（旧可用固件即如此）
        // encoding=1,1：hex 拼进 AT+MIPSEND；0,1：命令后跟原始 payload
        const char* encodings[] = {"1,1", "0,1"};
        for (const char* enc : encodings) {
            at_uart_->SendCommand("AT+MIPCFG=\"ssl\"," + std::to_string(id) + ",0,0", 1000);
            if (!at_uart_->SendCommand("AT+MIPCFG=\"encoding\"," + std::to_string(id) + "," + enc,
                                       1000)) {
                ESP_LOGW(TAG, "MIPCFG encoding=%s failed on id=%d, still try MIPOPEN", enc, id);
            }
            send_hex_in_command_ = (enc[0] == '1');

            std::string p = std::to_string(port);
            std::string i = std::to_string(id);
            std::string lp = (local_port_ == 0) ? "" : std::to_string(local_port_);

            std::string cmds[10];
            int n = 0;
            // 旧 XJ2113 可用固件优先的格式
            cmds[n++] = "AT+MIPOPEN=" + i + ",\"UDP\",\"" + ip + "\"," + p;
            cmds[n++] = "AT+MIPOPEN=" + i + ",\"UDP\",\"" + ip + "\"," + p + ",60";
            cmds[n++] = "AT+MIPOPEN=" + i + ",\"UDP\",\"" + ip + "\"," + p + ",60,0";
            cmds[n++] = "AT+MIPOPEN=" + i + ",\"UDP\",\"" + ip + "\"," + p + ",60,0,0";
            cmds[n++] = "AT+MIPOPEN=" + i + ",0,\"" + ip + "\"," + p + ",1";
            cmds[n++] = "AT+MIPOPEN=" + i + ",0,\"" + ip + "\"," + p + ",1,0";
            // 官方 ML307 空 local port
            if (local_port_ == 0) {
                cmds[n++] = "AT+MIPOPEN=" + i + ",\"UDP\",\"" + ip + "\"," + p + ",,0";
            } else {
                cmds[n++] = "AT+MIPOPEN=" + i + ",\"UDP\",\"" + ip + "\"," + p + "," + lp + ",0";
            }
            cmds[n++] = "AT+MIPOPEN=" + i + ",1,\"" + ip + "\"," + p;
            cmds[n++] = "AT+MIPOPEN=" + i + ",1,\"" + ip + "\"," + p + ",0";

            for (int c = 0; c < n; c++) {
                if (TryOpen(cmds[c], id)) {
                    ESP_LOGI(TAG, "UDP connected %s:%d id=%d enc=%s", ip.c_str(), port, id, enc);
                    return true;
                }
            }
        }
    }

    ESP_LOGE(TAG, "Failed to connect UDP %s:%d", host.c_str(), port);
    return false;
}


void Ml307Udp::Disconnect() {
    if (!connected_ && !instance_active_) {
        return;
    }
    connected_ = false;
    instance_active_ = false;
    at_uart_->SendCommand("AT+MIPCLOSE=" + std::to_string(udp_id_));
}

int Ml307Udp::Send(const std::string& data) {
    const size_t MAX_PACKET_SIZE = 1460 / 2;

    if (!connected_) {
        ESP_LOGE(TAG, "Not connected");
        return -1;
    }

    if (data.size() > MAX_PACKET_SIZE) {
        ESP_LOGE(TAG, "数据块超过最大限制");
        return -1;
    }

    if (send_hex_in_command_) {
        // XJ: encoding=1,1 — hex 直接拼进 AT+MIPSEND
        std::string command = "AT+MIPSEND=" + std::to_string(udp_id_) + "," + std::to_string(data.size()) + ",";
        at_uart_->EncodeHexAppend(command, data.c_str(), data.size());
        if (!at_uart_->SendCommand(command, 100)) {
            ESP_LOGE(TAG, "Failed to send data chunk");
            return -1;
        }
    } else {
        // 官方 encoding=0,1 — 命令后跟原始 payload（SendCommandWithData 会补 \r\n）
        std::string command = "AT+MIPSEND=" + std::to_string(udp_id_) + "," + std::to_string(data.size());
        if (!at_uart_->SendCommandWithData(command, 1000, true, data.data(), data.size())) {
            ESP_LOGE(TAG, "Failed to send data chunk");
            return -1;
        }
    }
    return data.size();
}

int Ml307Udp::GetLastError() {
    return last_error_;
}
