#ifndef ML307_UDP_H
#define ML307_UDP_H

#include "udp.h"
#include "at_uart.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <string>

#define ML307_UDP_CONNECTED BIT0
#define ML307_UDP_DISCONNECTED BIT1
#define ML307_UDP_ERROR BIT2
#define ML307_UDP_RECEIVE BIT3
#define ML307_UDP_SEND_COMPLETE BIT4
#define ML307_UDP_INITIALIZED BIT5
#define ML307_UDP_DNS_DONE BIT6

#define UDP_CONNECT_TIMEOUT_MS 10000
#define UDP_DNS_TIMEOUT_MS 15000

class Ml307Udp : public Udp {
public:
    Ml307Udp(std::shared_ptr<AtUart> at_uart, int udp_id);
    ~Ml307Udp();

    bool Connect(const std::string& host, int port) override;
    bool Connect(const std::string& host, int port, int local_port);
    void Disconnect() override;
    int Send(const std::string& data) override;
    int GetLastError() override;

private:
    static bool IsIpv4(const std::string& value);
    bool ResolveHost(const std::string& host, std::string& ip);
    bool TryOpen(const std::string& cmd, int id);

    std::shared_ptr<AtUart> at_uart_;
    int udp_id_;
    int local_port_ = 0;
    bool instance_active_ = false;
    EventGroupHandle_t event_group_handle_;
    std::list<UrcCallback>::iterator urc_callback_it_;
    int last_error_ = -1;
    std::string resolved_ip_;
    bool send_hex_in_command_ = true;
};

#endif // ML307_UDP_H
