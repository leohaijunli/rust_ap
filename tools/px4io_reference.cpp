// Host-only protocol vectors, using unmodified PX4 v1.17.0 protocol.h.
// Pass that header's directory with -I; never link this into the firmware.
#include <cstdio>
#include <cstring>

#include "protocol.h"

static void print_packet(const char *name, unsigned code, unsigned page,
                         unsigned offset, const uint16_t *values, unsigned count)
{
    IOPacket packet{0, 0, 0, 0, {0}};
    packet.count_code = count | code;
    packet.page = page;
    packet.offset = offset;
    if (values != nullptr) {
        std::memcpy(packet.regs, values, count * sizeof(uint16_t));
    }
    packet.crc = crc_packet(&packet);
    const auto *bytes = reinterpret_cast<const uint8_t *>(&packet);
    std::printf("%s ", name);
    for (unsigned i = 0; i < PKT_SIZE(packet); ++i) {
        std::printf("%02x", bytes[i]);
    }
    std::printf("\n");
}

int main()
{
    print_packet("config_read", PKT_CODE_READ, PX4IO_PAGE_CONFIG, 0, nullptr, 8);
    print_packet("max_read", PKT_CODE_READ, PX4IO_PAGE_CONFIG, 0, nullptr, PKT_MAX_REGS);
    print_packet("disarm", PKT_CODE_WRITE, PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, nullptr, 1);
    print_packet("safety_on", PKT_CODE_WRITE, PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SAFETY_OFF, nullptr, 1);
    print_packet("button_ack", PKT_CODE_WRITE, PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SAFETY_BUTTON_ACK, nullptr, 1);
    print_packet("disable_pwm", PKT_CODE_WRITE, PX4IO_PAGE_DISARMED_PWM, 0, nullptr, 8);
    const uint16_t config[] = {5, 2, 3, 64, 8, 8, 18, 2};
    print_packet("config_reply", PKT_CODE_SUCCESS, PX4IO_PAGE_CONFIG, 0, config, 8);
    print_packet("write_reply", PKT_CODE_SUCCESS, PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SAFETY_BUTTON_ACK, nullptr, 0);
    return 0;
}
