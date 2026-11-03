/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>

using namespace time_literals;

namespace ina219
{

static constexpr uint32_t BUS_CLOCK_HZ = 100'000;

// INA219 register map
enum class Register : uint8_t {
        CONFIGURATION = 0x00,
        SHUNT_VOLTAGE = 0x01,
        BUS_VOLTAGE = 0x02,
        POWER = 0x03,
        CURRENT = 0x04,
        CALIBRATION = 0x05,
};

// Configuration register bits
enum CONFIG_BIT : uint16_t {

        // Reset
        RST = (1u << 15),

        // Bus voltage range
        BRNG_16V = (0u << 13),
        BRNG_32V = (1u << 13),

        // PGA gain
        PGA_1_40MV = (0u << 11),
        PGA_2_80MV = (1u << 11),
        PGA_4_160MV = (2u << 11),
        PGA_8_320MV = (3u << 11),

        // Bus ADC resolution / averaging
        BADC_9BIT = (0u << 7),
        BADC_10BIT = (1u << 7),
        BADC_11BIT = (2u << 7),
        BADC_12BIT = (3u << 7),

        // Bus ADC averaging modes
        BADC_2SAMPLES = (9u << 7),
        BADC_4SAMPLES = (10u << 7),
        BADC_8SAMPLES = (11u << 7),
        BADC_16SAMPLES = (12u << 7),
        BADC_32SAMPLES = (13u << 7),
        BADC_64SAMPLES = (14u << 7),
        BADC_128SAMPLES = (15u << 7),

        // Shunt ADC resolution / averaging
        SADC_9BIT = (0u << 3),
        SADC_10BIT = (1u << 3),
        SADC_11BIT = (2u << 3),
        SADC_12BIT = (3u << 3),

        // Shunt ADC averaging modes
        SADC_2SAMPLES = (9u << 3),
        SADC_4SAMPLES = (10u << 3),
        SADC_8SAMPLES = (11u << 3),
        SADC_16SAMPLES = (12u << 3),
        SADC_32SAMPLES = (13u << 3),
        SADC_64SAMPLES = (14u << 3),
        SADC_128SAMPLES = (15u << 3),

        // Operating mode
        MODE_POWER_DOWN = 0u,
        MODE_SHUNT_TRIGGERED = 1u,
        MODE_BUS_TRIGGERED = 2u,
        MODE_SHUNT_BUS_TRIGGERED = 3u,
        MODE_ADC_OFF = 4u,
        MODE_SHUNT_CONTINUOUS = 5u,
        MODE_BUS_CONTINUOUS = 6u,
        MODE_SHUNT_BUS_CONTINUOUS = 7u,
};

static constexpr hrt_abstime SAMPLE_INTERVAL_US = 100_ms;

} // namespace ina219


class INA219 : public device::I2C, public I2CSPIDriver<INA219>
{
public:
        INA219(const I2CSPIDriverConfig &config);
        ~INA219() override;

        static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
        static void print_usage();

        int init() override;
        void RunImpl();

        void print_status() override;

protected:
        int probe() override;

private:
        int collect();

        int registerRead(ina219::Register reg, uint16_t &value);
        int registerWrite(ina219::Register reg, uint16_t value);

        uint16_t _config_value{0};

        float _bus_voltage{0.f};
        float _shunt_voltage{0.f};
        float _current{0.f};
        float _power{0.f};

        perf_counter_t _sample_perf;
        perf_counter_t _comms_errors;
        perf_counter_t _collection_errors;
};