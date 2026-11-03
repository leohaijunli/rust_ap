/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "ina219.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

using namespace ina219;

static constexpr uint8_t INA219_DEFAULT_ADDRESS = 0x40;

// DFRobot SEN0291:
// 10 mOhm shunt, +/-8 A measurement range
static constexpr float SHUNT_RESISTANCE_OHM = 0.010f;
static constexpr float MAX_EXPECTED_CURRENT_A = 8.0f;

// INA219 calibration
// Current_LSB = Maximum Expected Current / 2^15
static constexpr float CURRENT_LSB_A = MAX_EXPECTED_CURRENT_A / 32768.0f;

// Calibration = trunc(0.04096 / (Current_LSB * R_SHUNT))
static constexpr uint16_t CALIBRATION_VALUE =
        static_cast<uint16_t>(0.04096f / (CURRENT_LSB_A * SHUNT_RESISTANCE_OHM));

// Power_LSB = 20 * Current_LSB
static constexpr float POWER_LSB_W = 20.0f * CURRENT_LSB_A;

// Register conversion constants
static constexpr float BUS_VOLTAGE_LSB_V = 0.004f;
static constexpr float SHUNT_VOLTAGE_LSB_V = 10.0e-6f;


INA219::INA219(const I2CSPIDriverConfig &config)
        : I2C(config),
          I2CSPIDriver(config),
          _sample_perf(perf_alloc(PC_ELAPSED, "ina219_read")),
          _comms_errors(perf_alloc(PC_COUNT, "ina219_com_err")),
          _collection_errors(perf_alloc(PC_COUNT, "ina219_collection_err"))
{
        /*
         * 32 V bus range
         * +/-320 mV shunt range
         * 12-bit bus ADC
         * 12-bit shunt ADC
         * continuous shunt + bus conversion
         */
        _config_value =
                ina219::BRNG_32V |
                ina219::PGA_8_320MV |
                ina219::BADC_12BIT |
                ina219::SADC_12BIT |
                ina219::MODE_SHUNT_BUS_CONT;

        I2C::_retries = 5;
}


INA219::~INA219()
{
        perf_free(_sample_perf);
        perf_free(_comms_errors);
        perf_free(_collection_errors);
}


int INA219::init()
{
        if (I2C::init() != PX4_OK) {
                return PX4_ERROR;
        }

        /*
         * Reset INA219.
         */
        if (registerWrite(Register::CONFIGURATION, RST) != PX4_OK) {
                PX4_ERR("INA219 reset failed");
                return PX4_ERROR;
        }

        /*
         * Give the device time to complete the reset.
         */
        px4_usleep(1_ms);

        /*
         * Program calibration.
         */
        if (registerWrite(Register::CALIBRATION, CALIBRATION_VALUE) != PX4_OK) {
                PX4_ERR("INA219 calibration write failed");
                return PX4_ERROR;
        }

        /*
         * Program measurement configuration.
         */
        if (registerWrite(Register::CONFIGURATION, _config_value) != PX4_OK) {
                PX4_ERR("INA219 configuration write failed");
                return PX4_ERROR;
        }

        PX4_INFO("INA219 initialized: address 0x%02x, calibration %u",
                 get_device_address(), CALIBRATION_VALUE);

        return PX4_OK;
}


void INA219::RunImpl()
{
        if (collect() != PX4_OK) {
                perf_count(_collection_errors);
        }

        ScheduleDelayed(SAMPLE_INTERVAL_US);
}


int INA219::collect()
{
        perf_begin(_sample_perf);

        uint16_t raw_bus = 0;
        uint16_t raw_shunt = 0;
        uint16_t raw_current = 0;
        uint16_t raw_power = 0;

        if (registerRead(Register::BUS_VOLTAGE, raw_bus) != PX4_OK ||
            registerRead(Register::SHUNT_VOLTAGE, raw_shunt) != PX4_OK ||
            registerRead(Register::CURRENT, raw_current) != PX4_OK ||
            registerRead(Register::POWER, raw_power) != PX4_OK) {

                perf_end(_sample_perf);
                return PX4_ERROR;
        }

        /*
         * INA219 bus voltage:
         * bits [15:3] contain the measurement.
         * LSB = 4 mV.
         */
        const uint16_t bus_raw = raw_bus >> 3;
        _bus_voltage = static_cast<float>(bus_raw) * BUS_VOLTAGE_LSB_V;

        /*
         * Shunt voltage:
         * signed 16-bit value, 10 uV/bit.
         */
        const int16_t shunt_raw = static_cast<int16_t>(raw_shunt);
        _shunt_voltage = static_cast<float>(shunt_raw) * SHUNT_VOLTAGE_LSB_V;

        /*
         * Current register:
         * signed 16-bit value, Current_LSB A/bit.
         */
        const int16_t current_raw = static_cast<int16_t>(raw_current);
        _current = static_cast<float>(current_raw) * CURRENT_LSB_A;

        /*
         * Power register:
         * Power_LSB = 20 * Current_LSB.
         */
        _power = static_cast<float>(raw_power) * POWER_LSB_W;

        PX4_INFO("INA219: V=%.3f V, I=%.3f A, P=%.3f W, Shunt=%.3f mV",
                 (double)_bus_voltage,
                 (double)_current,
                 (double)_power,
                 (double)(_shunt_voltage * 1000.0f));

        perf_end(_sample_perf);

        return PX4_OK;
}


int INA219::probe()
{
        /*
         * INA219 does not have a manufacturer/device-ID register like INA226.
         *
         * Instead, read the configuration register to verify communication.
         */
        uint16_t config = 0;

        if (registerRead(Register::CONFIGURATION, config) != PX4_OK) {
                return PX4_ERROR;
        }

        return PX4_OK;
}


int INA219::registerRead(ina219::Register reg, uint16_t &value)
{
        uint8_t address = static_cast<uint8_t>(reg);
        uint16_t raw = 0;

        const int ret = transfer(&address, 1, reinterpret_cast<uint8_t *>(&raw), sizeof(raw));

        if (ret == PX4_OK) {
                value = __builtin_bswap16(raw);

        } else {
                perf_count(_comms_errors);
        }

        return ret;
}


int INA219::registerWrite(ina219::Register reg, uint16_t value)
{
        const uint8_t buf[3] = {
                static_cast<uint8_t>(reg),
                static_cast<uint8_t>((value >> 8) & 0xff),
                static_cast<uint8_t>(value & 0xff)
        };

        const int ret = transfer(buf, sizeof(buf), nullptr, 0);

        if (ret != PX4_OK) {
                perf_count(_comms_errors);
        }

        return ret;
}


void INA219::print_status()
{
        I2CSPIDriverBase::print_status();

        PX4_INFO("configuration: 0x%04x", _config_value);
        PX4_INFO("calibration: %u (0x%04x)", CALIBRATION_VALUE, CALIBRATION_VALUE);
        PX4_INFO("current LSB: %.9f A", (double)CURRENT_LSB_A);
        PX4_INFO("power LSB: %.6f W", (double)POWER_LSB_W);

        PX4_INFO("V: %.3f V", (double)_bus_voltage);
        PX4_INFO("I: %.3f A", (double)_current);
        PX4_INFO("P: %.3f W", (double)_power);
        PX4_INFO("Shunt: %.3f mV", (double)(_shunt_voltage * 1000.0f));

        perf_print_counter(_sample_perf);
        perf_print_counter(_comms_errors);
        perf_print_counter(_collection_errors);
}


I2CSPIDriverBase *INA219::instantiate(const I2CSPIDriverConfig &config, int /*runtime_instance*/)
{
        INA219 *instance = new INA219(config);

        if (instance == nullptr) {
                PX4_ERR("alloc failed");
                return nullptr;
        }

        if (instance->init() == PX4_OK) {
                instance->ScheduleNow();
                return instance;
        }

        delete instance;
        return nullptr;
}


void INA219::print_usage()
{
        PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
### Description

Driver for the DFRobot Gravity I2C Digital Wattmeter
(SEN0291), based on the Texas Instruments INA219.

The default I2C address is 0x40.

The driver reports bus voltage, shunt voltage, current,
and power through the PX4 shell.
)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("ina219", "driver");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
        PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x40);
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}


extern "C" int ina219_main(int argc, char *argv[])
{
        using ThisDriver = INA219;

        BusCLIArguments cli{true, false};

        cli.i2c_address = INA219_DEFAULT_ADDRESS;
        cli.default_i2c_frequency = BUS_CLOCK_HZ;

        const char *verb = cli.optArg();

        int ch;

        while ((ch = cli.getOpt(argc, argv, "")) != EOF) {
                switch (ch) {
                default:
                        break;
                }
        }

        verb = cli.optArg();

        if (!verb) {
                ThisDriver::print_usage();
                return -1;
        }

        BusInstanceIterator iterator(MODULE_NAME, cli, DRV_POWER_DEVTYPE_INA219);

        if (!strcmp(verb, "start")) {
                return ThisDriver::module_start(cli, iterator);
        }

        if (!strcmp(verb, "stop")) {
                return ThisDriver::module_stop(iterator);
        }

        if (!strcmp(verb, "status")) {
                return ThisDriver::module_status(iterator);
        }

        ThisDriver::print_usage();
        return -1;
}