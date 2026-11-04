#include <array>
#include <charconv>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

#include "ekf.h"
#include "reference_params.h"

static_assert(estimator::State::size == 24);
static_assert(sizeof(estimator::StateSample) == 25 * sizeof(float));
static_assert(std::numeric_limits<float>::is_iec559 && sizeof(float) == 4);

constexpr std::string_view input_header =
    "timestamp_us,delta_angle_dt_us,delta_velocity_dt_us,delta_angle_x_rad,delta_angle_y_rad,"
    "delta_angle_z_rad,delta_velocity_x_m_s,delta_velocity_y_m_s,delta_velocity_z_m_s,"
    "delta_velocity_clipping";

template <typename T> T number(std::string_view text) {
    T value{};
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
        throw std::runtime_error("invalid number");
    }
    return value;
}

estimator::imuSample sample(std::string_view line) {
    std::array<std::string_view, 10> fields{};
    for (size_t index = 0; index < fields.size(); ++index) {
        const size_t comma = line.find(',');
        if ((index + 1 < fields.size()) != (comma != std::string_view::npos)) {
            throw std::runtime_error("expected ten IMU fields");
        }
        fields[index] = line.substr(0, comma);
        if (comma != std::string_view::npos) {
            line.remove_prefix(comma + 1);
        }
    }
    estimator::imuSample imu{};
    imu.time_us = number<uint64_t>(fields[0]);
    const auto angle_dt = number<uint32_t>(fields[1]);
    const auto velocity_dt = number<uint32_t>(fields[2]);
    const auto clipping = number<unsigned>(fields[9]);
    if (angle_dt == 0 || angle_dt > 1'000'000 || velocity_dt == 0 || velocity_dt > 1'000'000 ||
        imu.time_us < angle_dt || imu.time_us < velocity_dt || clipping > 7) {
        throw std::runtime_error("invalid IMU time or clipping mask");
    }
    imu.delta_ang_dt = angle_dt * 1e-6f;
    imu.delta_vel_dt = velocity_dt * 1e-6f;
    for (unsigned axis = 0; axis < 3; ++axis) {
        imu.delta_ang(axis) = number<float>(fields[3 + axis]);
        imu.delta_vel(axis) = number<float>(fields[6 + axis]);
        if (!std::isfinite(imu.delta_ang(axis)) || !std::isfinite(imu.delta_vel(axis))) {
            throw std::runtime_error("non-finite IMU delta");
        }
        imu.delta_vel_clipping[axis] = clipping & (1u << axis);
    }
    return imu;
}

void write_header(std::ostream &out) {
    out << "timestamp_us,imu_timestamp_us,control_status,fault_status";
#ifdef DUMP_AID_STATUS
    out << ",baro_timestamp_us,baro_innovation,baro_variance,baro_ratio,baro_fused,baro_rejected,mag_timestamp_us";
    for (const auto *kind : {"innovation", "variance", "ratio"}) {
        for (unsigned i = 0; i < 3; ++i) { out << ",mag_" << kind << '_' << i; }
    }
    out << ",mag_fused,mag_rejected";
#else
    for (unsigned index = 0; index < 25; ++index) {
        out << ",state_" << index;
    }
    for (unsigned row = 0; row < 24; ++row) {
        for (unsigned column = 0; column < 24; ++column) {
            out << ",cov_" << row << '_' << column;
        }
    }
#endif
    out << '\n';
}

void hash_word(uint64_t &hash, uint64_t value, unsigned bytes) {
    for (unsigned index = 0; index < bytes; ++index) {
        hash = (hash ^ (value & 0xffu)) * 0x100000001b3ull;
        value >>= 8;
    }
}

void hash_float(uint64_t &hash, float value) {
    uint32_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    hash_word(hash, bits, 4);
}

void hash_sample(uint64_t &hash, const estimator::imuSample &imu) {
    hash_word(hash, imu.time_us, 8);
    hash_float(hash, imu.delta_ang_dt);
    hash_float(hash, imu.delta_vel_dt);
    for (unsigned axis = 0; axis < 3; ++axis) {
        hash_float(hash, imu.delta_ang(axis));
    }
    unsigned clipping = 0;
    for (unsigned axis = 0; axis < 3; ++axis) {
        hash_float(hash, imu.delta_vel(axis));
        clipping |= unsigned(imu.delta_vel_clipping[axis]) << axis;
    }
    hash_word(hash, clipping, 1);
}

#if defined(CONFIG_EKF2_BAROMETER) && defined(CONFIG_EKF2_MAGNETOMETER)
class AidInput {
public:
    explicit AidInput(const char *path) : input(path) {
        std::string header;
        if (!std::getline(input, header)) {
            throw std::runtime_error("empty aid input");
        }
        if (!header.empty() && header.back() == '\r') { header.pop_back(); }
        if (header != "timestamp_us,baro_altitude_m,mag_x_gauss,mag_y_gauss,mag_z_gauss") {
            throw std::runtime_error("unexpected baro/mag CSV header");
        }
        advance();
        if (!ready) { throw std::runtime_error("no aid samples"); }
    }

    void push_until(Ekf &ekf, uint64_t time) {
        while (ready && baro.time_us <= time) {
            ekf.setBaroData(baro);
            ekf.setMagData(mag);
            ++consumed;
            advance();
        }
    }

    void finish() const {
        if (ready) { throw std::runtime_error("aid sample lies after last IMU sample"); }
    }
    unsigned count() const { return consumed; }

private:
    void advance() {
        std::string storage;
        if (!std::getline(input, storage)) {
            if (input.bad()) { throw std::runtime_error("aid input read failed"); }
            ready = false;
            return;
        }
        if (!storage.empty() && storage.back() == '\r') { storage.pop_back(); }
        std::string_view line(storage);
        std::array<std::string_view, 5> fields{};
        for (unsigned i = 0; i < fields.size(); ++i) {
            const auto comma = line.find(',');
            if ((i + 1 < fields.size()) != (comma != std::string_view::npos)) {
                throw std::runtime_error("expected five aid fields");
            }
            fields[i] = line.substr(0, comma);
            if (comma != std::string_view::npos) { line.remove_prefix(comma + 1); }
        }
        const auto time = number<uint64_t>(fields[0]);
        if (time <= previous) { throw std::runtime_error("aid time is not strictly increasing"); }
        previous = time;
        baro.time_us = mag.time_us = time;
        baro.hgt = number<float>(fields[1]);
        if (!std::isfinite(baro.hgt)) { throw std::runtime_error("non-finite baro altitude"); }
        for (unsigned i = 0; i < 3; ++i) {
            mag.mag(i) = number<float>(fields[i + 2]);
            if (!std::isfinite(mag.mag(i))) { throw std::runtime_error("non-finite magnetic field"); }
        }
        ready = true;
    }

    std::ifstream input;
    estimator::baroSample baro{};
    estimator::magSample mag{};
    uint64_t previous{0};
    unsigned consumed{0};
    bool ready{false};
};
#endif

int main(int argc, char **argv) {
#if defined(CONFIG_EKF2_BAROMETER) && defined(CONFIG_EKF2_MAGNETOMETER)
    if (argc != 4) {
        std::cerr << "Usage: golden_cpp IMU.csv OUTPUT.csv BARO_MAG.csv (distinct paths)\n";
        return 2;
    }
#else
    if (argc != 3) {
        std::cerr << "Usage: golden_cpp INPUT.csv OUTPUT.csv (distinct paths)\n";
        return 2;
    }
#endif
    try {
#if defined(CONFIG_EKF2_BAROMETER) && defined(CONFIG_EKF2_MAGNETOMETER)
        if (std::filesystem::weakly_canonical(argv[3]) == std::filesystem::weakly_canonical(argv[2]) ||
            (std::filesystem::exists(argv[2]) && std::filesystem::equivalent(argv[3], argv[2]))) {
            throw std::runtime_error("aid input and output must be distinct files");
        }
        AidInput aids(argv[3]);
#endif
        if (std::filesystem::weakly_canonical(argv[1]) ==
                std::filesystem::weakly_canonical(argv[2]) ||
            (std::filesystem::exists(argv[2]) && std::filesystem::equivalent(argv[1], argv[2]))) {
            throw std::runtime_error("input and output must be distinct files");
        }
        std::ifstream input(argv[1]);
        std::ofstream output(argv[2]);
        if (!input || !output) {
            throw std::runtime_error("cannot open input/output");
        }
        std::string line;
        if (!std::getline(input, line)) {
            throw std::runtime_error("empty input");
        }
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line != input_header) {
            throw std::runtime_error("unexpected IMU CSV header");
        }
        Ekf ekf;
        configure_reference(ekf);
        write_header(output);
        output << std::setprecision(std::numeric_limits<float>::max_digits10);
        uint64_t previous = 0;
        uint64_t digest = 0xcbf29ce484222325ull;
        unsigned samples = 0, updates = 0;
        while (std::getline(input, line)) {
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            const auto imu = sample(line);
            if (imu.time_us <= previous) {
                throw std::runtime_error("IMU sample time is not strictly increasing");
            }
            previous = imu.time_us;
            ++samples;
            hash_sample(digest, imu);
            ekf.setIMUData(imu);
#if defined(CONFIG_EKF2_BAROMETER) && defined(CONFIG_EKF2_MAGNETOMETER)
            aids.push_until(ekf, imu.time_us);
#endif
            if (ekf.update()) {
                ++updates;
                output << ekf.time_delayed_us() << ',' << imu.time_us << ','
                       << ekf.control_status().value << ',' << ekf.fault_status().value;
#ifdef DUMP_AID_STATUS
                const auto &baro = ekf.aid_src_baro_hgt();
                const auto &mag = ekf.aid_src_mag();
                output << ',' << baro.timestamp_sample << ',' << baro.innovation << ',' << baro.innovation_variance
                       << ',' << baro.test_ratio << ',' << baro.fused << ',' << baro.innovation_rejected << ',' << mag.timestamp_sample;
                for (const auto *values : {mag.innovation, mag.innovation_variance, mag.test_ratio}) {
                    for (unsigned i = 0; i < 3; ++i) { output << ',' << values[i]; }
                }
                output << ',' << mag.fused << ',' << mag.innovation_rejected;
#else
                const auto state = ekf.state().Data();
                for (unsigned index = 0; index < 25; ++index) {
                    output << ',' << state(index);
                }
                for (unsigned row = 0; row < 24; ++row) {
                    for (unsigned column = 0; column < 24; ++column) {
                        output << ',' << ekf.stateCovariance(row, column);
                    }
                }
#endif
                output << '\n';
            }
        }
#if defined(CONFIG_EKF2_BAROMETER) && defined(CONFIG_EKF2_MAGNETOMETER)
        aids.finish();
#endif
        output.close();
        if (samples == 0 || input.bad() || !output) {
            throw std::runtime_error("no samples or input/output failure");
        }
        std::cout << "{\"px4_ref\":\"" << REFERENCE_PX4_REF << "\",\"samples\":" << samples
                  << ",\"updates\":" << updates << ",\"input_digest\":\"" << std::hex
                  << std::setw(16) << std::setfill('0') << digest << "\""
#if defined(CONFIG_EKF2_BAROMETER) && defined(CONFIG_EKF2_MAGNETOMETER)
                  << ",\"aid_samples\":" << std::dec << aids.count()
#endif
                  << "}\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "golden_cpp: " << error.what() << '\n';
        return 1;
    }
}
