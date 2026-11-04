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
#include <utility>

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
#ifdef DUMP_OUTPUT
    out << "timestamp_us,delayed_timestamp_us,control_status,fault_status,q_w,q_x,q_y,q_z,velocity_n,velocity_e,velocity_d,position_n,position_e,position_d,height_msl_m,latitude_deg,longitude_deg,vertical_derivative,unaided_yaw,track_angle,track_velocity,track_position\n";
#else
    out << "timestamp_us,imu_timestamp_us,control_status,fault_status";
#ifdef DUMP_AID_STATUS
    out << ",baro_timestamp_us,baro_innovation,baro_variance,baro_ratio,baro_fused,baro_rejected,mag_timestamp_us";
    for (const auto *kind : {"innovation", "variance", "ratio"}) {
        for (unsigned i = 0; i < 3; ++i) { out << ",mag_" << kind << '_' << i; }
    }
    out << ",mag_fused,mag_rejected";
#if defined(CONFIG_EKF2_GNSS) && defined(CONFIG_EKF2_RANGE_FINDER)
    for (const auto &[name, dimension] : {std::pair{"gnss_pos", 2u}, std::pair{"gnss_vel", 3u}, std::pair{"gnss_hgt", 1u}, std::pair{"range", 1u}}) {
        out << ',' << name << "_timestamp_us";
        for (const auto *kind : {"innovation", "variance", "ratio"}) {
            for (unsigned i = 0; i < dimension; ++i) { out << ',' << name << '_' << kind << '_' << i; }
        }
        out << ',' << name << "_fused," << name << "_rejected";
    }
    out << ",gnss_quality_failure_bits";
#endif
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
#endif
}

#ifdef DUMP_OUTPUT
void write_output(std::ostream &out, std::ostream &resets, Ekf &ekf, uint64_t time) {
    out << time << ',' << ekf.time_delayed_us() << ',' << ekf.control_status().value << ',' << ekf.fault_status().value;
    const auto q = ekf.getQuaternion();
    for (unsigned i = 0; i < 4; ++i) { out << ',' << q(i); }
    for (const auto &values : {ekf.getVelocity(), ekf.getPosition()}) {
        for (unsigned i = 0; i < 3; ++i) { out << ',' << values(i); }
    }
    const auto global = ekf.getLatLonAlt();
    out << ',' << global.altitude() << std::setprecision(std::numeric_limits<double>::max_digits10)
        << ',' << global.latitude_deg() << ',' << global.longitude_deg()
        << std::setprecision(std::numeric_limits<float>::max_digits10)
        << ',' << ekf.getVerticalPositionDerivative() << ',' << ekf.getUnaidedYaw();
    const auto tracking = ekf.getOutputTrackingError();
    for (unsigned i = 0; i < 3; ++i) { out << ',' << tracking(i); }
    out << '\n';
    const auto &counts = ekf.state_reset_status().reset_count;
    resets << time;
    for (const auto count : {counts.quat, counts.velNE, counts.velD, counts.posNE, counts.posD, counts.hagl}) { resets << ',' << unsigned(count); }
    resets << '\n';
}
#endif

#if defined(DUMP_AID_STATUS) && defined(CONFIG_EKF2_GNSS)
template <unsigned N, typename Source> void write_vector_status(std::ostream &out, const Source &aid) {
    out << ',' << aid.timestamp_sample;
    for (const auto *values : {aid.innovation, aid.innovation_variance, aid.test_ratio}) {
        for (unsigned i = 0; i < N; ++i) { out << ',' << values[i]; }
    }
    out << ',' << aid.fused << ',' << aid.innovation_rejected;
}

void write_scalar_status(std::ostream &out, const estimator_aid_source1d_s &aid) {
    out << ',' << aid.timestamp_sample << ',' << aid.innovation << ',' << aid.innovation_variance
        << ',' << aid.test_ratio << ',' << aid.fused << ',' << aid.innovation_rejected;
}
#endif

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
#if defined(CONFIG_EKF2_GNSS) && defined(CONFIG_EKF2_RANGE_FINDER)
        constexpr std::string_view base_header = "timestamp_us,baro_altitude_m,mag_x_gauss,mag_y_gauss,mag_z_gauss,gps_valid,gps_latitude_deg,gps_longitude_deg,gps_altitude_m,gps_vn_m_s,gps_ve_m_s,gps_vd_m_s,range_distance_m,range_quality";
        extended = header == std::string(base_header) + ",gps_fix_type,gps_hacc_m,gps_vacc_m,gps_sacc_m_s,gps_satellites,gps_pdop,gps_spoofed,range_valid";
        if (header != base_header && !extended) {
#else
        if (header != "timestamp_us,baro_altitude_m,mag_x_gauss,mag_y_gauss,mag_z_gauss") {
#endif
            throw std::runtime_error("unexpected baro/mag CSV header");
        }
        advance();
        if (!ready) { throw std::runtime_error("no aid samples"); }
    }

    void push_until(Ekf &ekf, uint64_t time) {
        while (ready && baro.time_us <= time) {
            ekf.setBaroData(baro);
            ekf.setMagData(mag);
#if defined(CONFIG_EKF2_GNSS) && defined(CONFIG_EKF2_RANGE_FINDER)
            if (gps_ready) { ekf.setGpsData(gps); }
            if (range_ready) { ekf.setRangeData(range); }
#endif
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
#if defined(CONFIG_EKF2_GNSS) && defined(CONFIG_EKF2_RANGE_FINDER)
        std::array<std::string_view, 22> fields{};
        const auto width = extended ? 22u : 14u;
#else
        std::array<std::string_view, 5> fields{};
        const auto width = 5u;
#endif
        for (unsigned i = 0; i < width; ++i) {
            const auto comma = line.find(',');
            if ((i + 1 < width) != (comma != std::string_view::npos)) {
                throw std::runtime_error("unexpected aid CSV field count");
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
#if defined(CONFIG_EKF2_GNSS) && defined(CONFIG_EKF2_RANGE_FINDER)
        const auto valid = number<unsigned>(fields[5]);
        if (valid > 1) { throw std::runtime_error("invalid GPS ready flag"); }
        gps_ready = valid == 1;
        gps.time_us = range.time_us = time;
        gps.lat = number<double>(fields[6]); gps.lon = number<double>(fields[7]);
        gps.alt = number<float>(fields[8]);
        gps.hacc = 0.3f; gps.vacc = 0.5f; gps.sacc = 0.1f; gps.fix_type = 3; gps.nsats = 12; gps.pdop = 1.f;
        gps.spoofed = false;
        range_ready = true;
        if (extended) {
            const auto fix = number<unsigned>(fields[14]); const auto satellites = number<unsigned>(fields[18]);
            const auto spoofed = number<unsigned>(fields[20]); const auto range_valid = number<unsigned>(fields[21]);
            if (fix > 255 || satellites > 255 || spoofed > 1 || range_valid > 1) { throw std::runtime_error("invalid GNSS quality/range ready flag"); }
            gps.fix_type = static_cast<uint8_t>(fix); gps.nsats = static_cast<uint8_t>(satellites); gps.spoofed = spoofed == 1;
            gps.hacc = number<float>(fields[15]); gps.vacc = number<float>(fields[16]); gps.sacc = number<float>(fields[17]); gps.pdop = number<float>(fields[19]);
            for (const auto value : {gps.hacc, gps.vacc, gps.sacc, gps.pdop}) { if (!std::isfinite(value) || value < 0.f) { throw std::runtime_error("invalid GNSS accuracy"); } }
            range_ready = range_valid == 1;
        }
        for (unsigned i = 0; i < 3; ++i) {
            gps.vel(i) = number<float>(fields[9 + i]);
            if (!std::isfinite(gps.vel(i))) { throw std::runtime_error("non-finite GNSS velocity"); }
        }
        const auto quality = number<int>(fields[13]);
        range.rng = number<float>(fields[12]);
        if (!std::isfinite(gps.lat) || !std::isfinite(gps.lon) || !std::isfinite(gps.alt) ||
            std::abs(gps.lat) > 90.0 || std::abs(gps.lon) > 180.0 || !std::isfinite(range.rng) || quality < -1 || quality > 100) {
            throw std::runtime_error("invalid GNSS/range aid field");
        }
        range.quality = static_cast<int8_t>(quality);
#endif
        ready = true;
    }

    std::ifstream input;
    estimator::baroSample baro{};
    estimator::magSample mag{};
#if defined(CONFIG_EKF2_GNSS) && defined(CONFIG_EKF2_RANGE_FINDER)
    estimator::gnssSample gps{};
    estimator::sensor::rangeSample range{};
    bool gps_ready{false};
    bool range_ready{false};
    bool extended{false};
#endif
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
#ifdef DUMP_OUTPUT
        auto reset_path = std::filesystem::path(argv[2]);
        reset_path.replace_extension(".resets.csv");
        for (const auto *path : {argv[1], argv[2], argv[3]}) {
            if (std::filesystem::weakly_canonical(path) == std::filesystem::weakly_canonical(reset_path)
                || (std::filesystem::exists(reset_path) && std::filesystem::exists(path) && std::filesystem::equivalent(path, reset_path))) {
                throw std::runtime_error("reset dump aliases input/output");
            }
        }
        std::ofstream resets(reset_path);
        if (!resets) { throw std::runtime_error("cannot open reset dump"); }
        resets << "timestamp_us,quat_resets,velocity_ne_resets,velocity_d_resets,position_ne_resets,height_resets,hagl_resets\n";
#endif
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
            const bool updated = ekf.update();
#ifdef DUMP_OUTPUT
            updates += updated;
            write_output(output, resets, ekf, imu.time_us);
#else
            if (updated) {
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
#if defined(CONFIG_EKF2_GNSS) && defined(CONFIG_EKF2_RANGE_FINDER)
                write_vector_status<2>(output, ekf.aid_src_gnss_pos());
                write_vector_status<3>(output, ekf.aid_src_gnss_vel());
                write_scalar_status(output, ekf.aid_src_gnss_hgt());
                write_scalar_status(output, ekf.aid_src_rng_hgt());
                output << ',' << ekf.gps_check_fail_status().value;
#endif
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
#endif
        }
#if defined(CONFIG_EKF2_BAROMETER) && defined(CONFIG_EKF2_MAGNETOMETER)
        aids.finish();
#endif
        output.close();
#ifdef DUMP_OUTPUT
        resets.close();
        if (!resets) { throw std::runtime_error("reset dump write failure"); }
#endif
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
