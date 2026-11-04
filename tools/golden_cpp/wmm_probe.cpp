#include <charconv>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <lib/world_magnetic_model/geo_mag_declination.h>

float coordinate(std::string_view text) {
    float value{};
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || !std::isfinite(value)) {
        throw std::runtime_error("invalid coordinate");
    }
    return value;
}

int main(int argc, char **argv) {
    if (argc != 3) { std::cerr << "Usage: wmm_probe LATITUDE LONGITUDE\n"; return 2; }
    try {
        const auto lat = coordinate(argv[1]);
        const auto lon = coordinate(argv[2]);
        std::cout << std::setprecision(std::numeric_limits<float>::max_digits10)
                  << get_mag_declination_degrees(lat, lon) << ','
                  << get_mag_inclination_degrees(lat, lon) << ','
                  << get_mag_strength_gauss(lat, lon) << '\n';
    } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
    return 0;
}
