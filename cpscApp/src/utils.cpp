#include "utils.hpp"

namespace utils {

    std::string stylize_string(std::string s, Color color) {
        std::map<Color, std::string> color_map = {
            {RED, "\x1b[31m"},
            {GREEN, "\x1B[32m"},
            {BLUE, "\x1B[34m"},
            {YELLOW, "\x1B[33m"},
            {RESET,"\x1b[0m"},
        };

        std::stringstream ss;
        ss << color_map[color];
        ss << s;
        ss << color_map[Color::RESET];
        return ss.str();
    }

    std::vector<double> split_char_arr(const char *msg, char delimiter) {
        std::string s_msg(msg);
        std::replace(s_msg.begin(), s_msg.end(), delimiter, ' ');
        std::istringstream ss(s_msg);
        std::vector<double> v_out{std::istream_iterator<double>(ss), {}};
        return v_out;
    }

    double gen_random_float(double min, double max) {
        std::random_device rd;  // obtain a random number from hardware
        std::mt19937 eng(rd()); // seed the generator

        std::uniform_real_distribution<> distribution(min, max); // define the range

        return distribution(eng); // generate the random number
    }
}
