#include <string>
#include <map>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <random>

namespace utils {

    enum Color {
        RED,
        GREEN,
        BLUE,
        YELLOW,
        RESET
    };
    
    /// \brief Adds ANSI escape codes to a string to enable
    /// it to be displayed in color when written to the console
    std::string stylize_string(std::string s, Color color);
    
    /// \brief splits a char array into a std::vector<double>
    /// Copies char* to std::string, replaces delimiter with spaces,
    /// creates an istringstream from the string and splits by whitespace,
    /// and finally, converts each substring to a double and stores it in a
    /// std::vector<double> and returns it
    std::vector<double> split_char_arr(const char *msg, char delimiter=',');
    
    /// \brief generates a random double between min and max
    double gen_random_float(double min, double max);
}
