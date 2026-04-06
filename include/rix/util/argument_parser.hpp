#pragma once

#include <cctype>
#include <iostream>
#include <set>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace rix {
namespace util {

/**
 * @brief A command-line parser class.
 */
class ArgumentParser {
   private:
    enum VariantIndex { BOOL, CHAR, INT, DOUBLE, STRING, INT_VEC, DOUBLE_VEC, STRING_VEC };
    using Value = std::variant<bool, char, int, double, std::string, std::vector<int>, std::vector<double>,
                               std::vector<std::string>>;

   public:
    template <typename T>
    inline static constexpr bool is_valid_type =
        std::is_same_v<T, bool> || std::is_same_v<T, char> || std::is_same_v<T, int> || std::is_same_v<T, double> ||
        std::is_same_v<T, std::string> || std::is_same_v<T, std::vector<int>> ||
        std::is_same_v<T, std::vector<double>> || std::is_same_v<T, std::vector<std::string>>;

   private:
    /**
     * @brief A class representing a command line argument.
     */
    struct Arg {
        template <typename T>
        Arg(const std::string &name, const std::string &description, char short_name, const T &default_value,
            bool required);

        std::string name;         ///< long name (--<name>), must be alphanumeric and greater than 2 characters
        std::string description;  ///< description of the option
        Value value;
        bool required;
        char short_name;  ///< short name (single character) (-<short_name>), must be alphanumeric
    };

    void add(const Arg &arg);
    bool parse_arg(char **argv, int argc, int &offset, Arg &arg);

   public:
    /**
     * @brief Constructs a ArgumentParser object.
     * @param name The name of the program.
     * @param description The description of the program.
     */
    ArgumentParser(const std::string &name, const std::string &description);

    /**
     * @brief Adds an argument to the parser.
     * @param argument The argument to add.
     */
    template <typename T>
    void add(const std::string &name, const std::string &description);

    template <typename T>
    void add(const std::string &name, const std::string &description, const T &default_value);

    template <typename T>
    void add(const std::string &name, const std::string &description, char short_name, const T &default_value);

    /**
     * @brief Parses the command-line arguments.
     * @param argc The number of arguments.
     * @param argv The array of arguments.
     */
    bool parse(int argc, char **argv);

    /**
     * @brief Gets the values of an option.
     * @param name The name of the option.
     * @return The value of the option.
     */
    template <typename T>
    bool get(const std::string &name, T &value);

    /**
     * @brief Gets the help message.
     * @return The help message as a string.
     */
    std::string help();

   private:
    ///< The name of the program.
    std::string name_;
    ///< The description of the program.
    std::string description_;
    ///< Ordering of required arguments
    std::vector<std::string> required_arg_names_;
    ///< short name to long name
    std::unordered_map<char, std::string> short_to_long_;
    ///< arguments map
    std::unordered_map<std::string, Arg> args_;
};

namespace detail {

bool isalnum(const std::string &str);

template <typename T>
bool parse(char **argv, int argc, int &offset, T &value) {
    if (offset >= argc) {
        return false;
    }

    size_t index(0);
    try {
        if constexpr (std::is_same_v<T, int>) {
            value = std::stoi(std::string(argv[offset++]), &index);
        } else if constexpr (std::is_same_v<T, char>) {
            auto str = std::string(argv[offset++]);
            if (str.size() != 1) return false;
            value = str[0];
        } else if constexpr (std::is_same_v<T, double>) {
            value = std::stof(std::string(argv[offset++]), &index);
        } else {
            return false;
        }
    } catch (std::invalid_argument const &ex) {
        return false;
    } catch (std::out_of_range const &ex) {
        return false;
    }

    return true;
}
bool parse(char **argv, int argc, int &offset, std::string &value);
bool parse(char **argv, int argc, int &offset, bool &value);

template <typename T>
bool parse(char **argv, int argc, int &offset, std::vector<T> &value) {
    if (offset >= argc) {
        return false;
    }

    value.resize(0);
    size_t i = 0;
    while (offset < argc && argv[offset][0] != '-') {
        value.push_back({});
        if (!parse(argv, argc, offset, value[i++])) {
            value.clear();
            return false;
        }
    }
    value.shrink_to_fit();
    return true;
}

}  // namespace detail

template <typename T>
ArgumentParser::Arg::Arg(const std::string &name, const std::string &description, char short_name,
                         const T &default_value, bool required)
    : name(name), description(description), short_name(short_name), value(default_value), required(required) {
    static_assert(is_valid_type<T>, "Invalid type for option");
    if (name.size() < 2 || !detail::isalnum(name)) {
        throw std::invalid_argument("Name must be at least 2 alphanumeric characters.");
    }
    if (std::is_same_v<T, bool> && required) {
        throw std::invalid_argument("Bool cannot be required.");
    }
}

template <typename T>
bool ArgumentParser::get(const std::string &name, T &value) {
    static_assert(is_valid_type<T>, "Invalid type for option");
    auto arg = args_.find(name);
    if (arg == args_.end()) {
        return false;
    }

    try {
        value = std::get<T>(arg->second.value);
        return true;
    } catch (const std::bad_variant_access &e) {
        return false;
    }
}

template <typename T>
void ArgumentParser::add(const std::string &name, const std::string &description) {
    add(Arg(name, description, '\0', T(), true));
}

template <typename T>
void ArgumentParser::add(const std::string &name, const std::string &description, const T &default_value) {
    add(Arg(name, description, '\0', default_value, false));
}

template <typename T>
void ArgumentParser::add(const std::string &name, const std::string &description, char short_name,
                         const T &default_value) {
    if (!std::isalnum(short_name)) {
        throw std::invalid_argument("Short name must be alphanumeric.");
    }
    add(Arg(name, description, short_name, default_value, false));
}

}  // namespace util
}  // namespace rix