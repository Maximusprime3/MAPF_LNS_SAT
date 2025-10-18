#ifndef SIMPLE_JSON_H
#define SIMPLE_JSON_H

#include <cctype>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace simple_json {

class JsonValue;
using JsonObject = std::map<std::string, JsonValue>;
using JsonArray = std::vector<JsonValue>;

class JsonValue {
public:
    using Variant = std::variant<std::nullptr_t, bool, double, std::string, JsonArray, JsonObject>;

    JsonValue() : data_(nullptr) {}
    JsonValue(std::nullptr_t) : data_(nullptr) {}
    JsonValue(bool value) : data_(value) {}
    JsonValue(double value) : data_(value) {}
    JsonValue(std::string value) : data_(std::move(value)) {}
    JsonValue(JsonArray value) : data_(std::move(value)) {}
    JsonValue(JsonObject value) : data_(std::move(value)) {}

    bool is_null() const { return std::holds_alternative<std::nullptr_t>(data_); }
    bool is_bool() const { return std::holds_alternative<bool>(data_); }
    bool is_number() const { return std::holds_alternative<double>(data_); }
    bool is_string() const { return std::holds_alternative<std::string>(data_); }
    bool is_array() const { return std::holds_alternative<JsonArray>(data_); }
    bool is_object() const { return std::holds_alternative<JsonObject>(data_); }

    bool as_bool() const { return std::get<bool>(data_); }
    double as_number() const { return std::get<double>(data_); }
    const std::string &as_string() const { return std::get<std::string>(data_); }
    const JsonArray &as_array() const { return std::get<JsonArray>(data_); }
    const JsonObject &as_object() const { return std::get<JsonObject>(data_); }

    JsonArray &as_array() { return std::get<JsonArray>(data_); }
    JsonObject &as_object() { return std::get<JsonObject>(data_); }

private:
    Variant data_;
};

class Parser {
public:
    explicit Parser(const std::string &input) : input_(input), pos_(0) {}

    JsonValue parse() {
        skip_whitespace();
        JsonValue value = parse_value();
        skip_whitespace();
        if (pos_ != input_.size()) {
            throw std::runtime_error("Unexpected trailing characters in JSON input");
        }
        return value;
    }

private:
    const std::string &input_;
    std::size_t pos_;

    void skip_whitespace() {
        while (pos_ < input_.size() && std::isspace(static_cast<unsigned char>(input_[pos_]))) {
            ++pos_;
        }
    }

    char peek() const {
        if (pos_ >= input_.size()) {
            return '\0';
        }
        return input_[pos_];
    }

    char get() {
        if (pos_ >= input_.size()) {
            throw std::runtime_error("Unexpected end of JSON input");
        }
        return input_[pos_++];
    }

    JsonValue parse_value() {
        char c = peek();
        switch (c) {
            case 'n':
                return parse_literal("null", JsonValue(nullptr));
            case 't':
                return parse_literal("true", JsonValue(true));
            case 'f':
                return parse_literal("false", JsonValue(false));
            case '"':
                return JsonValue(parse_string());
            case '[':
                return JsonValue(parse_array());
            case '{':
                return JsonValue(parse_object());
            default:
                if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
                    return JsonValue(parse_number());
                }
                break;
        }
        throw std::runtime_error("Invalid JSON value");
    }

    JsonValue parse_literal(const char *literal, JsonValue value) {
        for (const char *p = literal; *p; ++p) {
            if (get() != *p) {
                throw std::runtime_error("Invalid JSON literal");
            }
        }
        return value;
    }

    std::string parse_string() {
        if (get() != '"') {
            throw std::runtime_error("JSON string must begin with \"");
        }
        std::string result;
        while (true) {
            char c = get();
            if (c == '"') {
                break;
            }
            if (c == '\\') {
                char esc = get();
                switch (esc) {
                    case '"': result.push_back('"'); break;
                    case '\\': result.push_back('\\'); break;
                    case '/': result.push_back('/'); break;
                    case 'b': result.push_back('\b'); break;
                    case 'f': result.push_back('\f'); break;
                    case 'n': result.push_back('\n'); break;
                    case 'r': result.push_back('\r'); break;
                    case 't': result.push_back('\t'); break;
                    case 'u':
                        result.push_back(parse_unicode());
                        break;
                    default:
                        throw std::runtime_error("Invalid escape sequence in JSON string");
                }
            } else {
                result.push_back(c);
            }
        }
        return result;
    }

    char parse_unicode() {
        unsigned int code = 0;
        for (int i = 0; i < 4; ++i) {
            char c = get();
            code <<= 4;
            if (c >= '0' && c <= '9') {
                code |= static_cast<unsigned int>(c - '0');
            } else if (c >= 'a' && c <= 'f') {
                code |= static_cast<unsigned int>(c - 'a' + 10);
            } else if (c >= 'A' && c <= 'F') {
                code |= static_cast<unsigned int>(c - 'A' + 10);
            } else {
                throw std::runtime_error("Invalid unicode escape in JSON string");
            }
        }
        if (code <= 0x7F) {
            return static_cast<char>(code);
        }
        // For simplicity, only support basic multilingual plane characters that map to single bytes.
        // If code point is outside ASCII range, throw; callers can extend this if needed.
        throw std::runtime_error("Only ASCII unicode escapes are supported in this simple JSON parser");
    }

    double parse_number() {
        std::size_t start = pos_;
        if (peek() == '-') {
            ++pos_;
        }
        if (peek() == '0') {
            ++pos_;
        } else if (std::isdigit(static_cast<unsigned char>(peek()))) {
            while (std::isdigit(static_cast<unsigned char>(peek()))) {
                ++pos_;
            }
        } else {
            throw std::runtime_error("Invalid number in JSON input");
        }
        if (peek() == '.') {
            ++pos_;
            if (!std::isdigit(static_cast<unsigned char>(peek()))) {
                throw std::runtime_error("Invalid fractional part in JSON number");
            }
            while (std::isdigit(static_cast<unsigned char>(peek()))) {
                ++pos_;
            }
        }
        if (peek() == 'e' || peek() == 'E') {
            ++pos_;
            if (peek() == '+' || peek() == '-') {
                ++pos_;
            }
            if (!std::isdigit(static_cast<unsigned char>(peek()))) {
                throw std::runtime_error("Invalid exponent in JSON number");
            }
            while (std::isdigit(static_cast<unsigned char>(peek()))) {
                ++pos_;
            }
        }
        double value = std::stod(input_.substr(start, pos_ - start));
        return value;
    }

    JsonArray parse_array() {
        if (get() != '[') {
            throw std::runtime_error("JSON array must begin with [");
        }
        JsonArray array;
        skip_whitespace();
        if (peek() == ']') {
            get();
            return array;
        }
        while (true) {
            skip_whitespace();
            array.push_back(parse_value());
            skip_whitespace();
            char c = get();
            if (c == ']') {
                break;
            }
            if (c != ',') {
                throw std::runtime_error("Expected ',' or ']' in JSON array");
            }
        }
        return array;
    }

    JsonObject parse_object() {
        if (get() != '{') {
            throw std::runtime_error("JSON object must begin with {");
        }
        JsonObject object;
        skip_whitespace();
        if (peek() == '}') {
            get();
            return object;
        }
        while (true) {
            skip_whitespace();
            std::string key = parse_string();
            skip_whitespace();
            if (get() != ':') {
                throw std::runtime_error("Expected ':' in JSON object");
            }
            skip_whitespace();
            object.emplace(std::move(key), parse_value());
            skip_whitespace();
            char c = get();
            if (c == '}') {
                break;
            }
            if (c != ',') {
                throw std::runtime_error("Expected ',' or '}' in JSON object");
            }
        }
        return object;
    }
};

inline JsonValue parse(const std::string &input) {
    Parser parser(input);
    return parser.parse();
}

}  // namespace simple_json

#endif  // SIMPLE_JSON_H