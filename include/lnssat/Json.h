#ifndef LNS_PUBLIC_JSON_H
#define LNS_PUBLIC_JSON_H

#include <cctype>
#include <charconv>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <locale>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace lnssat_json {

class JsonValue;
using JsonObject = std::map<std::string, JsonValue>;
using JsonArray = std::vector<JsonValue>;

class JsonValue {
public:
    using Variant = std::variant<std::nullptr_t, bool, int64_t, double, std::string, JsonArray, JsonObject>;

    JsonValue() : data_(nullptr) {}
    JsonValue(std::nullptr_t) : data_(nullptr) {}
    JsonValue(bool value) : data_(value) {}
    JsonValue(const char* value) : data_(std::string(value)) {}
    JsonValue(int value) : data_(int64_t(value)) {}
    JsonValue(int64_t value) : data_(value) {}
    JsonValue(double value) : data_(value) {}
    JsonValue(std::string value) : data_(std::move(value)) {}
    JsonValue(JsonArray value) : data_(std::move(value)) {}
    JsonValue(JsonObject value) : data_(std::move(value)) {}

    bool is_null() const { return std::holds_alternative<std::nullptr_t>(data_); }
    bool is_bool() const { return std::holds_alternative<bool>(data_); }
    bool is_integer() const { return std::holds_alternative<int64_t>(data_); }
    bool is_number() const { return is_integer() || std::holds_alternative<double>(data_); }
    bool is_string() const { return std::holds_alternative<std::string>(data_); }
    bool is_array() const { return std::holds_alternative<JsonArray>(data_); }
    bool is_object() const { return std::holds_alternative<JsonObject>(data_); }

    bool as_bool() const { return std::get<bool>(data_); }
    int64_t as_integer() const { return std::get<int64_t>(data_); }
    double as_number() const { return is_integer() ? static_cast<double>(as_integer()) : std::get<double>(data_); }
    const std::string &as_string() const { return std::get<std::string>(data_); }
    const JsonArray &as_array() const { return std::get<JsonArray>(data_); }
    const JsonObject &as_object() const { return std::get<JsonObject>(data_); }

    JsonArray &as_array() { return std::get<JsonArray>(data_); }
    JsonObject &as_object() { return std::get<JsonObject>(data_); }

private:
    Variant data_;
};

inline bool valid_utf8(const std::string& s) {
    for (size_t i = 0; i < s.size();) {
        unsigned char c = s[i++];
        if (c < 128) continue;
        int n; uint32_t cp, min;
        if (c >= 0xc2 && c <= 0xdf) { n = 1; cp = c & 31; min = 0x80; }
        else if (c >= 0xe0 && c <= 0xef) { n = 2; cp = c & 15; min = 0x800; }
        else if (c >= 0xf0 && c <= 0xf4) { n = 3; cp = c & 7; min = 0x10000; }
        else return false;
        while (n--) { if (i == s.size()) return false; unsigned char t = s[i++]; if ((t & 0xc0) != 0x80) return false; cp = (cp << 6) | (t & 63); }
        if (cp < min || cp > 0x10ffff || (cp >= 0xd800 && cp <= 0xdfff)) return false;
    }
    return true;
}

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
    int depth_ = 0;

    void skip_whitespace() {
        while (pos_ < input_.size() && (input_[pos_] == ' ' || input_[pos_] == '\n' || input_[pos_] == '\r' || input_[pos_] == '\t')) {
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
        struct Depth { int& n; ~Depth() { --n; } } guard{depth_};
        if (++depth_ > 128) throw std::runtime_error("JSON nesting exceeds 128");
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
                    return parse_number();
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
                    case 'u': {
                        unsigned int cp = parse_unicode();
                        if (cp >= 0xd800 && cp <= 0xdbff) {
                            if (get() != '\\' || get() != 'u') throw std::runtime_error("Missing low surrogate");
                            unsigned int low = parse_unicode();
                            if (low < 0xdc00 || low > 0xdfff) throw std::runtime_error("Invalid low surrogate");
                            cp = 0x10000 + ((cp - 0xd800) << 10) + (low - 0xdc00);
                        } else if (cp >= 0xdc00 && cp <= 0xdfff) throw std::runtime_error("Unpaired surrogate");
                        if (cp < 0x80) result += static_cast<char>(cp);
                        else if (cp < 0x800) { result += static_cast<char>(0xc0 | (cp >> 6)); result += static_cast<char>(0x80 | (cp & 63)); }
                        else if (cp < 0x10000) { result += static_cast<char>(0xe0 | (cp >> 12)); result += static_cast<char>(0x80 | ((cp >> 6) & 63)); result += static_cast<char>(0x80 | (cp & 63)); }
                        else { result += static_cast<char>(0xf0 | (cp >> 18)); result += static_cast<char>(0x80 | ((cp >> 12) & 63)); result += static_cast<char>(0x80 | ((cp >> 6) & 63)); result += static_cast<char>(0x80 | (cp & 63)); }
                        break;
                    }
                    default:
                        throw std::runtime_error("Invalid escape sequence in JSON string");
                }
            } else {
                if (static_cast<unsigned char>(c) < 32) throw std::runtime_error("Unescaped control byte");
                result.push_back(c);
            }
        }
        if (!valid_utf8(result)) throw std::runtime_error("Invalid UTF-8 JSON string");
        return result;
    }

    unsigned int parse_unicode() {
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
        return code;
    }

    JsonValue parse_number() {
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
        const std::string number = input_.substr(start, pos_ - start);
        if (number.find_first_of(".eE") == std::string::npos) {
            int64_t value;
            auto parsed = std::from_chars(number.data(), number.data() + number.size(), value);
            if (parsed.ec != std::errc()) throw std::runtime_error("JSON integer overflow");
            return JsonValue(value);
        }
        std::istringstream stream(number);
        stream.imbue(std::locale::classic());
        double value;
        if (!(stream >> value) || !std::isfinite(value)) throw std::runtime_error("JSON number overflow");
        return JsonValue(value);
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
            if (!object.emplace(std::move(key), parse_value()).second) throw std::runtime_error("Duplicate JSON key");
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


inline void write_string(std::ostream& out, const std::string& value) {
    if (!valid_utf8(value)) throw std::runtime_error("Invalid UTF-8 string");
    out << '"';
    const char* hex = "0123456789abcdef";
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << static_cast<char>(c);
        else if (c < 32) out << "\\u00" << hex[c >> 4] << hex[c & 15];
        else out << static_cast<char>(c);
    }
    out << '"';
}
inline void write(std::ostream& out, const JsonValue& value) {
    if (value.is_null()) out << "null";
    else if (value.is_bool()) out << (value.as_bool() ? "true" : "false");
    else if (value.is_integer()) out << value.as_integer();
    else if (value.is_number()) {
        if (!std::isfinite(value.as_number())) throw std::runtime_error("Nonfinite JSON number");
        out << std::setprecision(17) << value.as_number();
    } else if (value.is_string()) write_string(out, value.as_string());
    else if (value.is_array()) {
        out << '['; bool first = true;
        for (const auto& item : value.as_array()) { if (!first) out << ','; first = false; write(out, item); }
        out << ']';
    } else {
        out << '{'; bool first = true;
        for (const auto& item : value.as_object()) { if (!first) out << ','; first = false; write_string(out, item.first); out << ':'; write(out, item.second); }
        out << '}';
    }
}
inline std::string dump(const JsonValue& value) {
    std::ostringstream out; out.imbue(std::locale::classic()); write(out, value); return out.str() + "\n";
}

}  // namespace lnssat_json

#endif  // LNS_PUBLIC_JSON_H
