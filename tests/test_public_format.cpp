#include "lnssat/Json.h"
#include "lnssat/Sha256.h"
#include <iostream>
#include <stdexcept>
using namespace lnssat_json;
static void require(bool value) { if (!value) throw std::runtime_error("public format assertion failed"); }
int main() {
    require(lnssat::sha256("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    require(lnssat::sha256("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    require(lnssat::sha256(std::string(1000000,'a')) == "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
    std::string controls; for (int i=0;i<32;++i) controls += static_cast<char>(i);
    const std::string text = controls + "quote\" slash\\ UTF-8 é 😄";
    require(parse(dump(JsonValue(text))).as_string() == text);
    require(parse("\"\\u00e9 \\ud83d\\ude04\"").as_string() == "é 😄");
    require(dump(JsonObject{{"z",2},{"a",1}}) == "{\"a\":1,\"z\":2}\n");
    for (const auto& invalid : {"{\"a\":0,\"a\":1}","[1,]","{","[1]x","01","1e999","9223372036854775808","\"\\ud800\"","\"\\udc00\"","\"\n\"","\"\\x00\""}) {
        bool rejected=false;try { (void)parse(invalid); } catch(const std::exception&) { rejected=true; } require(rejected);
    }
    for (const auto& invalid : {std::string(130,'[')+std::string(130,']'),std::string("\"\xc0\x80\""),std::string("\"\xed\xa0\x80\"")}) {
        bool rejected=false;try { (void)parse(invalid); } catch(const std::exception&) { rejected=true; } require(rejected);
    }
    std::cout << "PASS: SHA-256 known vectors, Unicode/control escaping, strict JSON and deterministic object ordering\n";
}
