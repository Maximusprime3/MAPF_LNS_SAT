#include "lnssat/Sha256.h"
#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>

namespace lnssat {
namespace {
uint32_t rotate(uint32_t x, unsigned n) { return (x >> n) | (x << (32 - n)); }
constexpr uint32_t k[] = {
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
}
std::string sha256(const std::string& bytes) {
    std::array<uint32_t,8> hash{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
    const uint64_t bits = static_cast<uint64_t>(bytes.size()) * 8;
    const size_t blocks = (bytes.size() + 9 + 63) / 64;
    for (size_t block = 0; block < blocks; ++block) {
        uint32_t w[64]{};
        for (size_t j = 0; j < 64; ++j) {
            const size_t index = block * 64 + j;
            unsigned char byte = 0;
            if (index < bytes.size()) byte = static_cast<unsigned char>(bytes[index]);
            else if (index == bytes.size()) byte = 0x80;
            else if (index >= blocks * 64 - 8) byte = static_cast<unsigned char>(bits >> ((blocks * 64 - 1 - index) * 8));
            w[j / 4] |= uint32_t(byte) << (24 - (j % 4) * 8);
        }
        for (size_t j = 16; j < 64; ++j) {
            const uint32_t s0 = rotate(w[j-15],7) ^ rotate(w[j-15],18) ^ (w[j-15] >> 3);
            const uint32_t s1 = rotate(w[j-2],17) ^ rotate(w[j-2],19) ^ (w[j-2] >> 10);
            w[j] = w[j-16] + s0 + w[j-7] + s1;
        }
        auto v = hash;
        for (size_t j = 0; j < 64; ++j) {
            const uint32_t s1 = rotate(v[4],6) ^ rotate(v[4],11) ^ rotate(v[4],25);
            const uint32_t ch = (v[4] & v[5]) ^ (~v[4] & v[6]);
            const uint32_t t1 = v[7] + s1 + ch + k[j] + w[j];
            const uint32_t s0 = rotate(v[0],2) ^ rotate(v[0],13) ^ rotate(v[0],22);
            const uint32_t maj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);
            const uint32_t t2 = s0 + maj;
            v = {t1+t2,v[0],v[1],v[2],v[3]+t1,v[4],v[5],v[6]};
        }
        for (size_t j = 0; j < 8; ++j) hash[j] += v[j];
    }
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (uint32_t word : hash) output << std::setw(8) << word;
    return output.str();
}
}
