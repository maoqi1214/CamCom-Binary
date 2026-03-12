#include "rs.hpp"
#include <iostream>
#include <vector>
#include <random>
#include <cassert>

using namespace camcom::rs;

int main() {
    init_tables();
    std::mt19937_64 rng(12345);
    std::uniform_int_distribution<int> byte_dist(0,255);

    // Test parameters
    const int msg_len = 100;
    const int nsym = 16; // parity bytes
    std::vector<uint8_t> msg(msg_len);
    for (int i = 0; i < msg_len; ++i) msg[i] = static_cast<uint8_t>(byte_dist(rng));

    auto parity = encode(msg, nsym);
    std::vector<uint8_t> codeword = msg;
    codeword.insert(codeword.end(), parity.begin(), parity.end());

    // introduce up to nsym/2 random byte errors (should be correctable)
    const int nerr = nsym/2 - 1;
    std::vector<int> err_pos;
    for (int i = 0; i < nerr; ++i) {
        int pos = rng() % codeword.size();
        err_pos.push_back(pos);
        codeword[pos] ^= static_cast<uint8_t>(byte_dist(rng));
    }

    bool ok = decode(codeword, nsym);
    if (!ok) {
        std::cerr << "RS decode failed\n";
        return 2;
    }

    // original message should be recovered in the first msg_len bytes
    for (int i = 0; i < msg_len; ++i) {
        if (codeword[i] != msg[i]) {
            std::cerr << "Mismatch at " << i << "\n";
            return 3;
        }
    }

    std::cout << "RS unit test passed\n";
    return 0;
}
