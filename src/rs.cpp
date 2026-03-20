// Reed-Solomon encoding and decoding over GF(256).

#include "rs.hpp"

#include <mutex>
#include <vector>

namespace camcom { namespace rs {

static bool tables_inited = false;
static uint8_t gf_exp[512];
static uint8_t gf_log[256];
static std::once_flag init_flag;

void init_tables() {
    std::call_once(init_flag, []() {
        int x = 1;
        for (int i = 0; i < 255; ++i) {
            gf_exp[i] = static_cast<uint8_t>(x);
            gf_log[static_cast<uint8_t>(x)] = static_cast<uint8_t>(i);
            x <<= 1;
            if (x & 0x100) {
                x ^= 0x11d;
            }
        }
        for (int i = 255; i < 512; ++i) {
            gf_exp[i] = gf_exp[i - 255];
        }
        gf_log[0] = 0;
        tables_inited = true;
    });
}

static inline uint8_t gf_mul(uint8_t a, uint8_t b) {
    if (a == 0 || b == 0) {
        return 0;
    }
    return gf_exp[(gf_log[a] + gf_log[b]) % 255];
}

static inline uint8_t gf_pow(uint8_t a, int power) {
    if (power == 0) {
        return 1;
    }
    if (a == 0) {
        return 0;
    }
    const int log = gf_log[a];
    const int idx = (log * power) % 255;
    return gf_exp[idx];
}

static std::vector<uint8_t> poly_mul(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    std::vector<uint8_t> out(a.size() + b.size() - 1);
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] == 0) {
            continue;
        }
        for (size_t j = 0; j < b.size(); ++j) {
            if (b[j] == 0) {
                continue;
            }
            out[i + j] ^= gf_mul(a[i], b[j]);
        }
    }
    return out;
}

static std::vector<uint8_t> gen_generator_poly(int nsym) {
    std::vector<uint8_t> g = {1};
    for (int i = 0; i < nsym; ++i) {
        std::vector<uint8_t> term = {1, gf_exp[i]};
        g = poly_mul(g, term);
    }
    return g;
}

std::vector<uint8_t> encode(const std::vector<uint8_t>& msg, int nsym) {
    init_tables();
    std::vector<uint8_t> gen = gen_generator_poly(nsym);
    std::vector<uint8_t> parity(nsym, 0);
    for (size_t i = 0; i < msg.size(); ++i) {
        const uint8_t factor = msg[i] ^ parity[0];
        for (int j = 0; j < nsym - 1; ++j) {
            parity[j] = parity[j + 1];
        }
        parity[nsym - 1] = 0;
        if (factor != 0) {
            for (int j = 0; j < nsym; ++j) {
                parity[j] ^= gf_mul(gen[j + 1], factor);
            }
        }
    }
    return parity;
}

static std::vector<uint8_t> berlekamp_massey(const std::vector<uint8_t>& synd, int nsym) {
    std::vector<uint8_t> c(1, 1);
    std::vector<uint8_t> b(1, 1);
    int l = 0;
    int m = 1;
    uint8_t bb = 1;
    for (int n = 0; n < nsym; ++n) {
        uint8_t d = 0;
        for (int i = 0; i <= l; ++i) {
            d ^= gf_mul(c[i], synd[n - i]);
        }
        if (d == 0) {
            ++m;
        } else if (2 * l <= n) {
            std::vector<uint8_t> t = c;
            const uint8_t inv_b = gf_exp[(255 - gf_log[bb]) % 255];
            const uint8_t scale = gf_mul(d, inv_b);
            std::vector<uint8_t> scaled(b.size() + m);
            for (size_t i = 0; i < b.size(); ++i) {
                scaled[i + m] = gf_mul(b[i], scale);
            }
            c.resize(std::max(c.size(), scaled.size()));
            for (size_t i = 0; i < scaled.size(); ++i) {
                c[i] ^= scaled[i];
            }
            l = n + 1 - l;
            b = t;
            bb = d;
            m = 1;
        } else {
            const uint8_t inv_b = gf_exp[(255 - gf_log[bb]) % 255];
            const uint8_t scale = gf_mul(d, inv_b);
            std::vector<uint8_t> scaled(b.size() + m);
            for (size_t i = 0; i < b.size(); ++i) {
                scaled[i + m] = gf_mul(b[i], scale);
            }
            c.resize(std::max(c.size(), scaled.size()));
            for (size_t i = 0; i < scaled.size(); ++i) {
                c[i] ^= scaled[i];
            }
            ++m;
        }
    }
    return c;
}

static std::vector<int> find_error_locations(const std::vector<uint8_t>& err_loc) {
    std::vector<int> locs;
    const int errs = static_cast<int>(err_loc.size()) - 1;
    for (int i = 0; i < 255; ++i) {
        const uint8_t xi = gf_exp[(255 - i) % 255];
        uint8_t sum = 0;
        for (int j = static_cast<int>(err_loc.size()) - 1; j >= 0; --j) {
            sum = gf_mul(sum, xi) ^ err_loc[j];
        }
        if (sum == 0) {
            locs.push_back(255 - i);
            if (static_cast<int>(locs.size()) == errs) {
                break;
            }
        }
    }
    return locs;
}

static std::vector<uint8_t> calc_error_evaluator(
    const std::vector<uint8_t>& synd,
    const std::vector<uint8_t>& err_loc,
    int nsym) {
    std::vector<uint8_t> prod = poly_mul(synd, err_loc);
    prod.resize(nsym);
    return prod;
}

static uint8_t gf_inverse(uint8_t a) {
    if (a == 0) {
        return 0;
    }
    return gf_exp[255 - gf_log[a]];
}

bool decode(std::vector<uint8_t>& codeword, int nsym) {
    init_tables();
    const int n = static_cast<int>(codeword.size());
    if (nsym <= 0 || nsym >= n) {
        return false;
    }

    std::vector<uint8_t> synd(nsym);
    bool all_zero = true;
    for (int i = 0; i < nsym; ++i) {
        uint8_t s = 0;
        const uint8_t x = gf_exp[i];
        for (int j = 0; j < n; ++j) {
            s = gf_mul(s, x) ^ codeword[j];
        }
        synd[i] = s;
        if (s != 0) {
            all_zero = false;
        }
    }
    if (all_zero) {
        return true;
    }

    std::vector<uint8_t> synd_poly = synd;
    std::vector<uint8_t> err_loc = berlekamp_massey(synd_poly, nsym);
    const int errs = static_cast<int>(err_loc.size()) - 1;
    if (errs * 2 > nsym) {
        return false;
    }

    std::vector<int> pos = find_error_locations(err_loc);
    if (pos.empty()) {
        return false;
    }

    std::vector<uint8_t> err_eval = calc_error_evaluator(synd_poly, err_loc, nsym);

    for (size_t i = 0; i < pos.size(); ++i) {
        const int position = pos[i];
        const int xi_inv = (255 - position) % 255;
        const uint8_t x = gf_exp[xi_inv];

        uint8_t denom = 0;
        for (int j = 1; j < static_cast<int>(err_loc.size()); ++j) {
            denom ^= gf_mul(err_loc[j], gf_pow(x, j));
        }
        if (denom == 0) {
            return false;
        }

        uint8_t y = 0;
        for (int j = static_cast<int>(err_eval.size()) - 1; j >= 0; --j) {
            y = gf_mul(y, x) ^ err_eval[j];
        }
        const uint8_t magnitude = gf_mul(y, gf_inverse(denom));

        const int idx = n - 1 - position;
        if (idx < 0 || idx >= n) {
            return false;
        }
        codeword[idx] ^= magnitude;
    }

    for (int i = 0; i < nsym; ++i) {
        uint8_t s = 0;
        const uint8_t x = gf_exp[i];
        for (int j = 0; j < n; ++j) {
            s = gf_mul(s, x) ^ codeword[j];
        }
        if (s != 0) {
            return false;
        }
    }
    return true;
}

}} // namespace camcom::rs
