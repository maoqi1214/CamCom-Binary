#include "codec.hpp"

#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: sample_probe <image>\n";
        return 2;
    }

    const cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr << "Failed to read image: " << argv[1] << "\n";
        return 1;
    }

    camcom::EncoderConfig cfg = camcom::make_default_encoder_config();
    std::vector<uint8_t> payload;
    const bool ok = camcom::sample_frame(img, payload, cfg);

    std::cout << "sample_ok=" << (ok ? 1 : 0) << "\n";
    std::cout << "payload_size=" << payload.size() << "\n";
    if (!payload.empty()) {
        std::cout << "head=";
        const size_t n = std::min<size_t>(payload.size(), 48);
        for (size_t i = 0; i < n; ++i) {
            if (i) {
                std::cout << ' ';
            }
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(payload[i]);
        }
        std::cout << std::dec << "\n";
    }
    return 0;
}
