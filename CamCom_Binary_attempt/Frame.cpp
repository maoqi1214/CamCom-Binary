#include "Frame.h" 

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {
    const std::array<cv::Vec3b, 5> kPalette = {
        cv::Vec3b(255, 255, 255),
        cv::Vec3b(0, 0, 255),
        cv::Vec3b(0, 255, 0),
        cv::Vec3b(255, 0, 0),
        cv::Vec3b(0, 0, 0),
    };
}

Frame108::Frame108() {
    std::fill(&matrix[0][0], &matrix[0][0] + SIZE * SIZE, White);
}

bool Frame108::isAnchorCell(int x, int y) {
    const bool topLeft = (x < ANCHOR_SIZE && y < ANCHOR_SIZE);
    const bool topRight = (x >= SIZE - ANCHOR_SIZE && y < ANCHOR_SIZE);
    const bool bottomLeft = (x < ANCHOR_SIZE && y >= SIZE - ANCHOR_SIZE);
    return topLeft || topRight || bottomLeft;
}

bool Frame108::isDataCell(int x, int y) {
    if (y < ROW_DATA_START) {
        return false;
    }
    return !isAnchorCell(x, y);
}

cv::Vec3b Frame108::symbolToBgr(uint8_t symbol) {
    return kPalette.at(std::min<std::size_t>(symbol, kPalette.size() - 1));
}

uint8_t Frame108::bgrToSymbol(const cv::Vec3b& pixel) {
    const int blue = pixel[0];
    const int green = pixel[1];
    const int red = pixel[2];
    const int maxChannel = std::max({ blue, green, red });
    const int minChannel = std::min({ blue, green, red });
    const int chroma = maxChannel - minChannel;

    if (maxChannel < 70) {
        return Black;
    }

    // Low saturation bright pixels are treated as white.
    if (chroma < 35 && maxChannel > 145) {
        return White;
    }

    // Prefer dominant channel when it is clearly separated.
    if (red - std::max(green, blue) >= 25) {
        return Red;
    }
    if (green - std::max(red, blue) >= 25) {
        return Green;
    }
    if (blue - std::max(red, green) >= 25) {
        return Blue;
    }

    // Fallback: nearest palette color in BGR space.
    int bestDistance = std::numeric_limits<int>::max();
    uint8_t bestSymbol = White;
    for (uint8_t symbol : { White, Red, Green, Blue, Black }) {
        const cv::Vec3b ref = symbolToBgr(symbol);
        const int db = blue - ref[0];
        const int dg = green - ref[1];
        const int dr = red - ref[2];
        const int distance = db * db + dg * dg + dr * dr;
        if (distance < bestDistance) {
            bestDistance = distance;
            bestSymbol = symbol;
        }
    }
    return bestSymbol;
}

void Frame108::setAnchorPoints() {
    auto drawFinder = [&](int ox, int oy) {
        for (int y = 0; y < ANCHOR_SIZE; ++y) {
            for (int x = 0; x < ANCHOR_SIZE; ++x) {
                const bool isBorder = (x == 0 || x == ANCHOR_SIZE - 1 || y == 0 || y == ANCHOR_SIZE - 1);
                const bool isCenter = (x >= 2 && x <= 4 && y >= 2 && y <= 4);
                matrix[oy + y][ox + x] = (isBorder || isCenter) ? Black : White;
            }
        }
        };

    drawFinder(0, 0);
    drawFinder(SIZE - ANCHOR_SIZE, 0);
    drawFinder(0, SIZE - ANCHOR_SIZE);
}

void Frame108::setFrameId(uint32_t id, int bits) {
    std::fill(matrix[ROW_FRAME_ID_START], matrix[ROW_FRAME_ID_START] + SIZE, White);

    const int symbolCount = std::min(SIZE, (bits + DATA_BITS_PER_CELL - 1) / DATA_BITS_PER_CELL);
    for (int cell = 0; cell < symbolCount; ++cell) {
        uint8_t symbol = 0;
        for (int bit = 0; bit < DATA_BITS_PER_CELL; ++bit) {
            const int globalBit = cell * DATA_BITS_PER_CELL + bit;
            symbol <<= 1;
            if (globalBit < bits) {
                symbol |= static_cast<uint8_t>((id >> (bits - 1 - globalBit)) & 0x1U);
            }
        }
        matrix[ROW_FRAME_ID_START][cell] = symbol;
    }
}

void Frame108::setChecksum(uint32_t crc, int bits) {
    std::fill(matrix[ROW_CHECKSUM_START], matrix[ROW_CHECKSUM_START] + SIZE, White);

    const int symbolCount = std::min(SIZE, (bits + DATA_BITS_PER_CELL - 1) / DATA_BITS_PER_CELL);
    for (int cell = 0; cell < symbolCount; ++cell) {
        uint8_t symbol = 0;
        for (int bit = 0; bit < DATA_BITS_PER_CELL; ++bit) {
            const int globalBit = cell * DATA_BITS_PER_CELL + bit;
            symbol <<= 1;
            if (globalBit < bits) {
                symbol |= static_cast<uint8_t>((crc >> (bits - 1 - globalBit)) & 0x1U);
            }
        }
        matrix[ROW_CHECKSUM_START][cell] = symbol;
    }
}

void Frame108::setData(const std::vector<uint8_t>& data) {
    for (int y = ROW_DATA_START; y < SIZE; ++y) {
        for (int x = 0; x < SIZE; ++x) {
            if (isDataCell(x, y)) {
                matrix[y][x] = White;
            }
        }
    }

    const int totalBits = std::min<int>(static_cast<int>(data.size() * 8), DATA_BYTES_CAPACITY * 8);
    int bitIndex = 0;

    for (int y = ROW_DATA_START; y < SIZE; ++y) {
        for (int x = 0; x < SIZE; ++x) {
            if (!isDataCell(x, y)) {
                continue;
            }

            uint8_t symbol = 0;
            for (int bit = 0; bit < DATA_BITS_PER_CELL; ++bit) {
                symbol <<= 1;
                if (bitIndex < totalBits) {
                    const int byteIndex = bitIndex / 8;
                    const int bitInByte = 7 - (bitIndex % 8);
                    symbol |= static_cast<uint8_t>((data[byteIndex] >> bitInByte) & 0x1U);
                }
                ++bitIndex;
            }
            matrix[y][x] = symbol;
        }
    }
}

void Frame108::encode(uint32_t frameId, const std::vector<uint8_t>& data, uint32_t crc) {
    std::fill(&matrix[0][0], &matrix[0][0] + SIZE * SIZE, White);
    setAnchorPoints();
    setFrameId(frameId);
    setChecksum(crc);
    setData(data);
}

cv::Mat Frame108::toImage() const {
    cv::Mat img(SIZE, SIZE, CV_8UC3);
    for (int y = 0; y < SIZE; ++y) {
        for (int x = 0; x < SIZE; ++x) {
            img.at<cv::Vec3b>(y, x) = symbolToBgr(matrix[y][x]);
        }
    }
    return img;
}

std::vector<cv::Point2f> Frame108::findAnchorCorners(const cv::Mat& blackMask) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(blackMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    struct Candidate {
        double area;
        cv::Point2f center;
    };

    std::vector<Candidate> candidates;
    const double imageArea = static_cast<double>(blackMask.rows * blackMask.cols);
    const double minArea = std::max(25.0, imageArea * 0.0004);
    const double maxArea = imageArea * 0.25;

    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < minArea || area > maxArea) {
            continue;
        }

        const cv::RotatedRect box = cv::minAreaRect(contour);
        const float width = std::max(box.size.width, 1.0f);
        const float height = std::max(box.size.height, 1.0f);
        const float aspect = width / height;
        if (aspect < 0.6f || aspect > 1.4f) {
            continue;
        }

        candidates.push_back({ area, box.center });
    }

    if (candidates.size() < 3) {
        return {};
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
        return lhs.area > rhs.area;
        });

    std::vector<Candidate> uniqueCandidates;
    const float dedupeDistance = std::min(blackMask.cols, blackMask.rows) * 0.08f;
    for (const auto& candidate : candidates) {
        bool duplicate = false;
        for (const auto& chosen : uniqueCandidates) {
            if (cv::norm(candidate.center - chosen.center) < dedupeDistance) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            uniqueCandidates.push_back(candidate);
        }
        if (uniqueCandidates.size() >= 6) {
            break;
        }
    }

    if (uniqueCandidates.size() < 3) {
        return {};
    }

    auto tl = std::min_element(uniqueCandidates.begin(), uniqueCandidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
        return (lhs.center.x + lhs.center.y) < (rhs.center.x + rhs.center.y);
        });
    auto tr = std::max_element(uniqueCandidates.begin(), uniqueCandidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
        return (lhs.center.x - lhs.center.y) < (rhs.center.x - rhs.center.y);
        });
    auto bl = std::max_element(uniqueCandidates.begin(), uniqueCandidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
        return (lhs.center.y - lhs.center.x) < (rhs.center.y - rhs.center.x);
        });

    if (tl == tr || tl == bl || tr == bl) {
        return {};
    }

    const cv::Point2f topLeft = tl->center;
    const cv::Point2f topRight = tr->center;
    const cv::Point2f bottomLeft = bl->center;
    const cv::Point2f bottomRight = topRight + (bottomLeft - topLeft);

    return { topLeft, topRight, bottomRight, bottomLeft };
}

bool Frame108::fromImageRobust(const cv::Mat& rawInput, Frame108& outFrame, int dataBits) {
    if (rawInput.empty() || dataBits <= 0) {
        return false;
    }

    cv::Mat bgr;
    if (rawInput.channels() == 3) {
        bgr = rawInput;
    }
    else if (rawInput.channels() == 4) {
        cv::cvtColor(rawInput, bgr, cv::COLOR_BGRA2BGR);
    }
    else {
        cv::cvtColor(rawInput, bgr, cv::COLOR_GRAY2BGR);
    }

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    cv::Mat blackMask;
    cv::threshold(gray, blackMask, 60, 255, cv::THRESH_BINARY_INV);
    cv::morphologyEx(blackMask, blackMask, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    const std::vector<cv::Point2f> srcPoints = findAnchorCorners(blackMask);
    if (srcPoints.size() != 4) {
        return false;
    }

    const std::vector<cv::Point2f> dstPoints = {
        { 0.0f, 0.0f },
        { static_cast<float>(SIZE - 1), 0.0f },
        { static_cast<float>(SIZE - 1), static_cast<float>(SIZE - 1) },
        { 0.0f, static_cast<float>(SIZE - 1) },
    };

    const cv::Mat transform = cv::getPerspectiveTransform(srcPoints, dstPoints);
    cv::Mat warped;
    cv::warpPerspective(bgr, warped, transform, cv::Size(SIZE, SIZE), cv::INTER_NEAREST);

    outFrame = Frame108::fromImage(warped); // Correctly assigns to the reference
    return true;
}

Frame108 Frame108::fromImage(const cv::Mat& img) {
    Frame108 frame;
    if (img.rows != SIZE || img.cols != SIZE) {
        // Or handle error differently if size mismatch is critical
        // For now, just return an empty frame
        return frame;
    }

    if (img.channels() == 1) {
        for (int y = 0; y < SIZE; ++y) {
            for (int x = 0; x < SIZE; ++x) {
                frame.matrix[y][x] = (img.at<uint8_t>(y, x) < 128) ? Black : White;
            }
        }
        return frame;
    }

    for (int y = 0; y < SIZE; ++y) {
        for (int x = 0; x < SIZE; ++x) {
            frame.matrix[y][x] = bgrToSymbol(img.at<cv::Vec3b>(y, x));
        }
    }
    return frame;
}

uint32_t Frame108::getFrameId(int bits) const {
    uint32_t id = 0;
    const int symbolCount = std::min(SIZE, (bits + DATA_BITS_PER_CELL - 1) / DATA_BITS_PER_CELL);
    int recoveredBits = 0;

    for (int cell = 0; cell < symbolCount && recoveredBits < bits; ++cell) {
        const uint8_t symbol = matrix[ROW_FRAME_ID_START][cell] & 0x3U;
        for (int bit = DATA_BITS_PER_CELL - 1; bit >= 0 && recoveredBits < bits; --bit) {
            id = (id << 1) | ((symbol >> bit) & 0x1U);
            ++recoveredBits;
        }
    }
    return id;
}

uint32_t Frame108::getChecksum(int bits) const {
    uint32_t crc = 0;
    const int symbolCount = std::min(SIZE, (bits + DATA_BITS_PER_CELL - 1) / DATA_BITS_PER_CELL);
    int recoveredBits = 0;

    for (int cell = 0; cell < symbolCount && recoveredBits < bits; ++cell) {
        const uint8_t symbol = matrix[ROW_CHECKSUM_START][cell] & 0x3U;
        for (int bit = DATA_BITS_PER_CELL - 1; bit >= 0 && recoveredBits < bits; --bit) {
            crc = (crc << 1) | ((symbol >> bit) & 0x1U);
            ++recoveredBits;
        }
    }
    return crc;
}

std::vector<uint8_t> Frame108::getData(int dataBits) const {
    const int cappedBits = std::min(dataBits, DATA_BYTES_CAPACITY * 8);
    std::vector<uint8_t> data((cappedBits + 7) / 8, 0);
    int bitIndex = 0;

    for (int y = ROW_DATA_START; y < SIZE && bitIndex < cappedBits; ++y) {
        for (int x = 0; x < SIZE && bitIndex < cappedBits; ++x) {
            if (!isDataCell(x, y)) {
                continue;
            }

            const uint8_t symbol = matrix[y][x] & 0x3U;
            for (int bit = DATA_BITS_PER_CELL - 1; bit >= 0 && bitIndex < cappedBits; --bit) {
                const int byteIndex = bitIndex / 8;
                const int bitInByte = 7 - (bitIndex % 8);
                data[byteIndex] |= static_cast<uint8_t>(((symbol >> bit) & 0x1U) << bitInByte);
                ++bitIndex;
            }
        }
    }

    return data;
}