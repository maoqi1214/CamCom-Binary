#include "codec.hpp"
#include "rs.hpp"
#include "common.hpp"
#include <opencv2/opencv.hpp>
#include <random>
#include <iostream>
#include <vector>
#include <cassert>

using namespace camcom;

static void serialize_u8(std::vector<uint8_t>& buf, uint8_t v) { buf.push_back(v); }
static void serialize_u32(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back(static_cast<uint8_t>(v & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFFu));
}
static void serialize_u64(std::vector<uint8_t>& buf, uint64_t v) {
    for (int i = 0; i < 8; ++i) buf.push_back(static_cast<uint8_t>((v >> (8*i)) & 0xFFu));
}

static int popcount8(uint8_t x) {
    // portable popcount
    int c = 0;
    while (x) { c += x & 1; x >>= 1; }
    return c;
}

int main() {
    // generate random payload
    std::mt19937_64 rng(123456);
    std::uniform_int_distribution<int> bd(0,255);
    std::vector<uint8_t> payload(1024);
    for (size_t i=0;i<payload.size();++i) payload[i]=static_cast<uint8_t>(bd(rng));

    EncoderConfig cfg;
    cfg.cell_size = 10;
    cfg.cells_per_row = 32;
    cfg.payload_bytes_per_frame = 128;
    cfg.rs_nsym = 16;
    cfg.reference_block_size = 2;
    cfg.fps = 10;

    const size_t payload_per_frame = static_cast<size_t>(cfg.payload_bytes_per_frame);
    const uint32_t total_frames = static_cast<uint32_t>((payload.size() + payload_per_frame - 1) / payload_per_frame);

    cv::Mat img;
    const int fourcc = cv::VideoWriter::fourcc('m','p','4','v');
    cv::Size sim_size(cfg.cells_per_row*cfg.cell_size + 8*cfg.cell_size, cfg.cell_size*8 + 8*cfg.cell_size);
    cv::VideoWriter writer("tests/out_simulated.mp4", fourcc, cfg.fps, sim_size);
    if (!writer.isOpened()) { std::cerr<<"Failed to open writer"<<std::endl; return 2; }

    // bootstrap repeat
    std::vector<uint8_t> bootstrap_buf;
    serialize_u32(bootstrap_buf, MAGIC);
    serialize_u8(bootstrap_buf, FORMAT_VERSION);
    serialize_u32(bootstrap_buf, static_cast<uint32_t>(cfg.rs_nsym));
    serialize_u32(bootstrap_buf, static_cast<uint32_t>(cfg.cell_size));
    serialize_u32(bootstrap_buf, static_cast<uint32_t>(cfg.cells_per_row));
    serialize_u32(bootstrap_buf, static_cast<uint32_t>(cfg.payload_bytes_per_frame));
    serialize_u32(bootstrap_buf, static_cast<uint32_t>(cfg.fps));
    serialize_u32(bootstrap_buf, static_cast<uint32_t>(cfg.reference_block_size));

    const int BOOTSTRAP_REPEAT = 3;
    for (int i=0;i<BOOTSTRAP_REPEAT;++i) {
        render_frame(img, bootstrap_buf, cfg);
        writer.write(img);
        img.setTo(cv::Scalar(0,0,0)); writer.write(img);
    }

    // stream header RS-protected repeat
    std::vector<uint8_t> stream_buf;
    serialize_u32(stream_buf, MAGIC);
    serialize_u8(stream_buf, FORMAT_VERSION);
    serialize_u64(stream_buf, static_cast<uint64_t>(payload.size()));
    serialize_u8(stream_buf, static_cast<uint8_t>(Encoding::Binary));
    serialize_u32(stream_buf, static_cast<uint32_t>(cfg.fps));
    serialize_u32(stream_buf, static_cast<uint32_t>(cfg.cell_size));
    serialize_u32(stream_buf, static_cast<uint32_t>(cfg.rs_nsym));
    serialize_u32(stream_buf, static_cast<uint32_t>(cfg.payload_bytes_per_frame));
    serialize_u32(stream_buf, static_cast<uint32_t>(cfg.cells_per_row));
    serialize_u32(stream_buf, total_frames);

    auto sp = rs::encode(stream_buf, cfg.rs_nsym);
    stream_buf.insert(stream_buf.end(), sp.begin(), sp.end());
    const int STREAMHDR_REPEAT = 3;
    for (int i=0;i<STREAMHDR_REPEAT;++i) { render_frame(img, stream_buf, cfg); writer.write(img); img.setTo(cv::Scalar(0,0,0)); writer.write(img);} 

    // data frames
    for (uint32_t fi=0; fi<total_frames; ++fi) {
        size_t offset = static_cast<size_t>(fi) * payload_per_frame;
        size_t remain = (offset < payload.size()) ? (payload.size()-offset) : 0;
        size_t chunk = std::min(remain, payload_per_frame);
        std::vector<uint8_t> frame_buf;
        serialize_u32(frame_buf, MAGIC);
        serialize_u8(frame_buf, FORMAT_VERSION);
        serialize_u32(frame_buf, fi);
        serialize_u32(frame_buf, total_frames);
        serialize_u32(frame_buf, static_cast<uint32_t>(chunk));
        // placeholder checksum
        size_t checksum_pos = frame_buf.size();
        serialize_u32(frame_buf, 0);
        frame_buf.insert(frame_buf.end(), payload.begin()+offset, payload.begin()+offset+chunk);
        uint32_t checksum = crc32(frame_buf.data()+checksum_pos+4, chunk);
        frame_buf[checksum_pos+0] = static_cast<uint8_t>(checksum & 0xFFu);
        frame_buf[checksum_pos+1] = static_cast<uint8_t>((checksum>>8)&0xFFu);
        frame_buf[checksum_pos+2] = static_cast<uint8_t>((checksum>>16)&0xFFu);
        frame_buf[checksum_pos+3] = static_cast<uint8_t>((checksum>>24)&0xFFu);
        auto parity = rs::encode(frame_buf, cfg.rs_nsym);
        frame_buf.insert(frame_buf.end(), parity.begin(), parity.end());
        render_frame(img, frame_buf, cfg);
        writer.write(img);
        img.setTo(cv::Scalar(0,0,0)); writer.write(img);
    }

    writer.release();
    std::cout << "Wrote simulated source video tests/out_simulated.mp4"<<std::endl;

    // Now simulate capture by re-opening and adding blur/noise
    cv::VideoCapture cap_in("tests/out_simulated.mp4");
    if (!cap_in.isOpened()) { std::cerr<<"open failed"<<std::endl; return 3; }
    int sim_frame_count = static_cast<int>(cap_in.get(cv::CAP_PROP_FRAME_COUNT));
    cv::VideoWriter writer2("tests/out_captured.mp4", fourcc, cfg.fps, cv::Size(cap_in.get(cv::CAP_PROP_FRAME_WIDTH), cap_in.get(cv::CAP_PROP_FRAME_HEIGHT)));
    cv::Mat f;
    while (cap_in.read(f)) {
        // add gaussian blur and noise
        cv::Mat b; cv::GaussianBlur(f, b, cv::Size(3,3), 1.0);
        cv::Mat noise = cv::Mat::zeros(b.size(), b.type());
        for (int y=0;y<noise.rows;++y) for (int x=0;x<noise.cols;++x) for (int c=0;c<3;++c) noise.at<cv::Vec3b>(y,x)[c] = static_cast<unsigned char>((bd(rng)%5));
        cv::Mat out; cv::add(b, noise, out);
        writer2.write(out);
    }
    writer2.release();
    cap_in.release();
    std::cout << "Wrote simulated captured video tests/out_captured.mp4"<<std::endl;

    // Now attempt to decode simulated captured video using sampling (no perspective applied)
    cv::VideoCapture cap("tests/out_captured.mp4");
    if (!cap.isOpened()) { std::cerr<<"open2 failed"<<std::endl; return 4; }
    bool have_bootstrap=false, have_stream=false;
    uint32_t stream_rs=0;
    uint32_t expected_frames=0;
    std::vector<std::vector<uint8_t>> frames_buf;
    std::vector<uint8_t> validity;

    int total_video_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    int frames_read = 0;
    int frames_processed = 0;

    while (cap.read(f)) {
        ++frames_read;
        double var = laplacian_variance(f);
        if (var < 5.0) continue;
        ++frames_processed;
        std::vector<uint8_t> sample;
        if (!sample_frame(f, sample, cfg)) continue;
        if (!have_bootstrap) {
            if (sample.size() >= 28) {
                const uint8_t* p = sample.data();
                uint32_t magic = p[0] | (p[1]<<8) | (p[2]<<16) | (p[3]<<24);
                uint8_t version = p[4];
                if (magic==MAGIC && version==FORMAT_VERSION) {
                    const uint8_t* q = p+5;
                    uint32_t rs_n = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
                    uint32_t cell = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
                    uint32_t cellspr = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
                    uint32_t payloadpf = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
                    uint32_t fpsr = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
                    uint32_t refb = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24);
                    cfg.rs_nsym = static_cast<int>(rs_n);
                    cfg.cell_size = static_cast<int>(cell);
                    cfg.cells_per_row = static_cast<int>(cellspr);
                    cfg.payload_bytes_per_frame = static_cast<int>(payloadpf);
                    cfg.reference_block_size = static_cast<int>(refb);
                    stream_rs = rs_n;
                    have_bootstrap=true; validity.push_back(1); continue;
                }
            }
            validity.push_back(0); continue;
        }
        if (have_bootstrap && !have_stream) {
            std::vector<uint8_t> code = sample;
            if (stream_rs>0) {
                if (!rs::decode(code, stream_rs)) { validity.push_back(0); continue; }
            }
            const uint8_t* p = code.data();
            uint32_t magic = p[0] | (p[1]<<8) | (p[2]<<16) | (p[3]<<24);
            uint8_t version = p[4];
            if (!(magic==MAGIC && version==FORMAT_VERSION)) { validity.push_back(0); continue; }
            // read total data bytes
            uint64_t total_data=0; for (int i=0;i<8;++i) total_data |= static_cast<uint64_t>(p[5+i]) << (8*i);
            const uint8_t* q = p+5+8;
            uint8_t encoding = q[0]; q+=1;
            uint32_t fpsr = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
            uint32_t cell = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
            uint32_t rsn = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
            uint32_t payloadpf = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
            uint32_t cellspr = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
            uint32_t totalf = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24);
            stream_rs = rsn; expected_frames = totalf; frames_buf.resize(expected_frames);
            have_stream=true; validity.push_back(1); continue;
        }
        // data frames
        std::vector<uint8_t> code = sample;
        if (stream_rs>0) {
            if (!rs::decode(code, stream_rs)) { validity.push_back(0); continue; }
        }
        const uint8_t* p = code.data();
        uint32_t magic = p[0] | (p[1]<<8) | (p[2]<<16) | (p[3]<<24);
        uint8_t version = p[4];
        if (!(magic==MAGIC && version==FORMAT_VERSION)) { validity.push_back(0); continue; }
        const uint8_t* q = p+5;
        uint32_t frame_idx = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
        uint32_t totalf = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
        uint32_t payloadb = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
        uint32_t checksum = q[0] | (q[1]<<8) | (q[2]<<16) | (q[3]<<24); q+=4;
        if (payloadb>0 && q + payloadb <= code.data()+code.size()) {
            uint32_t calc = crc32(q, payloadb);
            if (calc!=checksum) { validity.push_back(0); continue; }
            if (frame_idx < frames_buf.size()) frames_buf[frame_idx] = std::vector<uint8_t>(q, q+payloadb);
            validity.push_back(1);
        } else { validity.push_back(0); }
    }

    // reassemble
    std::vector<uint8_t> recovered;
    for (uint32_t i=0;i<expected_frames;++i) {
        if (i < frames_buf.size() && !frames_buf[i].empty()) recovered.insert(recovered.end(), frames_buf[i].begin(), frames_buf[i].end());
    }
    recovered.resize(payload.size());

    // metrics
    uint64_t total_bits = payload.size() * 8ULL;
    uint64_t bit_errors = 0;
    for (size_t i=0;i<payload.size() && i<recovered.size();++i) {
        uint8_t diff = payload[i] ^ recovered[i];
        bit_errors += popcount8(diff);
    }
    double ber = (total_bits==0)? 0.0 : static_cast<double>(bit_errors) / static_cast<double>(total_bits);

    // duration estimate from captured video frame count
    cv::VideoCapture cap_info("tests/out_captured.mp4");
    int cap_frames = static_cast<int>(cap_info.get(cv::CAP_PROP_FRAME_COUNT));
    double duration_s = (cfg.fps>0) ? (static_cast<double>(cap_frames) / cfg.fps) : 0.0;
    uint64_t correct_bits = total_bits - bit_errors;
    double bps = (duration_s>0.0) ? static_cast<double>(correct_bits) / duration_s : 0.0;

    int frames_decoded = 0;
    for (auto &v : frames_buf) if (!v.empty()) ++frames_decoded;
    double frame_success_rate = (expected_frames>0) ? static_cast<double>(frames_decoded) / static_cast<double>(expected_frames) : 0.0;

    std::cout<<"Integration metrics:\n";
    std::cout<<"  Total bits: "<<total_bits<<"\n";
    std::cout<<"  Bit errors: "<<bit_errors<<"\n";
    std::cout<<"  BER: "<<ber<<"\n";
    std::cout<<"  Throughput (correct bits/sec): "<<bps<<"\n";
    std::cout<<"  Expected frames: "<<expected_frames<<" Decoded frames: "<<frames_decoded<<"\n";
    std::cout<<"  Frame success rate: "<<frame_success_rate<<"\n";
    std::cout<<"  Video frames read: "<<frames_read<<" processed: "<<frames_processed<<" total file frames: "<<cap_frames<<"\n";
    if (recovered == payload) std::cout<<"Integration test success: payload recovered fully"<<std::endl;
    else std::cout<<"Integration test failed: mismatch"<<std::endl;

    return 0;
}
