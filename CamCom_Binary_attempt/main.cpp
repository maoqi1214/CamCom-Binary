// main.cpp
#include <iostream>
#include <string>
#include <cstring>

// 声明外部函数 (这些函数分别在 encoder.cpp 和 decoder.cpp 中定义)
int run_encoder(int argc, char* argv[]);
int run_decoder(int argc, char* argv[]);

void print_global_usage(const char* program_name) {
    std::cerr << "Visible Light Communication Experiment Tool\n";
    std::cerr << "-------------------------------------------\n";
    std::cerr << "Usage:\n";
    std::cerr << "  " << program_name << " encode <in.bin> <out.mp4> <duration_ms>\n";
    std::cerr << "      Encodes a binary file into a video signal.\n";
    std::cerr << "\n";
    std::cerr << "  " << program_name << " decode <in.mp4> <out.bin> <valid_mask.bin>\n";
    std::cerr << "      Decodes a recorded video back to binary data.\n";
    std::cerr << "\n";
    std::cerr << "Examples:\n";
    std::cerr << "  " << program_name << " encode data.bin signal.mp4 1000\n";
    std::cerr << "  " << program_name << " decode recorded.mp4 recovered.bin mask.bin\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        print_global_usage(argv[0]);
        return 1; // BadArgs
    }

    std::string command = argv[1];

    if (command == "encode") {
        // 调整参数指针，让 run_encoder 以为 argv[0] 是 "encode"
        // run_encoder 期望: argv[1]=in.bin, argv[2]=out.mp4, argv[3]=duration
        // 当前实际: argv[1]="encode", argv[2]=in.bin...
        // 所以我们需要传递 (argc-1, argv+1)
        return run_encoder(argc - 1, argv + 1);
    }
    else if (command == "decode") {
        return run_decoder(argc - 1, argv + 1);
    }
    else {
        std::cerr << "Error: Unknown command '" << command << "'\n\n";
        print_global_usage(argv[0]);
        return 1;
    }
}