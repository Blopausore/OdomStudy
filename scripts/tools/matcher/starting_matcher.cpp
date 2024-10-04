#include <iostream>
#include <string>



const std::string help_message = R"(
    Usage: starting_matcher <input_rosbag> <output_rosbag> [options]
    Compare the given file with the iostream header file.
    Arguments:
        <input_rosbag>      The input rosbag file.
        <output_rosbag>     The output rosbag file.
    Options:
        -h, --help      Display this help message.
        -v, --verbose   Display verbose output.
        --matching_time_percentage <percentage>   The percentage of the matching time [default: 0.1].

)";


int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cout << "Error: Not enough arguments." << std::endl;
        std::cout << help_message << std::endl;
        return 1;
    }

    std::string input_rosbag = argv[1];
    std::string output_rosbag = argv[2];
    bool verbose = false;
    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            std::cout << help_message << std::endl;
            return 0;
        } else if (arg == "-v" || arg == "--verbose") {
            std::cout << "Verbose output enabled." << std::endl;
            verbose = true;
        } else if (arg == "--matching_time_percentage") {
            if (i + 1 >= argc) {
                std::cout << "Error: Missing argument for --matching_time_percentage." << std::endl;
                return 1;
            }
            std::string percentage_str = argv[i + 1];
            try {
                int percentage = std::stoi(percentage_str);
                std::cout << "Matching time percentage: " << percentage << std::endl;
            } catch (std::invalid_argument& e) {
                std::cout << "Error: Invalid argument for --matching_time_percentage." << std::endl;
                return 1;
            }
        }
    }
 
    return 0;
}