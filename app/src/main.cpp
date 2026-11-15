#include "simulation_app.hpp"

#include <iostream>
#include <string>

/**
 * @file main.cpp
 * @brief Entry point for the N-Body gravity simulator executable.
 */

namespace {

/**
 * @brief Prints CLI usage and available launch-time command line options.
 * @param prog_name Name of the executable binary.
 */
void print_help(const char* prog_name) {
    std::cout << "Usage: " << prog_name << " [options]\n\n"
              << "Options:\n"
              << "  --integrator <euler|rk4>       Integrator algorithm (default: euler)\n"
              << "  --preset <orbit|solar|random>  Simulation preset (default: orbit)\n"
              << "  --help, -h                     Show this help message\n";
}

} // namespace

/**
 * @brief Main execution function parsing CLI arguments and launching SimulationApp.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Process exit code (0 on success, non-zero on error).
 */
int main(int argc, char* argv[]) {
    SimConfig config;

    // Parse command line arguments for initial launch configuration
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            print_help(argv[0]);
            return 0;
        } else if (arg == "--integrator") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --integrator requires an argument (euler|rk4)\n";
                print_help(argv[0]);
                return 1;
            }
            std::string val = argv[++i];
            if (val != "euler" && val != "rk4") {
                std::cerr << "Error: Invalid integrator '" << val << "'. Must be 'euler' or 'rk4'.\n";
                print_help(argv[0]);
                return 1;
            }
            config.integrator = val;
        } else if (arg.rfind("--integrator=", 0) == 0) {
            std::string val = arg.substr(std::string("--integrator=").length());
            if (val != "euler" && val != "rk4") {
                std::cerr << "Error: Invalid integrator '" << val << "'. Must be 'euler' or 'rk4'.\n";
                print_help(argv[0]);
                return 1;
            }
            config.integrator = val;
        } else if (arg == "--preset") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --preset requires an argument (orbit|solar|random)\n";
                print_help(argv[0]);
                return 1;
            }
            std::string val = argv[++i];
            if (val != "orbit" && val != "solar" && val != "random") {
                std::cerr << "Error: Invalid preset '" << val << "'. Must be 'orbit', 'solar', or 'random'.\n";
                print_help(argv[0]);
                return 1;
            }
            config.preset = val;
        } else if (arg.rfind("--preset=", 0) == 0) {
            std::string val = arg.substr(std::string("--preset=").length());
            if (val != "orbit" && val != "solar" && val != "random") {
                std::cerr << "Error: Invalid preset '" << val << "'. Must be 'orbit', 'solar', or 'random'.\n";
                print_help(argv[0]);
                return 1;
            }
            config.preset = val;
        } else {
            std::cerr << "Error: Unknown option '" << arg << "'\n";
            print_help(argv[0]);
            return 1;
        }
    }

    // Launch application and enter SFML main loop
    SimulationApp app(config);
    app.run();

    return 0;
}
