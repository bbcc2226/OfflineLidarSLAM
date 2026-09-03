#include "ConfigManager.hpp"
#include "SlamProcess.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

void PrintUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [--config <path>] [ROS arguments]\n"
              << "\n"
              << "Options:\n"
              << "  -c, --config <path>  Use a specific YAML configuration file.\n"
              << "  -h, --help           Show this help message.\n"
              << "\n"
              << "When --config is omitted, the installed package configuration is used.\n";
}

std::string ParseConfigPath(int argc, char* argv[]) {
    std::string config_path;

    for (int i = 1; i < argc; ++i) {
        const std::string argument(argv[i]);
        if (argument == "-h" || argument == "--help") {
            PrintUsage(argv[0]);
            return {};
        }
        if (argument == "-c" || argument == "--config") {
            if (i + 1 >= argc) {
                throw std::invalid_argument(argument + " requires a file path");
            }
            config_path = argv[++i];
        }
    }

    if (config_path.empty()) {
        config_path =
            ament_index_cpp::get_package_share_directory("offline_lidar_slam") +
            "/config/Config.yaml";
    }
    return config_path;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const std::string config_path = ParseConfigPath(argc, argv);
        if (config_path.empty()) {
            return 0;
        }
        if (!std::filesystem::is_regular_file(config_path)) {
            std::cerr << "Configuration file does not exist: " << config_path << '\n';
            return 1;
        }

        rclcpp::init(argc, argv);
        ConfigManager::Load(config_path);

        {
            SlamProcess slam;
            while (rclcpp::ok() && !slam.IsFinished()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // On normal completion, wait for every worker to exit. On Ctrl+C,
            // SlamProcess's destructor requests shutdown before joining workers.
            if (slam.IsFinished()) {
                slam.Join();
            }
        }

        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "offline_lidar_slam_node: " << error.what() << '\n';
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }
}
