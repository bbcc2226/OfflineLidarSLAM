#include "ConfigManager.hpp"
#include "ConfigLoader.hpp"
#include <iostream>

Config ConfigManager::config_;

void ConfigManager::Load(const std::string& path) {
    config_ = ConfigLoader::Load(path);
    std::cout << "[ConfigManager] Config loaded from: " << path << std::endl;
}

const Config& ConfigManager::Get() {
    return config_;
}