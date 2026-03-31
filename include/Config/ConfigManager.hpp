#pragma once

#include "Config.hpp"
#include <string>

class ConfigManager {
public:
    static void Load(const std::string& path);
    static const Config& Get();

private:
    static Config config_;
};