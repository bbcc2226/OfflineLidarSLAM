#pragma once

#include "Config.hpp"
#include <string>

class ConfigLoader {
public:
    static Config Load(const std::string& file_path);
};