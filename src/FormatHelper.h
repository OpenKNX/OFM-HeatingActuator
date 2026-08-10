#pragma once
#include <cstdio>
#include <string>

template <typename... Ts>
std::string string_format(const std::string& format, Ts... Args)
{
    const size_t n = std::snprintf(nullptr, 0, format.c_str(), Args...) + 1; // Extra space for '\0'
    std::string ret(n, '\0');
    std::snprintf(&ret.front(), n, format.c_str(), Args...);
    return ret;
}
