#pragma once
#include <queue>

struct sensor_data_t {
    std::queue<double> timestamp_q;

    sensor_data_t() { timestamp_q = std::queue<double>(); }
};