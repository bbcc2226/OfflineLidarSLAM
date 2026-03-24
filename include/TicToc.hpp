#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>
#include <iostream>
#include <string>

class TicToc {
public:
    TicToc() { tic(); }

    // start/reset timer
    void tic() { start_time_ = std::chrono::high_resolution_clock::now(); }

    // print elapsed time in milliseconds
    void toc(const std::string& msg = "") const {
        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time_).count();
        if(!msg.empty()) {
            std::cout << msg << ": ";
        }
        std::cout << elapsed_ms << " ms" << std::endl;
    }

    // return elapsed time in milliseconds
    double elapsed() const {
        auto end_time = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::milli>(end_time - start_time_).count();
    }

private:
    std::chrono::high_resolution_clock::time_point start_time_;
};

#endif
