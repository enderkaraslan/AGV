#pragma once

#include <chrono>

class Timer {
public:
    Timer();
    void start();
    void stop();
    void reset();
    long long elapsedMilliseconds() const;
    bool isExpired(long long durationMilliseconds) const;
    bool isRunning() const;

private:
    std::chrono::steady_clock::time_point start_time_;
    long long accumulated_time_; // Duraklatmalar arasında biriken süre
    bool is_running_;
};