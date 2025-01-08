#include "Timer.h"

Timer::Timer() : accumulated_time_(0), is_running_(false) {}

void Timer::start() {
    if (!is_running_) {
        start_time_ = std::chrono::steady_clock::now();
        is_running_ = true;
    }
}

void Timer::stop() {
    if (is_running_) {
        accumulated_time_ += std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time_).count();
        is_running_ = false;
    }
}

void Timer::reset() {
    accumulated_time_ = 0;
    is_running_ = false;
}

long long Timer::elapsedMilliseconds() const {
    if (is_running_) {
        return accumulated_time_ + std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time_).count();
    } else {
        return accumulated_time_;
    }
}

bool Timer::isExpired(long long durationMilliseconds) const {
    return elapsedMilliseconds() >= durationMilliseconds;
}

bool Timer::isRunning() const {
    return is_running_;
}