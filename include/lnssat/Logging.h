#pragma once
#include "lnssat/SolverConfiguration.h"
#include <iostream>
#include <streambuf>

namespace lnssat {
// Per-thread explicit sink, scoped to a solve. Never replace global stream buffers.
class NullLogBuffer : public std::streambuf {
    int overflow(int ch) override { return traits_type::not_eof(ch); }
};
inline thread_local LogLevel active_log_level = LogLevel::Quiet;
inline thread_local std::ostream* active_log_sink = &std::cerr;
inline std::ostream& debug_log() {
    static thread_local NullLogBuffer buffer;
    static thread_local std::ostream discarded(&buffer);
    return active_log_level == LogLevel::Debug ? *active_log_sink : discarded;
}
inline std::ostream& info_log() {
    return active_log_level == LogLevel::Quiet ? debug_log() : *active_log_sink;
}
struct LogScope {
    LogLevel previous_level = active_log_level;
    std::ostream* previous_sink = active_log_sink;
    explicit LogScope(LogLevel level, std::ostream* sink = nullptr) {
        active_log_level = level;
        if (sink) active_log_sink = sink;
    }
    ~LogScope() { active_log_level = previous_level; active_log_sink = previous_sink; }
    LogScope(const LogScope&) = delete;
    LogScope& operator=(const LogScope&) = delete;
};
}
