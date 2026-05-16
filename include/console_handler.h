#pragma once

#include "bridge.h"
#include <thread>
#include <atomic>

class ConsoleHandler {
public:
    explicit ConsoleHandler(Bridge &bridge);
    ~ConsoleHandler();

    void start();
    void stop();

private:
    void thread_func();
    void print_help();

    Bridge &m_bridge;
    std::atomic<bool> m_running{false};
    std::thread m_thread;
};
