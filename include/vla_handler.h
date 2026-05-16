#pragma once

#include "bridge.h"
#include <thread>
#include <atomic>

class VlaHandler {
public:
    explicit VlaHandler(Bridge &bridge);
    ~VlaHandler();

    bool start(int port);
    void stop();

private:
    void thread_func();

    Bridge &m_bridge;
    int m_sock = -1;
    std::atomic<bool> m_running{false};
    std::thread m_thread;
};
