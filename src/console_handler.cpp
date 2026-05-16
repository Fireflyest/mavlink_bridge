#include "console_handler.h"
#include <iostream>
#include <string>
#include <cstdio>

ConsoleHandler::ConsoleHandler(Bridge &bridge) : m_bridge(bridge) {}
ConsoleHandler::~ConsoleHandler() { stop(); }

void ConsoleHandler::start() {
    print_help();
    m_running = true;
    m_thread = std::thread(&ConsoleHandler::thread_func, this);
}

void ConsoleHandler::stop() {
    m_running = false;
    if (m_thread.joinable()) m_thread.join();
}

void ConsoleHandler::print_help() {
    printf("\n可用命令:\n"
           "  arm         - 解锁电机\n"
           "  disarm      - 锁定电机\n"
           "  takeoff [m] - 起飞 (默认10m)\n"
           "  land        - 降落\n"
           "  rtl         - 返航\n"
           "  loiter      - 悬停\n"
           "  auto        - 切换到自动模式\n"
           "  manual      - 切换到手动模式\n"
           "  status      - 显示状态\n"
           "  quit        - 退出\n"
           "  help        - 显示帮助\n\n");
}

void ConsoleHandler::thread_func() {
    std::string cmd;
    while (m_running && m_bridge.running()) {
        if (!std::getline(std::cin, cmd)) {
            m_bridge.shutdown();
            break;
        }

        if (cmd == "quit" || cmd == "exit") {
            printf("退出中...\n");
            m_bridge.shutdown();
        } else if (cmd == "arm") {
            m_bridge.cmd_arm();
        } else if (cmd == "disarm") {
            m_bridge.cmd_disarm();
        } else if (cmd.substr(0, 7) == "takeoff") {
            float alt = 10.0f;
            if (cmd.length() > 8) alt = std::stof(cmd.substr(8));
            m_bridge.cmd_takeoff(alt);
        } else if (cmd == "land") {
            m_bridge.cmd_land();
        } else if (cmd == "rtl") {
            m_bridge.cmd_rtl();
        } else if (cmd == "loiter") {
            m_bridge.cmd_loiter();
        } else if (cmd == "auto") {
            m_bridge.set_mode(Mode::Auto);
        } else if (cmd == "manual") {
            m_bridge.set_mode(Mode::Manual);
        } else if (cmd == "status") {
            m_bridge.print_status();
        } else if (cmd == "help") {
            print_help();
        } else if (!cmd.empty()) {
            printf("未知命令: %s\n", cmd.c_str());
        }
    }
}
