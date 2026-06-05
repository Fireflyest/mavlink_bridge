#include "console_handler.h"
#include <iostream>
#include <string>
#include <cstdio>
#include <sstream>

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
           "  goto lat lon alt [yaw]   - 飞到指定坐标\n"
           "  position lat lon alt [yaw] - goto 别名\n"
           "  set_home [lat lon alt]   - 设置/查看自定义 mark 点\n"
           "  set_mode auto|manual     - 切换桥接模式\n"
           "  auto        - 切换到自动模式 (别名)\n"
           "  manual      - 切换到手动模式 (别名)\n"
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

        std::istringstream iss(cmd);
        std::string op;
        iss >> op;

        if (op == "quit" || op == "exit") {
            printf("退出中...\n");
            m_bridge.shutdown();
        } else if (op == "arm") {
            m_bridge.cmd_arm();
        } else if (op == "disarm") {
            m_bridge.cmd_disarm();
        } else if (op == "takeoff") {
            float alt = 10.0f;
            iss >> alt;
            m_bridge.cmd_takeoff(alt);
        } else if (op == "land") {
            m_bridge.cmd_land();
        } else if (op == "rtl") {
            m_bridge.cmd_rtl();
        } else if (op == "loiter") {
            m_bridge.cmd_loiter();
        } else if (op == "goto" || op == "position") {
            double latitude_deg;
            double longitude_deg;
            float absolute_altitude_m;
            float yaw_deg;
            if (iss >> latitude_deg >> longitude_deg >> absolute_altitude_m) {
                if (!(iss >> yaw_deg)) {
                    yaw_deg = 0.0f;
                }
                m_bridge.cmd_goto(latitude_deg, longitude_deg, absolute_altitude_m, yaw_deg);
            } else {
                printf("用法: %s lat lon alt [yaw]\n", op.c_str());
            }
        } else if (op == "set_home") {
            double latitude_deg;
            double longitude_deg;
            float absolute_altitude_m;
            if (iss >> latitude_deg >> longitude_deg >> absolute_altitude_m) {
                m_bridge.cmd_set_mark(latitude_deg, longitude_deg, absolute_altitude_m);
            } else {
                double mark_lat;
                double mark_lon;
                float mark_alt;
                if (m_bridge.get_mark_point(mark_lat, mark_lon, mark_alt)) {
                    printf("[MARK] (%.7f, %.7f, %.1fm)\n", mark_lat, mark_lon, mark_alt);
                } else {
                    printf("[MARK] 未设置自定义 mark 点\n");
                }
            }
        } else if (op == "set_mode") {
            std::string mode;
            if (iss >> mode) {
                if (mode == "auto" || mode == "AUTO" || mode == "1") {
                    m_bridge.set_mode(Mode::Auto);
                } else if (mode == "manual" || mode == "MANUAL" || mode == "0") {
                    m_bridge.set_mode(Mode::Manual);
                } else {
                    printf("用法: set_mode auto|manual\n");
                }
            } else {
                printf("用法: set_mode auto|manual\n");
            }
        } else if (op == "auto") {
            m_bridge.set_mode(Mode::Auto);
        } else if (op == "manual") {
            m_bridge.set_mode(Mode::Manual);
        } else if (op == "status") {
            m_bridge.print_status();
        } else if (op == "help") {
            print_help();
        } else if (!cmd.empty()) {
            printf("未知命令: %s\n", cmd.c_str());
        }
    }
}
