#pragma once

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <string>
#include <memory>
#include <atomic>

enum class Mode { Manual, Auto };
const char* mode_to_str(Mode m);

struct BridgeConfig {
    std::string serial_url = "serial:///dev/ttyS3:115200";
    std::string mp_ip = "192.168.137.1";
    int mp_port = 14555;
    int vla_port = 14556;
};

struct LogConfig {
    bool heartbeat = false;   // 心跳包/飞行模式，默认关闭
    bool position = true;     // 位置信息
    bool armed = true;        // 解锁状态变化
    bool commands = true;     // 命令收发
    bool raw_bytes = false;   // 原始字节
    int  stats_interval = 10; // 统计打印间隔(秒)
};

class Bridge {
public:
    Bridge();
    ~Bridge();

    bool init(const BridgeConfig &cfg, const LogConfig &log);
    void run();
    void shutdown();
    bool running() const { return m_running; }

    // 命令（线程安全，VlaHandler/ConsoleHandler 调用）
    void cmd_arm();
    void cmd_disarm();
    void cmd_takeoff(float alt = 10.0f);
    void cmd_land();
    void cmd_rtl();
    void cmd_loiter();
    void set_mode(Mode mode);
    Mode get_mode() const { return m_mode; }
    void print_status();

    // VLA 命令处理
    void handle_vla_cmd(const char *cmd, char *resp, int resp_size);
    void exec_cmd(const char *cmd_name, float *args, int argc,
                        char *resp, int resp_size);
    void build_status_response(char *resp, int resp_size);
private:
    int parse_serial_url(const std::string &url, std::string &device, int &baudrate);
    int open_serial(const char *device, int baudrate);
    void setup_logging(const LogConfig &log);
    void process_io();
    void log_stats();

    std::atomic<bool> m_running{false};
    std::atomic<Mode> m_mode{Mode::Manual};
    std::atomic<bool> m_block_sdk_heartbeat{false};
    LogConfig m_log;

    // 串口
    int m_serial_fd = -1;

    // MP UDP
    int m_mp_sock = -1;
    struct sockaddr_in m_mp_addr{};
    socklen_t m_mp_addr_len = 0;
    bool m_mp_known = false;

    // MAVSDK 本地回环
    int m_loopback_sock = -1;
    struct sockaddr_in m_mavsdk_addr{};
    static constexpr int LOOPBACK_PORT = 14560;
    static constexpr int MAVSDK_PORT = 14561;

    // MAVSDK
    std::unique_ptr<mavsdk::Mavsdk> m_mavsdk;
    std::shared_ptr<mavsdk::System> m_sys;
    std::unique_ptr<mavsdk::Action> m_action;
    std::unique_ptr<mavsdk::Offboard> m_offboard;
    std::unique_ptr<mavsdk::Telemetry> m_telemetry;

    // 统计
    int m_serial_rx = 0;
    int m_mp_rx = 0;
    int m_sdk_rx = 0;
    time_t m_last_stats = 0;
    time_t m_last_pos_log = 0;

    float m_last_alt = -999;
    float m_last_lat = -999;
    float m_last_lon = -999;
    bool m_last_armed = false;
    mavsdk::Telemetry::FlightMode m_last_flight_mode =
        mavsdk::Telemetry::FlightMode::Unknown;
};
