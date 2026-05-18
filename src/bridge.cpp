#include "bridge.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <thread>
#include <chrono>
#include <algorithm>

const char* mode_to_str(Mode m) {
    return m == Mode::Auto ? "AUTO" : "MANUAL";
}

Bridge::Bridge() = default;
Bridge::~Bridge() { shutdown(); }

int Bridge::parse_serial_url(const std::string &url, std::string &device, int &baudrate) {
    if (url.find("serial://") != 0) return -1;
    std::string rest = url.substr(9);
    auto colon = rest.rfind(':');
    if (colon != std::string::npos) {
        device = rest.substr(0, colon);
        baudrate = std::stoi(rest.substr(colon + 1));
    } else {
        device = rest;
        baudrate = 115200;
    }
    return 0;
}

int Bridge::open_serial(const char *device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open serial"); return -1; }

    struct termios tio{};
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    cfsetispeed(&tio, baudrate);
    cfsetospeed(&tio, baudrate);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tio);
    tcflush(fd, TCIOFLUSH);
    return fd;
}

/* 从原始字节中提取消息 ID（支持 v1/v2） */
static uint32_t extract_msgid(const uint8_t *buf, int len) {
    if (len < 1) return 0xFFFF;
    if (buf[0] == 0xFE && len >= 6)  return buf[5];                          // v1
    if (buf[0] == 0xFD && len >= 10) return buf[7]|(buf[8]<<8)|(buf[9]<<16); // v2
    return 0xFFFF;
}

void Bridge::setup_logging(const LogConfig &log) {
    m_telemetry->subscribe_position([this](mavsdk::Telemetry::Position pos) {
        if (!m_log.position) return;
        /* 变化检测：高度变化 <0.3m 且位置变化 <0.000001° 时跳过 */
        float alt_diff = std::abs(pos.relative_altitude_m - m_last_alt);
        float lat_diff = std::abs(pos.latitude_deg - m_last_lat);
        float lon_diff = std::abs(pos.longitude_deg - m_last_lon);
        if (alt_diff < 0.3f && lat_diff < 0.000001f && lon_diff < 0.000001f)
            return;
        m_last_alt = pos.relative_altitude_m;
        m_last_lat = pos.latitude_deg;
        m_last_lon = pos.longitude_deg;
        printf("[遥测] 高度=%.1fm 位置=(%.7f, %.7f)\n",
               pos.relative_altitude_m, pos.latitude_deg, pos.longitude_deg);
    });

    m_telemetry->subscribe_armed([this](bool armed) {
        if (!m_log.armed) return;
        /* 只在状态变化时输出 */
        if (armed == m_last_armed) return;
        m_last_armed = armed;
        printf("[遥测] %s\n", armed ? "已解锁" : "已锁定");
    });

    m_telemetry->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode mode) {
        if (!m_log.heartbeat) return;
        if (mode == m_last_flight_mode) return;
        m_last_flight_mode = mode;
        printf("[心跳] 飞行模式: %d\n", (int)mode);
    });
}


bool Bridge::init(const BridgeConfig &cfg, const LogConfig &log) {
    m_log = log;

    printf("=== MAVLink 桥接 ===\n");
    printf("串口: %s\n", cfg.serial_url.c_str());
    printf("MP:   %s:%d (字节级转发)\n", cfg.mp_ip.c_str(), cfg.mp_port);
    printf("VLA:  UDP %d\n", cfg.vla_port);
    printf("模式: %s\n\n", mode_to_str(m_mode));

    /* 1. 串口 */
    std::string device;
    int baudrate;
    if (parse_serial_url(cfg.serial_url, device, baudrate) < 0) {
        fprintf(stderr, "串口URL格式错误: %s\n", cfg.serial_url.c_str());
        return false;
    }
    m_serial_fd = open_serial(device.c_str(), baudrate);
    if (m_serial_fd < 0) return false;
    printf("[串口] 已打开 %s @ %d\n", device.c_str(), baudrate);

    /* 2. MP UDP */
    int reuse = 1;
    m_mp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    setsockopt(m_mp_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    struct sockaddr_in mp_bind{};
    mp_bind.sin_family = AF_INET;
    mp_bind.sin_port = htons(cfg.mp_port);
    mp_bind.sin_addr.s_addr = INADDR_ANY;
    if (bind(m_mp_sock, (struct sockaddr *)&mp_bind, sizeof(mp_bind)) < 0) {
        perror("MP bind"); return false;
    }
    m_mp_addr.sin_family = AF_INET;
    m_mp_addr.sin_port = htons(cfg.mp_port);
    m_mp_addr.sin_addr.s_addr = inet_addr(cfg.mp_ip.c_str());
    m_mp_addr_len = sizeof(m_mp_addr);
    m_mp_known = true;
    printf("[MP]   UDP %d → %s:%d\n", cfg.mp_port, cfg.mp_ip.c_str(), cfg.mp_port);

    /* 3. MAVSDK 本地回环 */
    m_loopback_sock = socket(AF_INET, SOCK_DGRAM, 0);
    setsockopt(m_loopback_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    struct sockaddr_in loop_bind{};
    loop_bind.sin_family = AF_INET;
    loop_bind.sin_port = htons(LOOPBACK_PORT);
    loop_bind.sin_addr.s_addr = INADDR_ANY;
    if (bind(m_loopback_sock, (struct sockaddr *)&loop_bind, sizeof(loop_bind)) < 0) {
        perror("loopback bind"); return false;
    }
    m_mavsdk_addr.sin_family = AF_INET;
    m_mavsdk_addr.sin_port = htons(MAVSDK_PORT);
    m_mavsdk_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    printf("[SDK]  本地回环 %d ↔ %d\n", LOOPBACK_PORT, MAVSDK_PORT);

    /* 4. MAVSDK */
    m_mavsdk = std::make_unique<mavsdk::Mavsdk>(
        mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::CompanionComputer});

    std::string sdk_url = "udpin://0.0.0.0:" + std::to_string(MAVSDK_PORT);
    if (m_mavsdk->add_any_connection(sdk_url) != mavsdk::ConnectionResult::Success) {
        fprintf(stderr, "MAVSDK 连接失败\n");
        return false;
    }

    /* 5. 等待飞控（转发线程保证数据流通） */
    printf("等待飞控连接...\n");
    std::atomic<bool> fwd_running{true};
    std::thread fwd_thread([&]() {
        while (fwd_running) {
            fd_set fds;
            struct timeval tv = {0, 100000};
            FD_ZERO(&fds);
            FD_SET(m_serial_fd, &fds);
            if (select(m_serial_fd + 1, &fds, nullptr, nullptr, &tv) > 0) {
                uint8_t buf[2048];
                int n = read(m_serial_fd, buf, sizeof(buf));
                if (n > 0) {
                    sendto(m_loopback_sock, buf, n, 0,
                           (struct sockaddr *)&m_mavsdk_addr, sizeof(m_mavsdk_addr));
                    if (m_mp_known) {
                        sendto(m_mp_sock, buf, n, 0,
                               (struct sockaddr *)&m_mp_addr, m_mp_addr_len);
                    }
                }
            }
        }
    });

    auto sys_opt = m_mavsdk->first_autopilot(30.0);
    fwd_running = false;
    fwd_thread.join();

    if (!sys_opt) {
        fprintf(stderr, "飞控连接超时\n");
        return false;
    }
    m_sys = sys_opt.value();
    printf("飞控已连接 (sysid=%d)\n\n", m_sys->get_system_id());

    /* 初始化完成，开始拦截 SDK 心跳，避免干扰 MP */
    m_block_sdk_heartbeat = true;

    /* 6. 插件 + 日志 */
    m_action = std::make_unique<mavsdk::Action>(m_sys);
    m_offboard = std::make_unique<mavsdk::Offboard>(m_sys);
    m_telemetry = std::make_unique<mavsdk::Telemetry>(m_sys);
    setup_logging(log);

    m_running = true;
    m_last_stats = time(NULL);
    return true;
}

void Bridge::shutdown() {
    m_running = false;
    m_telemetry.reset();
    m_offboard.reset();
    m_action.reset();
    m_sys.reset();
    m_mavsdk.reset();
    if (m_serial_fd >= 0) { close(m_serial_fd); m_serial_fd = -1; }
    if (m_mp_sock >= 0) { close(m_mp_sock); m_mp_sock = -1; }
    if (m_loopback_sock >= 0) { close(m_loopback_sock); m_loopback_sock = -1; }
}

void Bridge::run() {
    while (m_running) {
        process_io();
        log_stats();
    }
}

void Bridge::process_io() {
    fd_set fds;
    struct timeval tv = {0, 50000};

    FD_ZERO(&fds);
    FD_SET(m_serial_fd, &fds);
    FD_SET(m_loopback_sock, &fds);
    FD_SET(m_mp_sock, &fds);
    int maxfd = std::max({m_serial_fd, m_loopback_sock, m_mp_sock});

    int ret = select(maxfd + 1, &fds, nullptr, nullptr, &tv);
    if (ret <= 0) return;

    /* 串口 → MAVSDK + MP */
    if (FD_ISSET(m_serial_fd, &fds)) {
        uint8_t buf[2048];
        int n = read(m_serial_fd, buf, sizeof(buf));
        if (n > 0) {
            m_serial_rx++;

            // → MAVSDK（遥测解析）
            sendto(m_loopback_sock, buf, n, 0,
                   (struct sockaddr *)&m_mavsdk_addr, sizeof(m_mavsdk_addr));

            // → MP（原始字节）
            if (m_mp_known) {
                sendto(m_mp_sock, buf, n, 0,
                       (struct sockaddr *)&m_mp_addr, m_mp_addr_len);
            }

            if (m_log.raw_bytes) {
                printf("[串口→] %d 字节", n);
                for (int i = 0; i < std::min(n, 16); i++)
                    printf(" %02X", buf[i]);
                if (n > 16) printf(" ...");
                printf("\n");
            }
        }
    }

    /* MAVSDK → 串口（初始化后拦截心跳，其他放行） */
    if (FD_ISSET(m_loopback_sock, &fds)) {
        uint8_t buf[2048];
        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        int n = recvfrom(m_loopback_sock, buf, sizeof(buf), 0,
                         (struct sockaddr *)&from, &fromlen);
        if (n > 0) {
            m_sdk_rx++;
            write(m_serial_fd, buf, n);
            // 不打印，避免刷屏。用户命令通过 cmd_xxx() 已有日志。
        }
    }

    /* MP → 串口（原始字节） */
    if (FD_ISSET(m_mp_sock, &fds)) {
        uint8_t buf[2048];
        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        int n = recvfrom(m_mp_sock, buf, sizeof(buf), 0,
                         (struct sockaddr *)&from, &fromlen);
        if (n > 0) {
            m_mp_rx++;

            if (from.sin_addr.s_addr != m_mp_addr.sin_addr.s_addr ||
                from.sin_port != m_mp_addr.sin_port) {
                m_mp_addr = from;
                m_mp_addr_len = fromlen;
                printf("[MP] 更新地址: %s:%d\n",
                       inet_ntoa(from.sin_addr), ntohs(from.sin_port));
            }

            write(m_serial_fd, buf, n);

            if (m_log.raw_bytes) {
                printf("[MP→串口] %d 字节\n", n);
            }
        }
    }
}

void Bridge::log_stats() {
    if (m_log.stats_interval <= 0) return;
    time_t now = time(NULL);
    if (now - m_last_stats < m_log.stats_interval) return;
    m_last_stats = now;
    printf("[统计] 串口RX=%d MP_RX=%d SDK_RX=%d\n", m_serial_rx, m_mp_rx, m_sdk_rx);
}

void Bridge::cmd_arm() {
    auto r = m_action->arm();
    if (m_log.commands) printf("[CMD] ARM: %d\n", (int)r);
}

void Bridge::cmd_disarm() {
    auto r = m_action->disarm();
    if (m_log.commands) printf("[CMD] DISARM: %d\n", (int)r);
}

void Bridge::cmd_takeoff(float alt) {
    m_action->set_takeoff_altitude(alt);
    auto r = m_action->takeoff();
    if (m_log.commands) printf("[CMD] TAKEOFF %.1fm: %d\n", alt, (int)r);
}

void Bridge::cmd_land() {
    auto r = m_action->land();
    if (m_log.commands) printf("[CMD] LAND: %d\n", (int)r);
}

void Bridge::cmd_rtl() {
    auto r = m_action->return_to_launch();
    if (m_log.commands) printf("[CMD] RTL: %d\n", (int)r);
}

void Bridge::cmd_loiter() {
    auto r = m_action->hold();
    if (m_log.commands) printf("[CMD] LOITER: %d\n", (int)r);
}

void Bridge::set_mode(Mode mode) {
    m_mode = mode;
    printf("[MODE] %s\n", mode_to_str(mode));
}

void Bridge::print_status() {
    auto pos = m_telemetry->position();
    auto armed = m_telemetry->armed();
    printf("[状态] 模式=%s 高度=%.1fm 位置=(%.7f, %.7f) %s\n",
           mode_to_str(m_mode), pos.relative_altitude_m,
           pos.latitude_deg, pos.longitude_deg,
           armed ? "ARMED" : "DISARMED");
}

void Bridge::handle_vla_cmd(const char *cmd, char *resp, int resp_size) {
    // 解析命令名和参数
    char cmd_name[64];
    float args[8];
    int argc = 0;

    // "VELOCITY 1.0 0.0 0.0" → cmd_name="VELOCITY", args=[1.0, 0.0, 0.0]
    if (sscanf(cmd, "%63s", cmd_name) != 1) {
        snprintf(resp, resp_size, "ERR parse");
        return;
    }

    // 提取参数
    const char *p = cmd + strlen(cmd_name);
    while (*p && argc < 8) {
        while (*p == ' ') p++;
        if (!*p) break;
        args[argc++] = strtof(p, (char**)&p);
    }

    // 分发执行
    exec_cmd(cmd_name, args, argc, resp, resp_size);
}

void Bridge::exec_cmd(const char *cmd_name, float *args, int argc,
                       char *resp, int resp_size) {
    auto ok = [&](const char *msg) { snprintf(resp, resp_size, "OK %s", msg); };
    auto err = [&](const char *msg, int code) { snprintf(resp, resp_size, "ERR %s (%d)", msg, code); };

    mavsdk::Action::Result r;

    // === 飞行控制 ===
    if (strcmp(cmd_name, "ARM") == 0) {
        r = m_action->arm();
        r == mavsdk::Action::Result::Success ? ok("ARM") : err("ARM", (int)r);
    }
    else if (strcmp(cmd_name, "DISARM") == 0) {
        r = m_action->disarm();
        r == mavsdk::Action::Result::Success ? ok("DISARM") : err("DISARM", (int)r);
    }
    else if (strcmp(cmd_name, "TAKEOFF") == 0) {
        float alt = argc > 0 ? args[0] : 10.f;
        m_action->set_takeoff_altitude(alt);
        r = m_action->takeoff();
        r == mavsdk::Action::Result::Success
            ? ok(("TAKEOFF " + std::to_string((int)alt)).c_str())
            : err("TAKEOFF", (int)r);
    }
    else if (strcmp(cmd_name, "LAND") == 0) {
        r = m_action->land();
        r == mavsdk::Action::Result::Success ? ok("LAND") : err("LAND", (int)r);
    }
    else if (strcmp(cmd_name, "RTL") == 0) {
        r = m_action->return_to_launch();
        r == mavsdk::Action::Result::Success ? ok("RTL") : err("RTL", (int)r);
    }
    else if (strcmp(cmd_name, "HOLD") == 0 || strcmp(cmd_name, "LOITER") == 0) {
        r = m_action->hold();
        r == mavsdk::Action::Result::Success ? ok("HOLD") : err("HOLD", (int)r);
    }
    else if (strcmp(cmd_name, "VELOCITY") == 0) {
        float vx = argc > 0 ? args[0] : 0;
        float vy = argc > 1 ? args[1] : 0;
        float vz = argc > 2 ? args[2] : 0;
        float duration = argc > 3 ? args[3] : 0;  // 0=不自动停止

        /* 安全限制 */
        const float MAX_VEL = 3.0f;
        const float MAX_VZ = 2.0f;
        const float MAX_DUR = 30.0f;
        vx = std::max(-MAX_VEL, std::min(MAX_VEL, vx));
        vy = std::max(-MAX_VEL, std::min(MAX_VEL, vy));
        vz = std::max(-MAX_VZ, std::min(MAX_VZ, vz));
        if (duration > 0) duration = std::min(duration, MAX_DUR);

        /* 启动 Offboard */
        if (m_offboard->is_active() != true) {
            m_offboard->set_velocity_ned({0, 0, 0, 0});
            if (m_offboard->start() != mavsdk::Offboard::Result::Success) {
                err("VELOCITY start offboard", 0);
                return;
            }
        }

        m_offboard->set_velocity_ned({vx, vy, vz, 0});

        if (m_log.commands)
            printf("[CMD] VELOCITY %.2f %.2f %.2f dur=%.1fs\n",
                   vx, vy, vz, duration);

        /* 如果指定了持续时间，等待后自动停止 */
        if (duration > 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds((int)(duration * 1000)));

            /* 停止，悬停 */
            m_offboard->set_velocity_ned({0, 0, 0, 0});
            m_offboard->stop();
            m_action->hold();

            if (m_log.commands)
                printf("[CMD] VELOCITY %.1fs 已停止，悬停\n", duration);

            snprintf(resp, resp_size, "OK VELOCITY %.2f %.2f %.2f %.1f",
                     vx, vy, vz, duration);
        } else {
            snprintf(resp, resp_size, "OK VELOCITY %.2f %.2f %.2f",
                     vx, vy, vz);
        }
    }
    else if (strcmp(cmd_name, "STOP_OFFBOARD") == 0) {
        /* 停止 Offboard 模式 */
        if (m_offboard->is_active()) {
            m_offboard->stop();
        }
        /* 切回悬停 */
        m_action->hold();
        ok("STOP_OFFBOARD");
    }
    else if (strcmp(cmd_name, "YAW") == 0) {
        float target_delta = argc > 0 ? args[0] : 0;

        /* 安全限制 */
        const float MAX_YAW = 360.0f;
        target_delta = std::max(-MAX_YAW, std::min(MAX_YAW, target_delta));

        /* 获取当前偏航角 */
        auto euler = m_telemetry->attitude_euler();
        float start_yaw = euler.yaw_deg;  // [-180, 180]
        float target_yaw = start_yaw + target_delta;

        /* 归一化到 [-180, 180] */
        while (target_yaw > 180.f)  target_yaw -= 360.f;
        while (target_yaw < -180.f) target_yaw += 360.f;

        printf("[YAW] 当前=%.1f° 目标=%.1f° (delta=%.1f°)\n",
               start_yaw, target_yaw, target_delta);

        /* 启动 Offboard */
        if (m_offboard->is_active() != true) {
            m_offboard->set_velocity_ned({0, 0, 0, 0});
            if (m_offboard->start() != mavsdk::Offboard::Result::Success) {
                err("YAW start offboard", 0);
                return;
            }
        }

        /* 开始旋转 */
        float yaw_rate = 30.0f;
        if (target_delta < 0) yaw_rate = -30.0f;
        m_offboard->set_velocity_ned({0, 0, 0, yaw_rate});

        /* 闭环等待：实时检查偏航角 */
        const float angle_tolerance = 5.0f;   // 5° 容差
        const int timeout_ms = (int)(std::abs(target_delta) / 30.f * 1000.f) + 5000;
        auto start_time = std::chrono::steady_clock::now();

        bool reached = false;
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            auto now = std::chrono::steady_clock::now();
            int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 now - start_time).count();
            if (elapsed_ms > timeout_ms) {
                printf("[YAW] 超时 (%dms)\n", elapsed_ms);
                break;
            }

            euler = m_telemetry->attitude_euler();
            float current_yaw = euler.yaw_deg;

            /* 计算剩余角度（处理环绕） */
            float remaining = target_yaw - current_yaw;
            if (remaining > 180.f)  remaining -= 360.f;
            if (remaining < -180.f) remaining += 360.f;

            if (std::abs(remaining) <= angle_tolerance) {
                printf("[YAW] 到达目标 %.1f° (实际 %.1f°, 剩余 %.1f°)\n",
                       target_yaw, current_yaw, remaining);
                reached = true;
                break;
            }

            /* 超调检测：如果已经开始反向走，说明过了 */
            if ((target_delta > 0 && remaining < -angle_tolerance) ||
                (target_delta < 0 && remaining > angle_tolerance)) {
                printf("[YAW] 超调检测：实际 %.1f°, 剩余 %.1f°\n",
                       current_yaw, remaining);
                reached = true;
                break;
            }

            if (m_log.commands && elapsed_ms % 1000 < 100) {
                printf("[YAW] 旋转中... %.1f° → %.1f° (剩余 %.1f°)\n",
                       current_yaw, target_yaw, remaining);
            }
        }

        /* 停止旋转，悬停 */
        m_offboard->set_velocity_ned({0, 0, 0, 0});
        m_offboard->stop();
        m_action->hold();

        if (reached) {
            snprintf(resp, resp_size, "OK YAW %.1f", target_delta);
        } else {
            snprintf(resp, resp_size, "ERR YAW timeout");
        }
    }
    else if (strcmp(cmd_name, "WAIT") == 0) {
        /* WAIT 不做任何事，只返回 OK，客户端负责等待 */
        int sec = argc > 0 ? (int)args[0] : 1;
        if (m_log.commands)
            printf("[CMD] WAIT %ds\n", sec);
        snprintf(resp, resp_size, "OK WAIT %d", sec);
    }


    // === 状态查询 ===
    else if (strcmp(cmd_name, "HEARTBEAT") == 0 || strcmp(cmd_name, "STATUS") == 0) {
        build_status_response(resp, resp_size);
    }
    else if (strcmp(cmd_name, "GET_STATUS") == 0) {
        build_status_response(resp, resp_size);
    }

    // === 未知命令 ===
    else {
        snprintf(resp, resp_size, "ERR unknown: %s", cmd_name);
    }
}


void Bridge::build_status_response(char *resp, int resp_size) {
    auto pos = m_telemetry->position();
    auto armed = m_telemetry->armed();
    auto flight_mode = m_telemetry->flight_mode();

    const char* mode_str = "UNKNOWN";
    switch (flight_mode) {
        case mavsdk::Telemetry::FlightMode::Ready:          mode_str = "READY"; break;
        case mavsdk::Telemetry::FlightMode::Takeoff:        mode_str = "TAKEOFF"; break;
        case mavsdk::Telemetry::FlightMode::Hold:           mode_str = "HOLD"; break;
        case mavsdk::Telemetry::FlightMode::Mission:        mode_str = "AUTO"; break;
        case mavsdk::Telemetry::FlightMode::ReturnToLaunch: mode_str = "RTL"; break;
        case mavsdk::Telemetry::FlightMode::Land:           mode_str = "LAND"; break;
        case mavsdk::Telemetry::FlightMode::Offboard:       mode_str = "OFFBOARD"; break;
        case mavsdk::Telemetry::FlightMode::Manual:         mode_str = "MANUAL"; break;
        case mavsdk::Telemetry::FlightMode::Altctl:         mode_str = "ALTCTL"; break;
        case mavsdk::Telemetry::FlightMode::Posctl:         mode_str = "POSCTL"; break;
        case mavsdk::Telemetry::FlightMode::Acro:           mode_str = "ACRO"; break;
        case mavsdk::Telemetry::FlightMode::Stabilized:     mode_str = "STABILIZE"; break;
        default:                                             mode_str = "OTHER"; break;
    }

    snprintf(resp, resp_size, "TELEMETRY %.1f %.7f %.7f %d %s",
             pos.relative_altitude_m, pos.latitude_deg, pos.longitude_deg,
             armed ? 1 : 0, mode_str);
}

