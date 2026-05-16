#include "bridge.h"
#include "vla_handler.h"
#include "console_handler.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <unistd.h>

static void print_usage(const char *prog) {
    fprintf(stderr,
        "用法: %s [选项]\n"
        "  -s <URL>   串口 (默认: serial:///dev/ttyS3:115200)\n"
        "  -m <IP>    MP的IP (默认: 192.168.137.1)\n"
        "  -p <端口>  MP端口 (默认: 14555)\n"
        "  -v <端口>  VLA端口 (默认: 14556)\n"
        "  -l <级别>  日志 0=安静 1=正常 2=详细 (默认: 1)\n"
        "  -h         显示帮助\n", prog);
}

int main(int argc, char *argv[]) {
    BridgeConfig cfg;
    LogConfig log;
    int log_level = 1;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-s" && i + 1 < argc) cfg.serial_url = argv[++i];
        else if (arg == "-m" && i + 1 < argc) cfg.mp_ip = argv[++i];
        else if (arg == "-p" && i + 1 < argc) cfg.mp_port = atoi(argv[++i]);
        else if (arg == "-v" && i + 1 < argc) cfg.vla_port = atoi(argv[++i]);
        else if (arg == "-l" && i + 1 < argc) log_level = atoi(argv[++i]);
        else if (arg == "-h") { print_usage(argv[0]); return 0; }
    }

    // 日志级别
    switch (log_level) {
    case 0: // 安静
        log.heartbeat = false;
        log.position = false;
        log.armed = false;
        log.commands = false;
        log.raw_bytes = false;
        log.stats_interval = 0;
        break;
    case 2: // 详细
        log.heartbeat = true;
        log.raw_bytes = true;
        log.stats_interval = 5;
        break;
    default: // 正常 (1)
        break;
    }

    Bridge bridge;
    if (!bridge.init(cfg, log)) return 1;

    VlaHandler vla(bridge);
    if (!vla.start(cfg.vla_port)) return 1;

    ConsoleHandler console(bridge);
    console.start();

    bridge.run();  // 阻塞，直到 quit

    console.stop();
    vla.stop();
    bridge.shutdown();

    printf("\n桥接已关闭\n");
    return 0;
}
