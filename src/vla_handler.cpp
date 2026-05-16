#include "vla_handler.h"
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

VlaHandler::VlaHandler(Bridge &bridge) : m_bridge(bridge) {}
VlaHandler::~VlaHandler() { stop(); }

bool VlaHandler::start(int port) {
    m_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sock < 0) { perror("VLA socket"); return false; }

    int reuse = 1;
    setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(m_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("VLA bind");
        close(m_sock); m_sock = -1;
        return false;
    }

    struct timeval tv = {1, 0};
    setsockopt(m_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    printf("[VLA]  监听 UDP %d\n", port);

    m_running = true;
    m_thread = std::thread(&VlaHandler::thread_func, this);
    return true;
}

void VlaHandler::stop() {
    m_running = false;
    if (m_thread.joinable()) m_thread.join();
    if (m_sock >= 0) { close(m_sock); m_sock = -1; }
}

static bool is_heartbeat(const char *cmd) {
    return strcmp(cmd, "HEARTBEAT") == 0 || strcmp(cmd, "STATUS") == 0;
}

void VlaHandler::thread_func() {
    char buf[512];
    char resp[512];
    struct sockaddr_in from;
    socklen_t fromlen;

    while (m_running && m_bridge.running()) {
        fromlen = sizeof(from);
        int n = recvfrom(m_sock, buf, sizeof(buf) - 1, 0,
                         (struct sockaddr *)&from, &fromlen);
        if (n <= 0) continue;

        buf[n] = '\0';

        bool hb = is_heartbeat(buf);
        if (!hb) printf("[VLA] 收到: %s\n", buf);

        m_bridge.handle_vla_cmd(buf, resp, sizeof(resp));

        sendto(m_sock, resp, strlen(resp), 0,
               (struct sockaddr *)&from, fromlen);

        if (!hb) printf("[VLA] 回复: %s\n", resp);
    }
}
