// sonar_server_udp.cpp
// Compile: g++ -std=c++17 sonar_server_udp.cpp -O2 -o sonar_server_udp
// Usage (unicast): ./sonar_server_udp <dest_ip> <dest_port> [interval_ms]
// Usage (broadcast): ./sonar_server_udp --broadcast <port> [interval_ms]

//./sonar_server_udp 127.0.0.1 9000 200

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// Preserve IEEE-754 bits of float and convert to network order
static uint32_t float_to_net32(float f) {
    uint32_t bits;
    static_assert(sizeof(bits) == sizeof(f));
    std::memcpy(&bits, &f, sizeof(f));
    return htonl(bits);
}

static void append_u32(std::vector<uint8_t>& buf, uint32_t v) {
    uint32_t n = htonl(v);
    uint8_t *p = reinterpret_cast<uint8_t*>(&n);
    buf.insert(buf.end(), p, p + 4);
}

static void append_u16(std::vector<uint8_t>& buf, uint16_t v) {
    uint16_t n = htons(v);
    uint8_t *p = reinterpret_cast<uint8_t*>(&n);
    buf.insert(buf.end(), p, p + 2);
}

static void append_float(std::vector<uint8_t>& buf, float f) {
    uint32_t n = float_to_net32(f);
    uint8_t *p = reinterpret_cast<uint8_t*>(&n);
    buf.insert(buf.end(), p, p + 4);
}

// ---- Data structures & random generation ----
#pragma pack(push, 1)
struct SingleObstacle {
    uint32_t id;
    float velocity_mps;
    float heading_rad;
    float range_m;
    float bearing_rad;
    float x_m;
    float y_m;
    float range_extent_m;
    float bearing_extent_rad;
    float peak_intensity;
    float confidence;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct Geometry {
    float range_max_m;
    float fov_rad;
    uint32_t beam_count;
    uint32_t range_bins;
    uint32_t data_points_mode;
    std::vector<SingleObstacle> obstacles;
};
#pragma pack(pop)

Geometry make_random_geometry(std::mt19937 &rng, int max_obstacles = 2) {
    std::uniform_real_distribution<float> dist_range(0.5f, 120.0f);
    std::uniform_real_distribution<float> dist_small(0.0f, 3.0f);
    std::uniform_real_distribution<float> ang_dist(-1.0472f, 1.0472f); // +-60deg
    std::uniform_real_distribution<float> vel_dist(-20.0f, 20.0f);
    std::uniform_real_distribution<float> conf_dist(0.0f, 1.0f);
    std::uniform_int_distribution<int> obs_count_dist(1, max_obstacles);

    Geometry g;
    g.range_max_m = dist_range(rng);
    // fov_rad here choose between small positive number and wide: use absolute
    g.fov_rad = fabs(ang_dist(rng)) * 2.0f; // total FOV
    g.beam_count = 360;
    g.range_bins = 100;
    g.data_points_mode = 0;

    int n = obs_count_dist(rng);
    g.obstacles.clear();
    g.obstacles.reserve(n);

    std::uniform_real_distribution<float> bearing_extent_dist(0.01f, 0.15f);
    std::uniform_int_distribution<uint32_t> id_dist(1, 10000);
    std::uniform_real_distribution<float> peak_dist(0.0f, 255.0f);

    for (int i = 0; i < n; ++i) {
        SingleObstacle o;
        o.id = id_dist(rng);
        o.velocity_mps = vel_dist(rng);
        o.heading_rad = ang_dist(rng);
        o.range_m = dist_range(rng);
        o.bearing_rad = ang_dist(rng);
        o.x_m = o.range_m * std::cos(o.bearing_rad);
        o.y_m = o.range_m * std::sin(o.bearing_rad);
        o.range_extent_m = dist_small(rng);
        o.bearing_extent_rad = bearing_extent_dist(rng);
        o.peak_intensity = peak_dist(rng);
        o.confidence = conf_dist(rng);
        g.obstacles.push_back(o);
    }
    return g;
}

// Serialize geometry to a byte buffer (network byte order)
std::vector<uint8_t> serialize_geometry(const Geometry &g) {
    std::vector<uint8_t> buf;
    buf.reserve(1024);
    append_float(buf, g.range_max_m);      // 4
    append_float(buf, g.fov_rad);          // 4
    append_u32(buf, g.beam_count);         // 4
    append_u32(buf, g.range_bins);         // 4
    append_u32(buf, g.data_points_mode);   // 4

    uint16_t num_obs = static_cast<uint16_t>(g.obstacles.size());
    append_u16(buf, num_obs);              // 2

    // append obstacles
    for (const auto &o : g.obstacles) {
        append_u32(buf, o.id);
        append_float(buf, o.velocity_mps);
        append_float(buf, o.heading_rad);
        append_float(buf, o.range_m);
        append_float(buf, o.bearing_rad);
        append_float(buf, o.x_m);
        append_float(buf, o.y_m);
        append_float(buf, o.range_extent_m);
        append_float(buf, o.bearing_extent_rad);
        append_float(buf, o.peak_intensity);
        append_float(buf, o.confidence);
    }
    return buf;
}

int run_udp_sender(const std::string &dest_ip, uint16_t dest_port, bool broadcast, int interval_ms) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    if (broadcast) {
        int b = 1;
        if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &b, sizeof(b)) < 0) {
            perror("setsockopt(SO_BROADCAST)");
        }
    }

    sockaddr_in dst;
    std::memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons(dest_port);

    if (broadcast) {
        dst.sin_addr.s_addr = INADDR_BROADCAST;
    } else {
        if (inet_pton(AF_INET, dest_ip.c_str(), &dst.sin_addr) != 1) {
            std::cerr << "Invalid dest IP: " << dest_ip << std::endl;
            close(sock);
            return 2;
        }
    }

    std::mt19937 rng((unsigned)std::chrono::high_resolution_clock::now().time_since_epoch().count());

    std::cout << "UDP sender: sending to " << (broadcast ? "BROADCAST" : dest_ip)
              << ":" << dest_port << " every " << interval_ms << " ms\n";

    while (true) {
        Geometry g = make_random_geometry(rng, 10);
        auto buf = serialize_geometry(g);

        ssize_t sent = sendto(sock, buf.data(), buf.size(), 0, (sockaddr*)&dst, sizeof(dst));
        if (sent < 0) {
            perror("sendto");
        } else {
            std::cout << "\n=== Sent datagram ===\n";
            std::cout << "Bytes: " << sent << ", Obstacles: " << g.obstacles.size() << "\n";

            // NEW: print obstacle summary
            for (size_t i = 0; i < g.obstacles.size(); ++i) {
                const auto &o = g.obstacles[i];
                std::cout << "Obstacle[" << i << "]: "
                          << "id=" << o.id
                          << ", vel=" << o.velocity_mps
                          << ", heading=" << o.heading_rad
                          << ", range=" << o.range_m
                          << ", bearing=" << o.bearing_rad
                          << ", peak=" << o.peak_intensity
                          << ", conf=" << o.confidence << "\n";
            }

            // NEW: print first few raw bytes in hex
//            std::cout << "Raw data (first 64 bytes): ";
//            for (size_t i = 0; i < std::min<size_t>(64, buf.size()); ++i)
//                printf("%02X ", buf[i]);
            std::cout << "\n======================\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }

    close(sock);
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage (unicast): " << argv[0] << " <dest_ip> <dest_port> [interval_ms]\n";
        std::cerr << "Usage (broadcast): " << argv[0] << " --broadcast <port> [interval_ms]\n";
        return 1;
    }

    bool broadcast = false;
    std::string dest_ip;
    uint16_t port = 0;
    int interval_ms = 200;

    if (std::string(argv[1]) == "--broadcast") {
        broadcast = true;
        port = static_cast<uint16_t>(std::stoi(argv[2]));
        if (argc >= 4) interval_ms = std::stoi(argv[3]);
    } else {
        dest_ip = argv[1];
        port = static_cast<uint16_t>(std::stoi(argv[2]));
        if (argc >= 4) interval_ms = std::stoi(argv[3]);
    }

    return run_udp_sender(dest_ip, port, broadcast, interval_ms);
}

