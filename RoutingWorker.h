#ifndef ROUTINGWORKER_H
#define ROUTINGWORKER_H

#include <QObject>
#include <QString>
#include <cmath>

//====================

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>
#include <QDebug>

//===========

//osgEarth libraries

#include <osg/Group>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgEarth/MapNode>
#include <osgEarth/Feature>
#include <osgEarth/FeatureNode>
#include <osgEarth/Geometry>
#include <osgEarth/Style>
#include <osgEarth/LineSymbol>
#include <osgEarth/AltitudeSymbol>
#include <osgEarth/ExtrusionSymbol>
#include <osgEarth/PolygonSymbol>
#include <osgEarth/PointSymbol>
#include <osgEarth/GeoData>
#include <osgEarth/GeoTransform>
#include <osgEarth/ModelNode>

#include <osgViewer/Viewer>
#include <osgEarth/EarthManipulator>

#include "3DMapView/3DModelSetup/ModelPlacementWidget.h"

//================================

static double euclid(double x1,double y1,double x2,double y2){
    return ::hypot(x1 - x2, y1 - y2);
}

class RoutingWorker : public QObject
{
    Q_OBJECT

public:
    enum AlgorithmType {
        Dijkstra = 0,
        AStar     = 1
    };

    RoutingWorker(const QString &shapefile,
                  double sx, double sy,
                  double ex, double ey,
                  AlgorithmType algo,
                  osgEarth::MapNode* mapNode = nullptr);
    RoutingWorker();

public slots:
    void process();

signals:
    void stageUpdate(const QString &msg);
    /// Emits the single output path (shp;csv) when done
    void routingComplete(const QString &outputPaths);
    void finished();

public:
    QString        inputPath;
    double         startX, startY, endX, endY;
    AlgorithmType  algoType;

    static double euclid(double x1,double y1,double x2,double y2) {
        return std::hypot(x1-x2, y1-y2);
    }
    struct PtKey { double x,y; bool operator==(PtKey const& o) const {
        return x==o.x && y==o.y;
    }};
    struct Edge { int to; double cost; };

public:

    struct NavMessage {
      // Header
      char tag[3];           // 'N','A','V'
      uint8_t version;       // 0x01
      uint16_t total_len;    // expect 256
      uint8_t message_type;  // 0x03
      uint8_t flags;

      // Core
      uint32_t sequence;
      uint32_t epoch_seconds;
      uint32_t epoch_ms_residual;

      // Endpoints (IP in network order; port in host order)
      uint32_t source_ipv4_nbo;
      uint16_t source_port;
      uint32_t dest_ipv4_nbo;
      uint16_t dest_port;

      // Timestamp
      uint16_t year;
      uint8_t month;
      uint8_t day;
      uint8_t hour;
      uint8_t minutes;
      uint8_t seconds;
      uint16_t milliseconds;

      // Attitude, rates, heave
      float roll_deg;
      float pitch_deg;
      float heading_deg;
      float roll_rate_dps;
      float pitch_rate_dps;
      float heading_rate_dps;
      float heave_m;

      // Vel NED
      float vel_n_mps;
      float vel_e_mps;
      float vel_d_mps;

      // Geodetic
      double lat_deg;
      double lon_deg;
      float alt_m;

      // Std devs
      uint16_t lat_std;
      uint16_t lon_std;
      uint16_t vel_n_std;
      uint16_t vel_e_std;
      uint16_t vel_d_std;
      uint16_t roll_std;
      uint16_t pitch_std;
      uint16_t heading_std;

      // Altimeter and env
      float altimeter_depth_m;
      float altimeter_temp_c;
      uint16_t running_depth_mm;
      uint16_t reserved1;

      // Mission control
      uint8_t mission_cmd;
      uint8_t reserved2;

      float x_m;
      float y_m;
      float z_m;
      float surge_mps;
      float sway_mps;
    };
    void Add_location();
    void Update_location(double lat_deg, double lon_deg, double alt_m);
    void Connect_UDP();
    osg::ref_ptr<osgEarth::MapNode> m_mapNode = nullptr;
    osg::ref_ptr<osg::Group> group = nullptr;
    osg::ref_ptr<osgEarth::LineString> _flightPath;
    osg::ref_ptr<osgEarth::FeatureNode> _flightPathNode;

    inline bool parse_nav(const uint8_t* buf, size_t len, NavMessage& out, std::string& err);
    static std::string ipv4_to_string(uint32_t ipv4_nbo);

};

//=================================================

//UDP - SERVER

class UdpSocket {
 public:
  UdpSocket() = default;
  ~UdpSocket() { close(); }

  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;

  UdpSocket(UdpSocket&& other) noexcept { *this = std::move(other); }
  UdpSocket& operator=(UdpSocket&& other) noexcept {
    if (this != &other) {
      close();
      fd_ = other.fd_;
      remote_ = other.remote_;
      has_remote_ = other.has_remote_;
      other.fd_ = -1;
      other.has_remote_ = false;
    }
    return *this;
  }

  bool try_bind(const std::string& ip, uint16_t port) {
    close();
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      perror("socket()");
      return false;
    }

    // Set socket to non-blocking mode
    int flags = fcntl(fd_, F_GETFL, 0);
    if (flags == -1) {
      perror("fcntl(F_GETFL) failed");
      return false;
    }

    if (fcntl(fd_, F_SETFL, flags | O_NONBLOCK) == -1) {
      perror("fcntl(F_SETFL) failed");
      return false;
    }

    set_cloexec();

    int one = 1;
    (void)::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
#if defined(SO_REUSEPORT)
    (void)::setsockopt(fd_, SOL_SOCKET, SO_REUSEPORT, &one, sizeof(one));
#endif

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    if (ip.empty() || ip == "*" || ip == "0.0.0.0") {
      addr.sin_addr.s_addr = INADDR_ANY;
    } else if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
      std::cerr << "[Error] Invalid local IP: " << ip << "\n";
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      perror("bind()");
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    return true;
  }

  bool try_connect_remote(const std::string& ip, uint16_t port) {
    if (port == 0) {
      std::cerr << "[Error] Remote port cannot be 0\n";
      return false;
    }

    if (fd_ < 0) {
      fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
      if (fd_ < 0) {
        perror("socket()");
        return false;
      }

      // Set socket to non-blocking mode
      int flags = fcntl(fd_, F_GETFL, 0);
      if (flags == -1) {
        perror("fcntl(F_GETFL) failed");
        return false;
      }

      if (fcntl(fd_, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("fcntl(F_SETFL) failed");
        return false;
      }

      set_cloexec();
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
      std::cerr << "[Error] Invalid remote IP: " << ip << "\n";
      return false;
    }

    if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      perror("connect()");
      return false;
    }

    remote_ = addr;
    has_remote_ = true;
    return true;
  }

  bool is_ready() const { return fd_ >= 0 && has_remote_; }

  void send(const std::vector<uint8_t>& buf) const {
    if (!is_ready()) {
      throw std::runtime_error(
          "Failed to send: socket not bound or remote not connected");
    }
    if (buf.size() > kMaxUdpPayload) {
      throw std::length_error("UDP payload too large (> 65507 bytes)");
    }

    for (;;) {
      ssize_t n = ::send(fd_, buf.data(), buf.size(), 0);
      if (n < 0) {
        if (errno == EINTR) continue;  // retry on signal
        throw_errno("send()");
      }
      if (static_cast<size_t>(n) != buf.size()) {
        // UDP should send the full datagram atomically or fail.
        throw std::runtime_error("send(): partial datagram sent");
      }
      break;
    }
  }

  void close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    has_remote_ = false;
    std::memset(&remote_, 0, sizeof(remote_));
  }

  std::pair<uint32_t, uint16_t> local_address() const {
    if (fd_ < 0) throw std::runtime_error("Socket not open");
    sockaddr_in addr{};
    socklen_t len = sizeof(addr);
    if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len) < 0) {
      throw_errno("getsockname()");
    }
    return {addr.sin_addr.s_addr, ntohs(addr.sin_port)};
  }

  std::pair<uint32_t, uint16_t> remote_address() const {
    if (!has_remote_) throw std::runtime_error("Remote not set");
    return {remote_.sin_addr.s_addr, ntohs(remote_.sin_port)};
  }

  ssize_t recv_from(void* buf, size_t len, uint32_t* src_ipv4 = nullptr,
                    uint16_t* src_port = nullptr, int flags = 0) const {

      qDebug("Came inside ot Recv_from()");

    qDebug() << "-----recv_from: fd_ =" << fd_;


    if (fd_ < 0) {
      errno = EBADF;
      return -1;
    }
    sockaddr_in from{};
    socklen_t fromlen = sizeof(from);

    for (;;) {
      ssize_t n = ::recvfrom(fd_, buf, len, flags,
                             reinterpret_cast<sockaddr*>(&from), &fromlen);
      if (n < 0) {
        if (errno == EINTR) continue;
        return -1;
      }
      if (src_ipv4) *src_ipv4 = from.sin_addr.s_addr;  // network byte order
      if (src_port) *src_port = ntohs(from.sin_port);  // host order
      return n;
    }
  }

 public:
  int fd_{-1};
  sockaddr_in remote_{};
  bool has_remote_{false};

  static constexpr size_t kMaxUdpPayload = 65507;  // typical max for IPv4

  static void throw_errno(const std::string& msg) {
    throw std::system_error(errno, std::generic_category(), msg);
  }

  void set_cloexec() const {
    if (fd_ < 0) return;
    int flags = ::fcntl(fd_, F_GETFD);
    if (flags >= 0) {
      (void)::fcntl(fd_, F_SETFD, flags | FD_CLOEXEC);
    }
  }

public:
  void send_to(const uint8_t* data, size_t len, const char* ip, uint16_t port) const {
    if (len > kMaxUdpPayload) {
      throw std::length_error("UDP payload too large (> 65507 bytes)");
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &addr.sin_addr) != 1) {
      throw std::invalid_argument("Invalid IP address");
    }

    for (;;) {
      ssize_t n = ::sendto(fd_, data, len, 0, (struct sockaddr*)&addr, sizeof(addr));
      if (n < 0) {
        if (errno == EINTR) continue;  // Retry on signal
        throw_errno("sendto()");
      }
      if (static_cast<size_t>(n) != len) {
        throw std::runtime_error("sendto(): partial datagram sent");
      }
      break;
    }
  }

};

//==========================
#pragma pack(push, 1)
struct Waypoint {
  uint16_t serial_no = 0;
  uint8_t mission_type = 0;
  uint8_t waypoint_tag = 0;
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  uint16_t running_depth_m = 0;
  uint8_t speed_mps = 0;

  static constexpr size_t expected_wire_size() {
    return 23;
  }  // 23 vs 22 discussed
};
#pragma pack(pop)

#pragma pack(push, 1)
struct Route {
  uint8_t version = 0x01;
  uint8_t message_type = 0x02;
  uint8_t flags = 0;
  uint32_t sequence = 0;
  uint32_t epoch_seconds = 0;
  uint32_t epoch_millis_residual = 0;
  uint32_t source_ipv4 = 0;
  uint16_t source_port = 0;
  uint32_t dest_ipv4 = 0;
  uint16_t dest_port = 0;
  uint16_t route_id = 0;

  uint8_t Fragment_index =0;
  uint8_t Fragment_count = 0;

  std::vector<Waypoint> waypoints;
};
#pragma pack(pop)


inline Waypoint make_waypoint(uint16_t serial, double latitude_deg,
                              double longitude_deg, uint16_t depth_m,
                              uint8_t speed_mps, uint8_t mission_type = 0,
                              uint8_t waypoint_tag = 0) {
  Waypoint wp;
  wp.serial_no = serial;
  wp.latitude_deg = latitude_deg;
  wp.longitude_deg = longitude_deg;
  wp.running_depth_m = depth_m;
  wp.speed_mps = speed_mps;
  wp.mission_type = mission_type;
  wp.waypoint_tag = waypoint_tag;
  return wp;
}

//====================================
// ----------------- Interface -----------------
template <typename T>
struct ISerializer {
  virtual ~ISerializer() = default;
  virtual std::vector<uint8_t> serialize(const T& msg) const = 0;
};

// ----------------- Utility -----------------
namespace rte_detail {
inline bool host_is_little_endian() {
  uint16_t x = 1;
  return *reinterpret_cast<uint8_t*>(&x) == 1;
}
inline uint64_t bswap64(uint64_t x) {
#if defined(__GNUC__) || defined(__clang__)
  return __builtin_bswap64(x);
#else
  return ((x & 0xFF00000000000000ull) >> 56) |
         ((x & 0x00FF000000000000ull) >> 40) |
         ((x & 0x0000FF0000000000ull) >> 24) |
         ((x & 0x000000FF00000000ull) >> 8) |
         ((x & 0x00000000FF000000ull) << 8) |
         ((x & 0x0000000000FF0000ull) << 24) |
         ((x & 0x000000000000FF00ull) << 40) |
         ((x & 0x00000000000000FFull) << 56);
#endif
}
//inline void write_be_double(std::vector<uint8_t>& out, double v) {
//  uint64_t bits;
//  std::memcpy(&bits, &v, sizeof(bits));
//  if (host_is_little_endian()) bits = bswap64(bits);
//  for (int i = 7; i >= 0; --i) out.push_back((bits >> (8 * i)) & 0xFF);
//}

inline void write_be_double(std::vector<uint8_t>& out, double v) {
uint64_t bits;
std::memcpy(&bits, &v, sizeof(bits));
for (int i = 7; i >= 0; --i) {
out.push_back(static_cast<uint8_t>((bits >> (8 * i)) & 0xFF));
}
}

inline void write_be_u8(std::vector<uint8_t>& out, uint8_t v) {
  out.push_back(v);
}
inline void write_be_u16(std::vector<uint8_t>& out, uint16_t v) {
  //uint16_t be = htons(v);
//  out.push_back(be >> 8);
//  out.push_back(be & 0xFF);
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(v & 0xFF));
}
inline void write_be_u32(std::vector<uint8_t>& out, uint32_t v) {
  //uint32_t be = htonl(v);
//  out.push_back((be >> 24) & 0xFF);
//  out.push_back((be >> 16) & 0xFF);
//  out.push_back((be >> 8) & 0xFF);
//  out.push_back(be & 0xFF);

    out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(v & 0xFF));
}
inline void write_be_u32_raw(std::vector<uint8_t>& out, uint32_t net_order) {
//  out.push_back((be >> 24) & 0xFF);
//  out.push_back((be >> 16) & 0xFF);
//  out.push_back((be >> 8) & 0xFF);
//  out.push_back(be & 0xFF);
    uint8_t b[4];
    std::memcpy(b, &net_order, 4); // bytes in memory are already network-order
    out.insert(out.end(), b, b + 4);
}
}  // namespace rte_detail

class RteSerializer : public ISerializer<Route>
{
public:
    std::vector<uint8_t> serialize(const Route& route) const
    {
        std::vector<uint8_t> out;
        out.reserve(64 + route.waypoints.size() * Waypoint::expected_wire_size());

        // Header
        out.push_back('R');
        out.push_back('T');
        out.push_back('E');
        rte_detail::write_be_u8(out, route.version);
        out.push_back(0);
        out.push_back(0);  // total length placeholder
        rte_detail::write_be_u8(out, route.message_type);
        rte_detail::write_be_u8(out, route.flags);
        rte_detail::write_be_u32(out, route.sequence);
        rte_detail::write_be_u32(out, route.epoch_seconds);
        rte_detail::write_be_u32(out, route.epoch_millis_residual);
        rte_detail::write_be_u32_raw(out, route.source_ipv4);
        rte_detail::write_be_u16(out, route.source_port);
        rte_detail::write_be_u32_raw(out, route.dest_ipv4);
        rte_detail::write_be_u16(out, route.dest_port);
        rte_detail::write_be_u16(out, route.route_id);
        rte_detail::write_be_u8(out, route.Fragment_index);
        rte_detail::write_be_u8(out, route.Fragment_count);
        //rte_detail::write_be_u16(out, 0);  // reserved
        rte_detail::write_be_u8(out, 0);  // reserved
        rte_detail::write_be_u8(out, 0);  // reserved
        rte_detail::write_be_u8(out, 0);  // reserved

        // Payload
//        if (route.waypoints.size() > 0xFF)
//            throw std::runtime_error("Too many waypoints");

        rte_detail::write_be_u16(out, static_cast<uint8_t>(route.waypoints.size()));

        for (const auto& wp : route.waypoints) {
            rte_detail::write_be_u16(out, wp.serial_no);
            rte_detail::write_be_u8(out, wp.mission_type);
            rte_detail::write_be_u8(out, wp.waypoint_tag);
            rte_detail::write_be_double(out, wp.latitude_deg);
            rte_detail::write_be_double(out, wp.longitude_deg);
            rte_detail::write_be_u16(out, wp.running_depth_m);
            rte_detail::write_be_u8(out, wp.speed_mps);
        }

        uint16_t total_len = static_cast<uint16_t>(out.size());
        out[4] = static_cast<uint8_t>((total_len >> 8) & 0xFF);
        out[5] = static_cast<uint8_t>(total_len & 0xFF);

        return out;
    }
};
//==========================================

template <typename T, typename Serializer>
class MessageSender {
 public:
  MessageSender(UdpSocket&& sock, Serializer serializer)
      : sock_(std::move(sock)), serializer_(std::move(serializer)) {}

  void send_message(const T& msg) {
    auto buf = serializer_.serialize(msg);

    qDebug("CAME INSIDE SEND MESSAGE");
    qDebug() << "+++++Serialized buffer size: " << buf.size();  // Should be 646
    sock_.send(buf);
  }

  UdpSocket& socket() { return sock_; }
  const UdpSocket& socket() const { return sock_; }

 private:
  UdpSocket sock_;
  Serializer serializer_;
};

//==========================================================================

//UDP CLIENT

inline uint16_t be16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] << 8 | p[1]);
}
inline uint32_t be32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
         (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}
inline uint64_t be64(const uint8_t* p) {
  return (uint64_t(p[0]) << 56) | (uint64_t(p[1]) << 48) |
         (uint64_t(p[2]) << 40) | (uint64_t(p[3]) << 32) |
         (uint64_t(p[4]) << 24) | (uint64_t(p[5]) << 16) |
         (uint64_t(p[6]) << 8) | uint64_t(p[7]);
}
inline float be_float(const uint8_t* p) {
  uint32_t u = be32(p);
  float f;
  std::memcpy(&f, &u, sizeof(f));
  return f;
}
inline double be_double(const uint8_t* p) {
  uint64_t u = be64(p);
  double d;
  std::memcpy(&d, &u, sizeof(d));
  return d;
}


//=========================================




#endif // ROUTINGWORKER_H
