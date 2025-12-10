// 该版本:
// 1. 通过串口 / TCP / UDP 接收 #AGRICA 报文（默认 UDP）
// 2. 解码最新的 GNSS 信息（lat/lon/heading/pitch/alt）
// 3. 使用定时器以可配置频率（默认 5Hz）发布 Gnss 消息

#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialPort.h>
#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <chrono>
#include <sstream>

#include "marnav_interfaces/msg/gnss.hpp"

using namespace std::chrono_literals;
using boost::asio::ip::tcp;
using boost::asio::ip::udp;
using namespace LibSerial;

// ---------------- 通信接口 ----------------
class CommInterface {
public:
  virtual ~CommInterface() = default;
  virtual bool connect() = 0;
  virtual void disconnect() = 0;
  virtual bool is_connected() = 0;
  virtual std::string read_data() = 0;
};

class SerialComm : public CommInterface {
public:
  SerialComm(const std::string &port, unsigned int baud_rate)
      : port_(port), baud_rate_(baud_rate) {}

  bool connect() override {
    try {
      serial_port_.Open(port_);
      if (!serial_port_.IsOpen()) {
        return false;
      }
      BaudRate baud = convert_baud_rate(baud_rate_);
      if (baud == BaudRate::BAUD_INVALID) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "不支持的波特率: %u", baud_rate_);
        serial_port_.Close();
        return false;
      }
      serial_port_.SetBaudRate(baud);
      serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
      serial_port_.SetParity(Parity::PARITY_NONE);
      serial_port_.SetStopBits(StopBits::STOP_BITS_1);
      serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "连接失败: %s", e.what());
      return false;
    }
  }

  void disconnect() override {
    if (serial_port_.IsOpen()) {
      try {
        serial_port_.Close();
      } catch (const std::exception &e) {
        RCLCPP_WARN(rclcpp::get_logger("SerialComm"), "关闭失败: %s", e.what());
      }
    }
  }

  bool is_connected() override { return serial_port_.IsOpen(); }

  std::string read_data() override {
    try {
      std::string data;
      if (serial_port_.IsDataAvailable()) {
        serial_port_.ReadLine(data, '\n', 1000);
      }
      return data;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "读取错误: %s", e.what());
      return "";
    }
  }

private:
  BaudRate convert_baud_rate(unsigned int baud_rate) {
    switch (baud_rate) {
    case 1200:
      return BaudRate::BAUD_1200;
    case 2400:
      return BaudRate::BAUD_2400;
    case 4800:
      return BaudRate::BAUD_4800;
    case 9600:
      return BaudRate::BAUD_9600;
    case 19200:
      return BaudRate::BAUD_19200;
    case 38400:
      return BaudRate::BAUD_38400;
    case 57600:
      return BaudRate::BAUD_57600;
    case 115200:
      return BaudRate::BAUD_115200;
    default:
      return BaudRate::BAUD_INVALID;
    }
  }

  std::string port_;
  unsigned int baud_rate_;
  SerialPort serial_port_;
};

class TcpComm : public CommInterface {
public:
  TcpComm(const std::string &ip, unsigned short port)
      : ip_(ip), port_(port), io_context_(), socket_(io_context_) {}

  bool connect() override {
    try {
      tcp::resolver resolver(io_context_);
      auto endpoints = resolver.resolve(ip_, std::to_string(port_));
      boost::asio::connect(socket_, endpoints);
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("TcpComm"), "连接失败: %s", e.what());
      return false;
    }
  }

  void disconnect() override {
    try {
      socket_.close();
    } catch (const std::exception &e) {
      RCLCPP_WARN(rclcpp::get_logger("TcpComm"), "关闭失败: %s", e.what());
    }
  }

  bool is_connected() override { return socket_.is_open(); }

  std::string read_data() override {
    try {
      char buffer[2048];
      size_t len = socket_.read_some(boost::asio::buffer(buffer));
      return std::string(buffer, len);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("TcpComm"), "读取错误: %s", e.what());
      return "";
    }
  }

private:
  std::string ip_;
  unsigned short port_;
  boost::asio::io_context io_context_;
  tcp::socket socket_;
};

class UdpComm : public CommInterface {
public:
  UdpComm(const std::string &ip, unsigned short port)
      : ip_(ip), port_(port), io_context_(),
        socket_(io_context_, udp::endpoint(udp::v4(), port)) {}

  bool connect() override {
    try {
      return socket_.is_open();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("UdpComm"), "初始化失败: %s", e.what());
      return false;
    }
  }

  void disconnect() override {
    try {
      socket_.close();
    } catch (const std::exception &e) {
      RCLCPP_WARN(rclcpp::get_logger("UdpComm"), "关闭失败: %s", e.what());
    }
  }

  bool is_connected() override { return socket_.is_open(); }

  std::string read_data() override {
    try {
      char buffer[2048];
      udp::endpoint sender_endpoint;
      size_t len = socket_.receive_from(boost::asio::buffer(buffer), sender_endpoint);
      return std::string(buffer, len);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("UdpComm"), "读取错误: %s", e.what());
      return "";
    }
  }

private:
  std::string ip_;
  unsigned short port_;
  boost::asio::io_context io_context_;
  udp::socket socket_;
};

// ---------------- GNSS 解析节点 ----------------
class GNSSParserNode : public rclcpp::Node {
public:
  GNSSParserNode()
      : Node("gnss_parser_node"), running_(false) {

    declare_parameter<std::string>("comm_type", "udp");
    declare_parameter<std::string>("serial_port", "/dev/ttyS3");
    declare_parameter<int>("baud_rate", 115200);
    declare_parameter<std::string>("tcp_ip", "127.0.0.1");
    declare_parameter<int>("tcp_port", 8010);
    declare_parameter<std::string>("udp_ip", "0.0.0.0");
    declare_parameter<int>("udp_port", 8010);
    declare_parameter<double>("publish_hz", 5.0); // 默认 5Hz

    gnss_pub_ = create_publisher<marnav_interfaces::msg::Gnss>("gnss_data", 10);

    init_comm_interface();

    if (comm_interface_ && comm_interface_->connect()) {
      running_ = true;
      comm_thread_ = std::thread(&GNSSParserNode::comm_loop, this);
      RCLCPP_INFO(get_logger(), "GNSS 通信线程已启动");
    } else {
      RCLCPP_FATAL(get_logger(), "通信接口初始化失败，节点退出");
      rclcpp::shutdown();
      return;
    }

    double hz = get_parameter("publish_hz").as_double();
    if (hz <= 0.0) {
      hz = 5.0;
    }
    auto period = std::chrono::duration<double>(1.0 / hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&GNSSParserNode::publish_timer, this));

    RCLCPP_INFO(get_logger(), "GNSS 解析节点启动，发布频率: %.2f Hz", hz);
  }

  ~GNSSParserNode() override {
    running_ = false;
    if (comm_thread_.joinable()) {
      comm_thread_.join();
    }
    if (comm_interface_) {
      comm_interface_->disconnect();
    }
    RCLCPP_INFO(get_logger(), "GNSS 解析节点已关闭");
  }

private:
  // 初始化通信接口
  void init_comm_interface() {
    std::string comm_type = get_parameter("comm_type").as_string();
    if (comm_type == "serial") {
      auto port = get_parameter("serial_port").as_string();
      int baud = get_parameter("baud_rate").as_int();
      comm_interface_ = std::make_unique<SerialComm>(port, baud);
      RCLCPP_INFO(get_logger(), "选择串口通信: %s, 波特率: %d", port.c_str(), baud);
    } else if (comm_type == "tcp") {
      auto ip = get_parameter("tcp_ip").as_string();
      int port = get_parameter("tcp_port").as_int();
      comm_interface_ = std::make_unique<TcpComm>(ip, port);
      RCLCPP_INFO(get_logger(), "选择 TCP 通信: %s:%d", ip.c_str(), port);
    } else {
      // 默认 UDP
      auto ip = get_parameter("udp_ip").as_string();
      int port = get_parameter("udp_port").as_int();
      comm_interface_ = std::make_unique<UdpComm>(ip, port);
      RCLCPP_INFO(get_logger(), "选择 UDP 通信: %s:%d", ip.c_str(), port);
    }
  }

  // 通信循环
  void comm_loop() {
    while (running_ && rclcpp::ok()) {
      if (!comm_interface_->is_connected()) {
        RCLCPP_WARN(get_logger(), "通信未连接，尝试重连...");
        if (!comm_interface_->connect()) {
          std::this_thread::sleep_for(1s);
          continue;
        }
        RCLCPP_INFO(get_logger(), "通信已重连");
      }

      std::string data = comm_interface_->read_data();
      if (!data.empty()) {
        parse_and_store(data);
      }

      std::this_thread::sleep_for(5ms);
    }
  }

  // 解析并存储最新 GNSS
  void parse_and_store(const std::string &raw) {
    // 捕获第一个 #AGRICA...*
    size_t start = raw.find("#AGRICA");
    size_t star = raw.find('*', start == std::string::npos ? 0 : start);
    if (start == std::string::npos || star == std::string::npos) {
      RCLCPP_DEBUG(get_logger(), "未找到 #AGRICA 报文");
      return;
    }
    std::string msg = raw.substr(start, star - start); // 不含 '*'

    // 按 ';' 分离头和数据
    auto sep_pos = msg.find(';');
    if (sep_pos == std::string::npos) {
      RCLCPP_DEBUG(get_logger(), "AGRICA 格式错误，缺少 ';'");
      return;
    }
    std::string data_part = msg.substr(sep_pos + 1);

    // 去掉校验和
    auto star2 = data_part.find('*');
    if (star2 != std::string::npos) {
      data_part = data_part.substr(0, star2);
    }

    std::vector<std::string> fields;
    {
      std::stringstream ss(data_part);
      std::string item;
      while (std::getline(ss, item, ',')) {
        fields.emplace_back(item);
      }
    }

    if (fields.size() < 32) {
      RCLCPP_DEBUG(get_logger(), "AGRICA 字段数不足: %zu", fields.size());
      return;
    }

    auto parse_double = [](const std::string &s, double &out) -> bool {
      try {
        if (s.empty())
          return false;
        out = std::stod(s);
        return true;
      } catch (...) {
        return false;
      }
    };

    double lat = 0.0, lon = 0.0, alt = 0.0;
    double heading = 0.0, pitch = 0.0;
    bool ok_lat = parse_double(fields[29], lat);
    bool ok_lon = parse_double(fields[30], lon);
    bool ok_alt = parse_double(fields[31], alt);
    bool ok_heading = parse_double(fields[19], heading);
    bool ok_pitch = parse_double(fields[20], pitch);

    if (!ok_lat || !ok_lon) {
      RCLCPP_DEBUG(get_logger(), "AGRICA 位置字段无效");
      return;
    }

    marnav_interfaces::msg::Gnss gnss_msg;
    gnss_msg.latitude = lat;
    gnss_msg.longitude = lon;
    gnss_msg.horizontal_orientation = ok_heading ? heading : 0.0;
    gnss_msg.vertical_orientation = ok_pitch ? pitch : 0.0;
    gnss_msg.camera_height = ok_alt ? alt : 0.0;
    gnss_msg.timestamp = this->get_clock()->now();

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_gnss_ = gnss_msg;
    }
  }

  // 定时发布
  void publish_timer() {
    marnav_interfaces::msg::Gnss msg;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!last_gnss_.has_value()) {
        return;
      }
      msg = *last_gnss_;
      msg.timestamp = this->get_clock()->now();
    }
    gnss_pub_->publish(msg);
  }

private:
  std::unique_ptr<CommInterface> comm_interface_;
  std::thread comm_thread_;
  std::atomic<bool> running_;

  rclcpp::Publisher<marnav_interfaces::msg::Gnss>::SharedPtr gnss_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<marnav_interfaces::msg::Gnss> last_gnss_;
  std::mutex data_mutex_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GNSSParserNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


