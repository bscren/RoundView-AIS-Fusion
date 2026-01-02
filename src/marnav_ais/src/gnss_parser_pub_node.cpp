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
#include <fstream>

#include "marnav_interfaces/msg/gnss.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

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
      : Node("gnss_parser_node"), running_(false),
      gnss_yaw_bias_(90.0), gnss_pitch_bias_(0.0), gnss_roll_bias_(0.0) {

    // 获取配置文件路径参数
    declare_parameter<std::string>("config_file", "");
    std::string config_file_path = get_parameter("config_file").as_string();
    
    // 如果未指定配置文件，使用默认路径
    if (config_file_path.empty()) {
      try {
        std::string package_path = ament_index_cpp::get_package_share_directory("marnav_vis");
        config_file_path = package_path + "/config/track_realtime_config.yaml";
        RCLCPP_INFO(get_logger(), "未指定配置文件，使用默认路径: %s", config_file_path.c_str());
      } catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(), "查找默认配置文件失败: %s", e.what());
        rclcpp::shutdown();
        return;
      }
    }
    
    // 检查配置文件是否存在
    std::ifstream file_check(config_file_path);
    if (!file_check.good()) {
      RCLCPP_FATAL(get_logger(), "配置文件不存在: %s", config_file_path.c_str());
      rclcpp::shutdown();
      return;
    }
    file_check.close();
    
    // 从YAML文件加载配置
    std::string comm_type = "udp";
    std::string serial_port = "/dev/ttyS3";
    int baud_rate = 115200;
    std::string tcp_ip = "127.0.0.1";
    int tcp_port = 8010;
    std::string udp_ip = "0.0.0.0";
    int udp_port = 8010;
    double publish_hz = 5.0;
    std::string gnss_pub_topic = "gnss_data";


    try {
      YAML::Node config = YAML::LoadFile(config_file_path);
      
      if (!config["gnss"]) {
        RCLCPP_WARN(get_logger(), "YAML配置文件中未找到'gnss'节点，使用默认值");
      } else {
        const YAML::Node& gnss_config = config["gnss"];
        
        // 读取GNSS偏差
        if (gnss_config["gnss_yaw_bias"]) {
          gnss_yaw_bias_ = gnss_config["gnss_yaw_bias"].as<double>();
        }
        if (gnss_config["gnss_pitch_bias"]) {
          gnss_pitch_bias_ = gnss_config["gnss_pitch_bias"].as<double>();
        }
        if (gnss_config["gnss_roll_bias"]) {
          gnss_roll_bias_ = gnss_config["gnss_roll_bias"].as<double>();
        }

        // 读取通信类型
        if (gnss_config["comm_type"]) {
          comm_type = gnss_config["comm_type"].as<std::string>();
        }
        
        // 读取UDP参数
        if (gnss_config["udp_para"]) {
          const YAML::Node& udp_para = gnss_config["udp_para"];
          if (udp_para["udp_ip"]) udp_ip = udp_para["udp_ip"].as<std::string>();
          if (udp_para["udp_port"]) udp_port = udp_para["udp_port"].as<int>();
        }
        
        // 读取串口参数
        if (gnss_config["serial_port_para"]) {
          const YAML::Node& serial_para = gnss_config["serial_port_para"];
          if (serial_para["serial_port"]) serial_port = serial_para["serial_port"].as<std::string>();
          if (serial_para["baud_rate"]) baud_rate = serial_para["baud_rate"].as<int>();
        }
        
        // 读取TCP参数
        if (gnss_config["tcp_para"]) {
          const YAML::Node& tcp_para = gnss_config["tcp_para"];
          if (tcp_para["tcp_ip"]) tcp_ip = tcp_para["tcp_ip"].as<std::string>();
          if (tcp_para["tcp_port"]) tcp_port = tcp_para["tcp_port"].as<int>();
        }
        
        // 读取发布频率
        if (gnss_config["gnss_publish_rate"]) {
          publish_hz = gnss_config["gnss_publish_rate"].as<double>();
        }
        
        // 读取发布话题
        if (gnss_config["gnss_pub_topic"]) {
          gnss_pub_topic = gnss_config["gnss_pub_topic"].as<std::string>();
        }
        
        RCLCPP_INFO(get_logger(), "成功从YAML文件加载GNSS配置: %s", config_file_path.c_str());
      }
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(get_logger(), "解析YAML配置文件失败: %s，使用默认值", e.what());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "读取配置文件时发生错误: %s，使用默认值", e.what());
    }

    // 创建发布者（使用配置文件中的话题名）
    gnss_pub_ = create_publisher<marnav_interfaces::msg::Gnss>(gnss_pub_topic, 10);
    RCLCPP_INFO(get_logger(), "发布GNSS话题: %s", gnss_pub_topic.c_str());

    // 初始化通信接口（使用从YAML读取的参数）
    init_comm_interface(comm_type, serial_port, baud_rate, tcp_ip, tcp_port, udp_ip, udp_port);

    if (comm_interface_ && comm_interface_->connect()) {
      running_ = true;
      comm_thread_ = std::thread(&GNSSParserNode::comm_loop, this);
      RCLCPP_INFO(get_logger(), "GNSS 通信线程已启动");
    } else {
      RCLCPP_FATAL(get_logger(), "通信接口初始化失败，节点退出");
      rclcpp::shutdown();
      return;
    }

    // 设置发布频率
    if (publish_hz <= 0.0) {
      publish_hz = 5.0;
    }
    auto period = std::chrono::duration<double>(1.0 / publish_hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&GNSSParserNode::publish_timer, this));

    RCLCPP_INFO(get_logger(), "GNSS 解析节点启动，发布频率: %.2f Hz", publish_hz);
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
  // 初始化通信接口（使用传入的参数）
  void init_comm_interface(const std::string& comm_type,
                          const std::string& serial_port,
                          int baud_rate,
                          const std::string& tcp_ip,
                          int tcp_port,
                          const std::string& udp_ip,
                          int udp_port) {
    if (comm_type == "serial") {
      comm_interface_ = std::make_unique<SerialComm>(serial_port, baud_rate);
      RCLCPP_INFO(get_logger(), "选择串口通信: %s, 波特率: %d", serial_port.c_str(), baud_rate);
    } else if (comm_type == "tcp") {
      comm_interface_ = std::make_unique<TcpComm>(tcp_ip, tcp_port);
      RCLCPP_INFO(get_logger(), "选择 TCP 通信: %s:%d", tcp_ip.c_str(), tcp_port);
    } else {
      // 默认 UDP
      comm_interface_ = std::make_unique<UdpComm>(udp_ip, udp_port);
      RCLCPP_INFO(get_logger(), "选择 UDP 通信: %s:%d", udp_ip.c_str(), udp_port);
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
    // 添加GNSS偏差修正，yaw角（0~360度）需要考虑超过360度的情况，而pitch角（-90~90度）不需要
    if (ok_heading) {
      heading += gnss_yaw_bias_;
      if (heading > 360) {
        heading -= 360;
      } else if (heading < 0) {
        heading += 360;
      }
    }if (ok_pitch) {
      pitch += gnss_pitch_bias_;
    }

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
  // ========== 类的私有成员变量 ==========
  double gnss_yaw_bias_;    
  double gnss_pitch_bias_;
  double gnss_roll_bias_;   

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


