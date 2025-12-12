// 该版本:
// 1. 通过串口/TCP/UDP接收AIVDM报文
// 2. 支持多片段AIVDM消息组合
// 3. 每秒发布一次AisBatch消息（包含该秒内所有解析的AIS数据）

#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialPort.h>
#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include <deque>
#include <marnav/nmea/nmea.hpp>
#include <marnav/nmea/vdm.hpp>
#include <marnav/nmea/sentence.hpp>
#include <marnav/nmea/ais_helper.hpp>
#include <marnav/ais/ais.hpp>
#include <marnav/ais/message_01.hpp>
#include <marnav/ais/message_02.hpp>
#include <marnav/ais/message_03.hpp>
#include <fstream>
#include <chrono>
#include <filesystem>
#include <sstream>
#include <ctime>
#include <iomanip>
#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/string.hpp"
#include "marnav_interfaces/msg/ais.hpp"
#include "marnav_interfaces/msg/ais_batch.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
using namespace LibSerial;
using boost::asio::ip::tcp;
using boost::asio::ip::udp;
using namespace marnav;
namespace fs = std::filesystem;

/**
 * @brief 通信接口抽象基类
 */
class CommInterface {
public:
    virtual ~CommInterface() = default;
    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool is_connected() = 0;
    virtual std::string read_data() = 0;
};

/**
 * @brief 串口通信实现
 */
class SerialComm : public CommInterface {
public:
    SerialComm(const std::string& port, unsigned int baud_rate)
        : port_(port), baud_rate_(baud_rate), serial_port_() {}

    bool connect() override {
        try {
            serial_port_.Open(port_);
            if (!serial_port_.IsOpen()) {
                return false;
            }
            
            BaudRate baud = convert_baud_rate(baud_rate_);
            if (baud == BaudRate::BAUD_INVALID) {
                RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "不支持的波特率: %d", baud_rate_);
                serial_port_.Close();
                return false;
            }
            
            serial_port_.SetBaudRate(baud);
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port_.SetParity(Parity::PARITY_NONE);
            serial_port_.SetStopBits(StopBits::STOP_BITS_1);
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "连接失败: %s", e.what());
            return false;
        }
    }

    void disconnect() override {
        if (serial_port_.IsOpen()) {
            try {
                serial_port_.Close();
            } catch (const std::exception& e) {
                RCLCPP_WARN(rclcpp::get_logger("SerialComm"), "关闭失败: %s", e.what());
            }
        }
    }

    bool is_connected() override {
        return serial_port_.IsOpen();
    }

    std::string read_data() override {
        try {
            std::string data;
            if (serial_port_.IsDataAvailable()) {
                serial_port_.ReadLine(data, '\n', 1000);
            }
            return data;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "读取错误: %s", e.what());
            return "";
        }
    }

private:
    BaudRate convert_baud_rate(unsigned int baud_rate) {
        switch (baud_rate) {
            case 1200:    return BaudRate::BAUD_1200;
            case 2400:    return BaudRate::BAUD_2400;
            case 4800:    return BaudRate::BAUD_4800;
            case 9600:    return BaudRate::BAUD_9600;
            case 19200:   return BaudRate::BAUD_19200;
            case 38400:   return BaudRate::BAUD_38400;
            case 57600:   return BaudRate::BAUD_57600;
            case 115200:  return BaudRate::BAUD_115200;
            default:      return BaudRate::BAUD_INVALID;
        }
    }

    std::string port_;
    unsigned int baud_rate_;
    SerialPort serial_port_;
};

/**
 * @brief TCP通信实现
 */
class TcpComm : public CommInterface {
public:
    TcpComm(const std::string& ip, unsigned short port)
        : ip_(ip), port_(port), io_context_(), socket_(io_context_) {}

    bool connect() override {
        try {
            tcp::resolver resolver(io_context_);
            auto endpoints = resolver.resolve(ip_, std::to_string(port_));
            boost::asio::connect(socket_, endpoints);
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("TcpComm"), "连接失败: %s", e.what());
            return false;
        }
    }

    void disconnect() override {
        try {
            socket_.close();
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("TcpComm"), "关闭失败: %s", e.what());
        }
    }

    bool is_connected() override {
        return socket_.is_open();
    }

    std::string read_data() override {
        try {
            char buffer[1024];
            size_t len = socket_.read_some(boost::asio::buffer(buffer));
            return std::string(buffer, len);
        } catch (const std::exception& e) {
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

/**
 * @brief UDP通信实现
 */
class UdpComm : public CommInterface {
public:
    UdpComm(const std::string& ip, unsigned short port)
        : ip_(ip), port_(port), io_context_(), socket_(io_context_, udp::endpoint(udp::v4(), port)) {}

    bool connect() override {
        try {
            return socket_.is_open();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("UdpComm"), "初始化失败: %s", e.what());
            return false;
        }
    }

    void disconnect() override {
        try {
            socket_.close();
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("UdpComm"), "关闭失败: %s", e.what());
        }
    }

    bool is_connected() override {
        return socket_.is_open();
    }

    std::string read_data() override {
        try {
            char buffer[1024];
            udp::endpoint sender_endpoint;
            size_t len = socket_.receive_from(
                boost::asio::buffer(buffer), sender_endpoint);
            return std::string(buffer, len);
        } catch (const std::exception& e) {
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

/**
 * @brief AIS批量发布节点
 * 
 * 功能：
 * 1. 通过串口/TCP/UDP接收AIVDM报文
 * 2. 支持多片段AIVDM消息组合
 * 3. 每秒发布一次AisBatch消息（包含该秒内所有解析的AIS数据）
 */
class AISBatchParserNode : public rclcpp::Node {
public:
    AISBatchParserNode() 
        : Node("ais_batch_parser"), 
          comm_interface_(nullptr), 
          running_(false) {
        
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
        
        // 从YAML文件加载配置（默认值）
        std::string comm_type = "udp";
        std::string serial_port = "/dev/ttyS3";
        int baud_rate = 38400;
        std::string tcp_ip = "127.0.0.1";
        int tcp_port = 5000;
        std::string udp_ip = "0.0.0.0";
        int udp_port = 1800;
        ais_batch_pub_topic_ = "/ais_batch_topic_realtime";
        
        try {
            YAML::Node config = YAML::LoadFile(config_file_path);
            
            if (!config["ais"]) {
                RCLCPP_WARN(get_logger(), "YAML配置文件中未找到'ais'节点，使用默认值");
            } else {
                const YAML::Node& ais_config = config["ais"];
                
                // 读取通信类型
                if (ais_config["comm_type"]) {
                    comm_type = ais_config["comm_type"].as<std::string>();
                }
                
                // 读取UDP参数
                if (ais_config["udp_para"]) {
                    const YAML::Node& udp_para = ais_config["udp_para"];
                    if (udp_para["udp_ip"]) udp_ip = udp_para["udp_ip"].as<std::string>();
                    if (udp_para["udp_port"]) udp_port = udp_para["udp_port"].as<int>();
                }
                
                // 读取串口参数
                if (ais_config["serial_port_para"]) {
                    const YAML::Node& serial_para = ais_config["serial_port_para"];
                    if (serial_para["serial_port"]) serial_port = serial_para["serial_port"].as<std::string>();
                    if (serial_para["baud_rate"]) baud_rate = serial_para["baud_rate"].as<int>();
                }
                
                // 读取TCP参数
                if (ais_config["tcp_para"]) {
                    const YAML::Node& tcp_para = ais_config["tcp_para"];
                    if (tcp_para["tcp_ip"]) tcp_ip = tcp_para["tcp_ip"].as<std::string>();
                    if (tcp_para["tcp_port"]) tcp_port = tcp_para["tcp_port"].as<int>();
                }
                
                // 读取发布话题
                if (ais_config["ais_batch_pub_topic"]) {
                    ais_batch_pub_topic_ = ais_config["ais_batch_pub_topic"].as<std::string>();
                }
                
                RCLCPP_INFO(get_logger(), "成功从YAML文件加载AIS配置: %s", config_file_path.c_str());
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(get_logger(), "解析YAML配置文件失败: %s，使用默认值", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "读取配置文件时发生错误: %s，使用默认值", e.what());
        }

        // 初始化AIS批量消息发布者
        ais_batch_publisher_ = create_publisher<marnav_interfaces::msg::AisBatch>(
            ais_batch_pub_topic_, 10);
        RCLCPP_INFO(get_logger(), "发布AIS批量话题: %s", ais_batch_pub_topic_.c_str());

        // 初始化通信接口（使用从YAML读取的参数）
        init_comm_interface(comm_type, serial_port, baud_rate, tcp_ip, tcp_port, udp_ip, udp_port);

        // 启动通信线程
        if (comm_interface_ && comm_interface_->connect()) {
            running_ = true;
            comm_thread_ = std::thread(&AISBatchParserNode::comm_loop, this);
            RCLCPP_INFO(get_logger(), "通信节点启动成功");
        } else {
            RCLCPP_FATAL(get_logger(), "通信接口初始化失败，节点启动失败");
            rclcpp::shutdown();
            return;
        }

        // 启动1Hz定时器用于批量发布
        batch_timer_ = create_wall_timer(
            1s,
            std::bind(&AISBatchParserNode::publish_batch_callback, this));
        
        RCLCPP_INFO(get_logger(), "AIS批量发布节点启动完成（1Hz发布频率）");
    }

    ~AISBatchParserNode() {
        running_ = false;
        if (comm_thread_.joinable()) {
            comm_thread_.join();
        }
        if (comm_interface_) {
            comm_interface_->disconnect();
        }
        RCLCPP_INFO(get_logger(), "AIS批量发布节点已关闭");
    }

private:
    /**
     * @brief 去除字符串首尾空白
     */
    std::string trim_nmea(const std::string &s) {
        auto start = s.begin();
        while (start != s.end() && std::isspace(*start)) {
            start++;
        }

        auto end = s.end();
        do {
            end--;
        } while (std::distance(start, end) > 0 && std::isspace(*end));

        return std::string(start, end + 1);
    }


    /**
     * @brief 解析AIVDM报文
     */
    void parse_aivdm(const std::string &data) {
        std::string trimmed_data = trim_nmea(data);
        RCLCPP_DEBUG(get_logger(), "接收到数据: %s", trimmed_data.c_str());
        
        // 过滤调试信息
        if (trimmed_data.substr(0, 6) == "$SYDBG") {
            RCLCPP_DEBUG(get_logger(), "忽略调试信息");
            return;
        }

        try {
            auto sentence = nmea::make_sentence(trimmed_data);
            
            // 仅处理VDM（AIVDM）句子
            if (sentence->id() == nmea::sentence_id::VDM) {
                const auto vdm = nmea::sentence_cast<marnav::nmea::vdm>(sentence.get());
                if (!vdm) {
                    RCLCPP_ERROR(get_logger(), "无效的VDM句子");
                    return;
                }

                const auto n_fragments = vdm->get_n_fragments();
                const auto fragment = vdm->get_fragment();
                RCLCPP_DEBUG(get_logger(), "收到VDM句子: 片段 %d/%d", fragment, n_fragments);

                std::lock_guard<std::mutex> lock(vdm_mutex_);
                
                // 验证片段编号
                if (fragment < 1 || fragment > n_fragments) {
                    RCLCPP_WARN(get_logger(), "无效片段编号: %d/%d，清空缓存", 
                                fragment, n_fragments);
                    vdm_fragments_.clear();
                    return;
                }
                
                // 新消息第一片，清空旧缓存
                if (fragment == 1) {
                    vdm_fragments_.clear();
                }
                
                vdm_fragments_.push_back(std::move(sentence));

                // 片段收集完成
                if (fragment == n_fragments) {
                    RCLCPP_DEBUG(get_logger(), "所有VDM片段已收集完成，开始解析");
                    
                    auto payload = nmea::collect_payload(
                        vdm_fragments_.begin(), vdm_fragments_.end());
                    vdm_fragments_.clear();
                    
                    try {
                        auto message = ais::make_message(payload);
                        if (message) {
                            RCLCPP_DEBUG(get_logger(), "成功创建AIS消息，类型: %d", 
                                         static_cast<int>(message->type()));
                            process_ais_message(message);
                        } else {
                            RCLCPP_WARN(get_logger(), "make_message返回空指针");
                        }
                    } catch (const std::bad_cast &e) {
                        RCLCPP_WARN(get_logger(), "类型转换失败 (bad_cast): %s", e.what());
                    } catch (const std::exception &e) {
                        RCLCPP_WARN(get_logger(), "解析AIS消息失败: %s (类型: %s)", 
                                   e.what(), typeid(e).name());
                    }
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "NMEA解析失败: %s，原始数据: %s", 
                         e.what(), data.c_str());
        }
    }

    /**
     * @brief 从message_01提取数据（辅助函数）
     */
    bool extract_ais_data_from_message_01(
        const std::unique_ptr<ais::message_01>& report,
        marnav_interfaces::msg::Ais& ais_msg) {
        
        if (!report) {
            return false;
        }

        // 验证数据有效性
        auto mmsi = report->get_mmsi();
        auto lat_opt = report->get_lat();
        auto lon_opt = report->get_lon();
        auto sog_opt = report->get_sog();
        auto cog_opt = report->get_cog();
        auto hdg_opt = report->get_hdg();

        if (mmsi == 0) {
            RCLCPP_DEBUG(get_logger(), "无效MMSI: 0");
            return false;
        }
        if (!lat_opt.has_value() || !lon_opt.has_value()) {
            RCLCPP_DEBUG(get_logger(), "无效位置数据");
            return false;
        }
        if (!sog_opt.has_value()) {
            RCLCPP_DEBUG(get_logger(), "无效速度数据");
            return false;
        }
        if (!cog_opt.has_value()) {
            RCLCPP_DEBUG(get_logger(), "无效航向数据");
            return false;
        }
        if (!hdg_opt.has_value()) {
            RCLCPP_DEBUG(get_logger(), "无效头向数据");
            return false;
        }

        // 填充AIS消息
        ais_msg.mmsi = mmsi;
        ais_msg.lat = lat_opt.value().get();
        ais_msg.lon = lon_opt.value().get();
        ais_msg.speed = sog_opt.value().value();
        ais_msg.course = cog_opt.value();
        ais_msg.heading = hdg_opt.value();
        
        return true;
    }

    /**
     * @brief 处理AIS消息（仅类型1/2/3）
     */
    void process_ais_message(std::unique_ptr<ais::message>& message) {
        if (!message) {
            RCLCPP_WARN(get_logger(), "process_ais_message: 消息指针为空");
            return;
        }

        auto msg_type = message->type();
        RCLCPP_DEBUG(get_logger(), "处理AIS消息，类型ID: %d", static_cast<int>(msg_type));

        // 仅处理位置报告类型1/2/3
        if (msg_type != ais::message_id::position_report_class_a &&
            msg_type != ais::message_id::position_report_class_a_assigned_schedule &&
            msg_type != ais::message_id::position_report_class_a_response_to_interrogation) {
            RCLCPP_DEBUG(get_logger(), "忽略AIS消息类型: %d (仅处理类型1/2/3)", 
                         static_cast<int>(msg_type));
            return;
        }

        // 创建AIS消息
        marnav_interfaces::msg::Ais ais_msg;
        bool extract_success = false;

        try {
            if (msg_type == ais::message_id::position_report_class_a) {
                // 类型1：位置报告（Class A）
                RCLCPP_DEBUG(get_logger(), "尝试转换为message_01 (类型1)");
                auto report = ais::message_cast<ais::message_01>(message);
                if (report) {
                    extract_success = extract_ais_data_from_message_01(report, ais_msg);
                    ais_msg.type = 1;
                    RCLCPP_DEBUG(get_logger(), "成功处理类型1消息");
                } else {
                    RCLCPP_WARN(get_logger(), "message_cast<message_01>返回空指针");
                }
            } else if (msg_type == ais::message_id::position_report_class_a_assigned_schedule) {
                // 类型2：位置报告（Class A，计划分配）
                // 注意：message_02 和 message_01 结构相同，但类型不同，不能直接转换
                // 暂时跳过类型2，或需要单独实现提取逻辑
                RCLCPP_DEBUG(get_logger(), "收到类型2消息，暂不支持处理");
                return;
            } else if (msg_type == ais::message_id::position_report_class_a_response_to_interrogation) {
                // 类型3：位置报告（Class A，响应询问）
                // 注意：message_03 和 message_01 结构相同，但类型不同，不能直接转换
                // 暂时跳过类型3，或需要单独实现提取逻辑
                RCLCPP_DEBUG(get_logger(), "收到类型3消息，暂不支持处理");
                return;
            }
        } catch (const std::bad_cast &e) {
            RCLCPP_WARN(get_logger(), "类型转换失败 (bad_cast): 消息类型=%d, 错误=%s", 
                       static_cast<int>(msg_type), e.what());
            return;
        } catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(), "类型转换异常: 消息类型=%d, 错误类型=%s, 错误信息=%s", 
                       static_cast<int>(msg_type), typeid(e).name(), e.what());
            return;
        }

        if (!extract_success) {
            RCLCPP_WARN(get_logger(), "数据提取失败，消息类型: %d", static_cast<int>(msg_type));
            return;
        }

        // 正确设置时间戳（builtin_interfaces/Time）
        rclcpp::Time ros_time = this->get_clock()->now();
        builtin_interfaces::msg::Time msg_time = ros_time;
        ais_msg.timestamp = msg_time;

        // 添加到当前批次缓存
        {
            std::lock_guard<std::mutex> lock(batch_mutex_);
            current_batch_.push_back(ais_msg);
        }

        RCLCPP_DEBUG(get_logger(), 
                     "AIS解析: MMSI=%lu, Lat=%.6f, Lon=%.6f, Speed=%.2f, Course=%.2f, Heading=%.2f",
                     ais_msg.mmsi, ais_msg.lat, ais_msg.lon, 
                     ais_msg.speed, ais_msg.course, ais_msg.heading);
    }

    /**
     * @brief 定时器回调：每秒发布一次批量AIS数据
     */
    void publish_batch_callback() {
        std::vector<marnav_interfaces::msg::Ais> batch_to_publish;
        
        {
            std::lock_guard<std::mutex> lock(batch_mutex_);
            if (current_batch_.empty()) {
                RCLCPP_DEBUG(get_logger(), "当前批次无AIS数据，跳过发布");
                return;
            }
            
            batch_to_publish = std::move(current_batch_);
            current_batch_.clear();
        }

        // 构建并发布AisBatch消息
        marnav_interfaces::msg::AisBatch batch_msg;
        batch_msg.ais_list = batch_to_publish;
        
        // 正确设置批次时间戳（builtin_interfaces/Time）
        rclcpp::Time ros_time = this->get_clock()->now();
        builtin_interfaces::msg::Time msg_time = ros_time;
        batch_msg.batch_time = msg_time;
        
        ais_batch_publisher_->publish(batch_msg);
        
        RCLCPP_INFO(get_logger(), 
                    "已发布AIS批次: %zu条数据，时间戳: %d.%09d",
                    batch_to_publish.size(),
                    batch_msg.batch_time.sec,
                    batch_msg.batch_time.nanosec);
    }


    /**
     * @brief 初始化通信接口（使用传入的参数）
     */
    void init_comm_interface(const std::string& comm_type,
                            const std::string& serial_port,
                            int baud_rate,
                            const std::string& tcp_ip,
                            int tcp_port,
                            const std::string& udp_ip,
                            int udp_port) {
        if (comm_type == "serial") {
            comm_interface_ = std::make_unique<SerialComm>(serial_port, baud_rate);
            RCLCPP_INFO(get_logger(), "选择串口通信: %s, 波特率: %d", 
                        serial_port.c_str(), baud_rate);
        }
        else if (comm_type == "tcp") {
            comm_interface_ = std::make_unique<TcpComm>(tcp_ip, tcp_port);
            RCLCPP_INFO(get_logger(), "选择TCP通信: %s:%d", tcp_ip.c_str(), tcp_port);
        } 
        else if (comm_type == "udp") {
            comm_interface_ = std::make_unique<UdpComm>(udp_ip, udp_port);
            RCLCPP_INFO(get_logger(), "选择UDP通信: %s:%d", udp_ip.c_str(), udp_port);
        } 
        else {
            RCLCPP_ERROR(get_logger(), "不支持的通信类型: %s", comm_type.c_str());
        }
    }


    /**
     * @brief 通信循环（独立线程）
     */
    void comm_loop() {
        while (running_ && rclcpp::ok()) {
            if (!comm_interface_->is_connected()) {
                RCLCPP_WARN(get_logger(), "通信连接已断开，尝试重连...");
                if (!comm_interface_->connect()) {
                    std::this_thread::sleep_for(1s);
                    continue;
                }
                RCLCPP_INFO(get_logger(), "通信连接已恢复");
            }

            std::string data = comm_interface_->read_data();
            if (!data.empty()) {
                
                // 解析AIVDM数据
                parse_aivdm(data);
            }

            std::this_thread::sleep_for(10ms);
        }
    }

    // 通信接口
    std::unique_ptr<CommInterface> comm_interface_;
    std::thread comm_thread_;
    std::atomic<bool> running_;

    // 发布者
    rclcpp::Publisher<marnav_interfaces::msg::AisBatch>::SharedPtr ais_batch_publisher_;

    // 话题名（由参数读取）
    std::string ais_batch_pub_topic_;

    // 定时器
    rclcpp::TimerBase::SharedPtr batch_timer_;

    // VDM片段缓存
    std::vector<std::unique_ptr<nmea::sentence>> vdm_fragments_;
    std::mutex vdm_mutex_;

    // AIS批次缓存
    std::vector<marnav_interfaces::msg::Ais> current_batch_;
    std::mutex batch_mutex_;

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AISBatchParserNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
