// 该版本:
// 基础版本，只用于参考解码流程
#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialPort.h>
#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include <marnav/nmea/angle.hpp>
#include <marnav/nmea/nmea.hpp>
#include <marnav/nmea/rmc.hpp>
#include <marnav/nmea/gga.hpp>
#include <marnav/nmea/vtg.hpp>
#include <marnav/nmea/vdm.hpp>
#include <marnav/nmea/gsa.hpp>
#include <marnav/nmea/gsv.hpp>
#include <marnav/ais/ais.hpp>
#include <marnav/ais/message_01.hpp>
#include <marnav/ais/message_04.hpp>
#include <marnav/ais/message_05.hpp>
#include <marnav/nmea/sentence.hpp>
#include <marnav/nmea/ais_helper.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <fstream>
#include <chrono>
#include <filesystem>
#include <sstream>
#include <ctime>
#include <iomanip>
#include "builtin_interfaces/msg/time.hpp"  // 用于时间消息类型
#include <iomanip>                          // 用于setw和setfill格式化
#include "std_msgs/msg/string.hpp"  // 引入ROS 2字符串消息类型

#include "marnav_interfaces/msg/ais.hpp"
#include "marnav_interfaces/msg/gga.hpp"
#include "marnav_interfaces/msg/rmc.hpp"

// [INFO] [1757312253.694677996] [raw_ais_parser]: 3接收到数据: !AIVDM,2,1,0,A,56:btmP000030000001`PDQE`PuDQDwGKOP000164h<5500009f00000,0*2D
// 解析为 VDM 句子: VDM
// 收到 VDM 句子: 片段 1/2
// [ros2run]: Segmentation fault


// [INFO] [1757306588.647267085] [raw_ais_parser]: 3接收到数据: !AIVDM,1,1,,A,16:aUWU00R8cRIRAs>IqS@0@0000,0*3A
// 解析为 VDM 句子: VDM
// 收到 VDM 句子: 片段 1/1
// 所有 VDM 片段已收集完成，开始解析 AIS 消息...
// 提取并组合 AIS 有效载荷完成，开始生成 AIS 消息对象...
// AIS 消息类型: 位置报告（Class A），开始提取具体数据...
// AIS 位置报告: MMSI=413820318, 经度lontitude=121.359, 纬度latitude=31.3269, 速度=3.4, 相对于地面的实际运动航向=244.5, 相对于磁北的航向0, 检测到的船只类型


// [INFO] [1756538479.558293588] [raw_ais_parser]: 3接收到数据: !AIVDM,1,1,,A,16:cuC0P0S8cUtLAsGu9cWgp0000,0*61
// 解析为 VDM 句子: VDM
// 收到 VDM 句子: 片段 1/1
// 所有 VDM 片段已收集完成，开始解析 AIS 消息...
// 提取并组合 AIS 有效载荷完成，开始生成 AIS 消息对象...
// AIS 消息类型: 位置报告（Class A），开始提取具体数据...
// AIS 位置报告: MMSI=413859148, 纬度=31.331, 经度=121.371, 速度=3.5节, 相对于地面的实际运动航向=247.8, 相对于磁北的航向247
// [ros2run]: Segmentation fault

using namespace std::chrono_literals;
using namespace LibSerial;
using boost::asio::ip::tcp;
using boost::asio::ip::udp;
using namespace GeographicLib;
using namespace marnav;
namespace fs = std::filesystem;

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

class AISParserNode : public rclcpp::Node {
public:
    AISParserNode() : Node("raw_ais_parser"), comm_interface_(nullptr), running_(false), log_file_(nullptr) {
        // 声明所有配置参数（带默认值）
        declare_parameter<std::string>("comm_type", "serial");//serial tcp udp
        declare_parameter<std::string>("serial_port", "/dev/ttyS3");
        declare_parameter<int>("baud_rate", 38400);
        declare_parameter<std::string>("tcp_ip", "127.0.0.1");
        declare_parameter<int>("tcp_port", 5000);
        declare_parameter<std::string>("udp_ip", "192.168.0.10");
        declare_parameter<int>("udp_port", 1300);
        declare_parameter<std::string>("log_directory", "/home/tl/RV/ais_logs");
        declare_parameter<std::string>("frame_id", "ais");
        declare_parameter<bool>("log_raw_data", false);
        // double localLon=121.371; //本地位置经度
        // double localLat=31.331; //本地位置纬度
        declare_parameter<double>("local_longitude", 121.3738); //本地位置经度
        declare_parameter<double>("local_latitude", 31.3336); //本地位置纬度

        localLon_ = get_parameter("local_longitude").as_double();
        localLat_ = get_parameter("local_latitude").as_double();

        // 初始化AIS消息发布者
        nmea_publisher_ = create_publisher<std_msgs::msg::String>("nmea_data",10);
        ais_publisher_ = create_publisher<marnav_interfaces::msg::Ais>("ais_raw_data",10);
        // gga_publisher_ = create_publisher<marnav_interfaces::msg::Gga>("gga_raw_data",10),
        // rmc_publisher_ = create_publisher<marnav_interfaces::msg::Rmc>("rmc_raw_data",10),

        // 初始化日志系统
        init_logging();

        // 初始化通信接口
        init_comm_interface();

        // 启动通信线程
        if (comm_interface_ && comm_interface_->connect()) {
            running_ = true;
            comm_thread_ = std::thread(&AISParserNode::comm_loop, this);
            RCLCPP_INFO(get_logger(), "通信节点启动成功");
        } else {
            RCLCPP_FATAL(get_logger(), "通信接口初始化失败，节点启动失败");
            rclcpp::shutdown();
        }
    }

    ~AISParserNode() {
        running_ = false;
        if (comm_thread_.joinable()) {
            comm_thread_.join();
        }
        if (comm_interface_) {
            comm_interface_->disconnect();
        }
        // 关闭日志文件
        if (log_file_) {
            log_file_->close();
            delete log_file_;
        }
        RCLCPP_INFO(get_logger(), "通信节点已关闭");
    }

private:

    std::string trim_nmea(const std::string &s) {
        // 记录原始字符串的首尾位置，用于判断是否有空白被移除
        // auto original_start = s.begin();
        // auto original_end = s.end();

        // 跳过开头的空白字符
        auto start = s.begin();
        while (start != s.end() && std::isspace(*start)) {
            start++;
        }

        // 跳过结尾的空白字符
        auto end = s.end();
        do {
            end--;
        } while (std::distance(start, end) > 0 && std::isspace(*end));

        // 判断是否存在被移除的空白字符（开头有空白 或 结尾有空白）
        // bool has_whitespace = (start > original_start) || (end < original_end - 1);
        // if (has_whitespace) {
        //     std::cout << "检测到并移除了首尾的空白字符" << std::endl;
        // }
        // else {
        //     std::cout << "未检测到首尾的空白字符" << std::endl;
        // }

        // 返回清理后的字符串
        return std::string(start, end + 1);
    }

    void Loop_test_pub_fuction()
    {
        // 创建并发布 AIS 消息
        // uint64 mmsi
        // float64 lon
        // float64 lat
        // float64 speed
        // float64 course
        // float64 heading
        // uint8 type
        // builtin_interfaces/Time timestamp
        
        // AIS 位置报告: MMSI=413820318, 经度lontitude=121.359, 纬度latitude=31.3269, 速度=3.4, 相对于地面的实际运动航向=244.5, 相对于磁北的航向0, 检测到的船只类型

        auto ais_msg = marnav_interfaces::msg::Ais();
        ais_msg.mmsi = 123456789;
        ais_msg.lon = 121.3749859;
        ais_msg.lat = 31.3323638;
        ais_msg.speed = 3;
        ais_msg.course = 60;
        ais_msg.heading = 65;
        ais_msg.type = 1;
        
        // 设置时间戳
        ais_msg.timestamp = this->get_clock()->now();

        // 发布消息
        ais_publisher_->publish(ais_msg);
    }
    
    void nmea_publish(const std::string &data)
    {
        // 检查发布者是否已初始化
        if (!nmea_publisher_) {
            RCLCPP_ERROR(get_logger(), "NMEA发布者未初始化，无法发布数据");
            return;
        }

        // 检查数据是否为空
        if (data.empty()) {
            RCLCPP_WARN(get_logger(), "尝试发布空的NMEA数据，已忽略");
            return;
        }

        try {
            std_msgs::msg::String msg;
            msg.data = data;
            nmea_publisher_->publish(msg);
            // 可选：添加调试日志
            RCLCPP_DEBUG(get_logger(), "已发布NMEA数据: %s", data.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "发布NMEA数据失败: %s", e.what());
        }
    }

    void AIS_parse(const std::string &data)
    {
        std::string trimmed_data = trim_nmea(data);
        RCLCPP_INFO(get_logger(), "接收到数据: %s", trimmed_data.c_str());
        
        // Loop_test_pub_fuction();
        // 过滤非AIS相关的调试信息
        if (trimmed_data.substr(0, 6) == "$SYDBG") {
        RCLCPP_DEBUG(get_logger(), "忽略调试信息: %s", trimmed_data.c_str());
        return;
        }

        //
        try{
            auto sentence = nmea::make_sentence(trimmed_data);
            
// 1. VDM AIS船舶自动识别系统
            if(sentence->id() == nmea::sentence_id::VDM){
                const auto vdm = nmea::sentence_cast<marnav::nmea::vdm>(sentence.get()); //功能：将通用的 NMEA 句子指针转换为vdm类型的指针。 sentence_cast类似于 C++ 的dynamic_cast，用于获取 VDM 句子的具体接口，以便访问其特有的成员函数（如片段数量、 payload 等）。
                if(!vdm) // 增加空指针检查
                {
                    RCLCPP_ERROR(get_logger(), "无效的VDM句子");
                    return;
                }
                const auto n_fragments = vdm->get_n_fragments(); //功能：获取当前 AIS 消息被拆分的总片段数。
                const auto fragment = vdm->get_fragment(); //获取当前 VDM 句子的片段编号（从 1 开始）。
                RCLCPP_INFO(get_logger(), "收到VDM句子: 片段 %d/%d", fragment, n_fragments);

                std::lock_guard<std::mutex> lock(nmea_mutex_); // 加锁确保线程安全
                // 确保片段编号合法，避免越界
                if (fragment < 1 || fragment > n_fragments) {
                    RCLCPP_WARN(get_logger(), "无效的片段编号: %d/%d，清空缓存", fragment, n_fragments);
                    VDM_fragments.clear();
                    return;
                }
                // 若为新消息的第一段，先清空之前的缓存
                if (fragment == 1) {
                    // std::cout<<"若为新消息的第一段，先清空之前的缓存"<<std::endl;
                    VDM_fragments.clear();
                }
                VDM_fragments.push_back(std::move(sentence)); //将当前 VDM 句子添加到VDM_fragments向量中，用于收集所有片段。

                if(fragment == n_fragments) //如果当前片段编号等于总片段数时，意味着所有拆分的 VDM 句子已收集完成
                {
                    std::cout<<"所有 VDM 片段已收集完成，开始解析 AIS 消息..." << std::endl;
                    //功能：从收集到的所有 VDM 句子中提取并组合 AIS 有效载荷（payload）。 VDM_fragments参数是一个包含所有 VDM 句子的向量。
                    RCLCPP_DEBUG(get_logger(), "开始组合片段，总片段数: %zu", VDM_fragments.size());
                    auto payload = nmea::collect_payload(VDM_fragments.begin(),VDM_fragments.end()); 
                    VDM_fragments.clear();
                    try{
                        auto message = ais::make_message(payload);

                        if(message->type() == ais::message_id::position_report_class_a) //如果消息类型是位置报告（Class A）
                        {
                            std::cout<<"AIS 消息类型: 位置报告（Class A），开始提取具体数据..." << std::endl;
                            auto report = ais::message_cast<ais::message_01>(message);
                            if(report)
                            {

                                // uint64 mmsi
                                // float64 lon
                                // float64 lat
                                // float64 speed
                                // float64 course
                                // float64 heading
                                // uint8 type
                                // builtin_interfaces/Time timestamp

                                // 检查message是否有效
                                auto lat_opt = report->get_lat();
                                auto lon_opt = report->get_lon();
                                auto sog_opt = report->get_sog();
                                auto cog_opt = report->get_cog();
                                auto hdg_opt = report->get_hdg();
                                if(report->get_mmsi() == 0){
                                    std::cout << "无效NMSI数据: NMSI不可用" << std::endl;
                                    return;
                                }
                                if (!lat_opt.has_value() || !lon_opt.has_value()) {
                                    std::cout << "无效位置数据: 纬度或经度不可用" << std::endl;
                                    return;
                                }
                                if (!sog_opt.has_value()) {
                                    std::cout << "无效速度数据: SOG不可用" << std::endl;
                                    return;
                                }
                                if (!cog_opt.has_value()) {
                                    std::cout << "无效航向数据: COG不可用" << std::endl;
                                    return;
                                }
                                if (!hdg_opt.has_value()) {
                                    std::cout << "无效航向数据: HDG不可用" << std::endl;
                                    return;
                                }   

                                // 创建并发布 AIS 消息
                                auto ais_msg = marnav_interfaces::msg::Ais();
                                ais_msg.mmsi = report->get_mmsi();
                                ais_msg.lon = lon_opt.value().get();
                                ais_msg.lat = lat_opt.value().get();
                                ais_msg.speed = sog_opt.value().value();
                                ais_msg.course = cog_opt.value();
                                ais_msg.heading = hdg_opt.value();
                                ais_msg.type = 65;
                                
                                // 设置时间戳
                                ais_msg.timestamp = this->get_clock()->now();

                                // 发布消息
                                ais_publisher_->publish(ais_msg);

                                std::cout << "AIS 位置报告: MMSI=" << ais_msg.mmsi
                                          << ", 经度lontitude=" << ais_msg.lon
                                          << ", 纬度latitude=" << ais_msg.lat
                                          << ", 速度=" << ais_msg.speed
                                          << ", 相对于地面的实际运动航向=" << ais_msg.course
                                          << ", 相对于磁北的航向" << ais_msg.heading
                                          << ", 检测到的船只类型" << ais_msg.type
                                          << std::endl;
                                return;
                            } else {
                                std::cout << "无法转换为位置报告消息" << std::endl; 
                            }
                        }
                        else if(message->type() ==  ais::message_id::position_report_class_a_assigned_schedule){
                        //auto report = ais::message_cast<ais::message_02>(message);
                            std::cout << "AIS: Position report class A (assigned schedule)\n";
                            return;
                        }
                        else{
                            // ignore all others
                            std::cout<< "AIS: 其他类型消息，类型ID=" << static_cast<int>(message->type()) << std::endl;
                            return;
                        }

                    }
                    catch(const std::exception &e)
                    {
                        RCLCPP_WARN(get_logger(), "解析AIS消息失败: %s", e.what());
                    }
                }
                else{
                    return;
                }
            }
// 2. RMC 最小定位信息，$GPRMC:有用数据如:UTC时间，经纬度，地面速率，地面航向，UTC日期，真北航向角(0-180°），磁偏角，)
            if(sentence->id() == nmea::sentence_id::RMC){
                // // std::cout<<"解析为 RMC 句子: " << to_string(sentence->id()) << std::endl;
                // auto rmc =  nmea::sentence_cast<marnav::nmea::rmc>(sentence.get());
                // if(!rmc) // 增加空指针检查
                // {
                //     RCLCPP_ERROR(get_logger(), "无效的RMC句子");
                //     return;
                // }
                // std::lock_guard<std::mutex> lock(nmea_mutex_); // 加锁确保线程安全

                // // 构建RMC消息体
                // marnav_interfaces::msg::Rmc rmc_msg;

                // // 1. 接收时间戳（ROS时间）
                // rmc_msg.timestamp = this->get_clock()->now();

                // // 2. UTC时间（转换为builtin_interfaces/Time）
                // if (auto utc_time = rmc->get_time_utc(); utc_time.has_value()) {
                //     auto total_seconds = static_cast<uint32_t>(
                //         utc_time->hour() * 3600 + 
                //         utc_time->minutes() * 60 + 
                //         static_cast<int>(utc_time->seconds())
                //     );
                //     auto nanosec = static_cast<uint32_t>(
                //         (utc_time->seconds() - std::floor(utc_time->seconds())) * 1e9
                //     );
                //     rmc_msg.utc_time.sec = total_seconds;
                //     rmc_msg.utc_time.nanosec = nanosec;
                // }
                // else{
                //     RCLCPP_WARN(get_logger(), "RMC缺少UTC时间，跳过当前帧");
                //     return;
                // }
                
                // // 3. 状态（A=有效，V=无效）
                // if (auto status = rmc->get_status(); status.has_value()) {
                //     rmc_msg.status = *status;
                // }
                // else{
                //     RCLCPP_WARN(get_logger(), "RMC缺少状态，跳过当前帧");
                //     return;
                // }

                // // 4. 纬度（含南北半球转换）
                // if (auto lat = rmc->get_lat(); lat.has_value()) {
                //     rmc_msg.latitude = lat->get();  // 已包含半球信息（N为正，S为负）
                // }
                // else{{
                //     RCLCPP_WARN(get_logger(), "RMC缺少维度，跳过当前帧");
                //     return;
                // }}

                // // 5. 经度（含东西半球转换）
                // if (auto lon = rmc->get_lon(); lon.has_value()) {
                //     rmc_msg.longitude = lon->get();  // 已包含半球信息（E为正，W为负）
                // }
                // else{
                //     RCLCPP_WARN(get_logger(), "RMC缺少经度，跳过当前帧");
                //     return;
                // }

                // // 6. 对地速度（节）
                // if (auto sog = rmc->get_sog(); sog.has_value()) {
                //     rmc_msg.sog = sog->value();  // 转换为数值（节）
                // }
                // else{
                //     RCLCPP_WARN(get_logger(), "RMC缺少对地速度，跳过当前帧");
                //     return;
                // }

                // // 7. 航向（真北，度）
                // if (auto heading = rmc->get_heading(); heading.has_value()) {
                //     rmc_msg.heading = *heading;
                // }
                // else{
                //     RCLCPP_WARN(get_logger(), "RMC缺少航向，跳过当前帧");
                //     return;
                // }


                // // 8. 磁偏角及方向
                // if (auto mag = rmc->get_mag(); mag.has_value()) {
                //     rmc_msg.magnetic_variation = mag->angle();
                //     rmc_msg.magnetic_hemisphere = (mag->hemisphere() == nmea::direction::east) ? 'E' : 'W';
                // }
                // else{
                //     RCLCPP_WARN(get_logger(), "RMC缺少磁偏角，跳过当前帧");
                //     return;
                // }


                // // 发布RMC消息（需先定义发布者）
                // rmc_publisher_->publish(rmc_msg);
                // RCLCPP_INFO(get_logger(), "已解析并填充RMC消息");

                // return;

            }


// 3. GGA GPS定位信息: $GPGGA:有用数据如:UTC时间，经纬度，精度，天线高度，
            if(sentence->id() == nmea::sentence_id::GGA){
                // // std::cout<<"解析为 GGA 句子: " << to_string(sentence->id()) << std::endl;
                // auto gga =  nmea::sentence_cast<marnav::nmea::gga>(sentence.get());
                // if(!gga) // 增加空指针检查
                // {
                //     RCLCPP_ERROR(get_logger(), "无效的GGA句子");
                //     return;
                // }
                // std::lock_guard<std::mutex> lock(nmea_mutex_); // 加锁确保线程安全

                // marnav_interfaces::msg::Gga gga_msg;

                // // 1. 接收时间戳（ROS 时间）
                // gga_msg.timestamp = this->get_clock()->now();

                // // 2. UTC时间（核心修正：使用 marnav 正确的成员函数）
                // if (auto utc_time_opt = gga->get_time(); utc_time_opt.has_value()) {
                //     const auto& utc_time = utc_time_opt.value();  // 解包 optional
                    
                //     // 2.1 计算总秒数（时→3600秒，分→60秒，秒取整数部分）
                //     uint32_t hours = utc_time.hour();             // 正确：hour()（单数）→ 小时（0-23）
                //     uint32_t minutes = utc_time.minutes();         // 修正：minutes()（复数）→ 分钟（0-59）
                //     double seconds_total = utc_time.seconds();    // 修正：seconds()（复数）→ 秒（含小数，如5.123秒）
                //     uint32_t seconds_int = static_cast<uint32_t>(std::floor(seconds_total));  // 秒的整数部分（0-59）
                    
                //     uint32_t total_seconds = hours * 3600 + minutes * 60 + seconds_int;

                //     // 2.2 计算纳秒（秒的小数部分 × 1e9，转换为整数）
                //     double seconds_frac = seconds_total - seconds_int;  // 秒的小数部分（0 ≤ frac < 1）
                //     uint32_t nanosec = static_cast<uint32_t>(seconds_frac * 1e9);  // 纳秒（0-999,999,999）

                //     // 填充到消息的 utc_time 字段（builtin_interfaces/Time 类型）
                //     gga_msg.utc_time.sec = total_seconds;
                //     gga_msg.utc_time.nanosec = nanosec;
                // }
                // else{
                //     RCLCPP_WARN(get_logger(), "GGA缺少UTC时间，跳过当前帧");
                //     return;
                // }
                // // 若UTC时间无值，消息中 utc_time.sec 和 nanosec 保持默认0（符合ROS 2 optional语义）

                // // 3. 纬度（含南北半球转换，已在 marnav 内部处理正负）
                // if (auto lat_opt = gga->get_lat(); lat_opt.has_value()) {
                //     gga_msg.latitude = lat_opt.value().get();  // get() 返回浮点型度数（N为正，S为负）
                // }
                // else{
                //     RCLCPP_WARN(get_logger(), "GGA缺少维度，跳过当前帧");
                //     return;
                // }

                // // 4. 经度（含东西半球转换，同理）
                // if (auto lon_opt = gga->get_lon(); lon_opt.has_value()) {
                //     gga_msg.longitude = lon_opt.value().get();  // E为正，W为负
                // }
                // else{
                //     RCLCPP_WARN(get_logger(), "GGA缺少经度，跳过当前帧");
                //     return;
                // }

                // // 5. 定位质量指示（枚举值→uint8）
                // if (auto quality_opt = gga->get_quality_indicator(); quality_opt.has_value()) {
                //     gga_msg.quality_indicator = static_cast<uint8_t>(quality_opt.value());
                // }
                // else{
                //     RCLCPP_WARN(get_logger(), "GGA缺少定位质量，跳过当前帧");
                //     return;
                // }

                // // 6. 水平精度因子（HDOP，浮点型）
                // if (auto hdop_opt = gga->get_hor_dilution(); hdop_opt.has_value()) {
                //     gga_msg.hdop = hdop_opt.value();
                // }
                // else{
                //     RCLCPP_WARN(get_logger(), "GGA缺少水平精度，跳过当前帧");
                //     return;
                // }

                // // 7. 海拔高度（米，从 units::meters 提取数值）
                // if (auto alt_opt = gga->get_altitude(); alt_opt.has_value()) {
                //     gga_msg.altitude = alt_opt.value().value();  // value() 返回米的浮点值
                // }
                // else{
                //     RCLCPP_WARN(get_logger(), "GGA缺少海拔高度，跳过当前帧");
                //     return;
                // }

                // // 8. 大地水准面高度（米，同理）
                // if (auto geoid_opt = gga->get_geodial_separation(); geoid_opt.has_value()) {
                //     gga_msg.geoid_separation = geoid_opt.value().value();
                // }
                // else{
                //     RCLCPP_WARN(get_logger(), "GGA缺少大地水准面高度，跳过当前帧");
                //     return;
                // }

                // gga_publisher_->publish(gga_msg);
                // RCLCPP_DEBUG(get_logger(), "已发布GGA消息，UTC时间：%u秒 + %u纳秒",  gga_msg.utc_time.sec, gga_msg.utc_time.nanosec);

                // return;
            }
            if(sentence->id() == nmea::sentence_id::VTG){
                // std::cout<<"解析为 VTG 句子: " << to_string(sentence->id()) << std::endl;
            }
            if(sentence->id() == nmea::sentence_id::GSA){
                // std::cout<<"解析为 GSA 句子: " << to_string(sentence->id()) << std::endl;
            }
            if(sentence->id() == nmea::sentence_id::GSV){
                // std::cout<<"解析为 GSV 句子: " << to_string(sentence->id()) << std::endl;
            }
        }
        catch (const std::invalid_argument &e) {
        // 校验和格式错误的详细日志
            RCLCPP_ERROR(get_logger(), "NMEA解析失败: %s，原始数据: [%s]", e.what(), data.c_str());
            
            // 检查校验和部分格式
            size_t checksum_pos = data.find('*');
            if (checksum_pos == std::string::npos) {
                RCLCPP_ERROR(get_logger(), "错误：数据中没有找到校验和分隔符 '*'");
            } else if (checksum_pos + 2 >= data.size()) {
                RCLCPP_ERROR(get_logger(), "错误：校验和长度不足（需要2位），位置: %zu，数据长度: %zu", 
                    checksum_pos, data.size());
            } else {
                std::string checksum = data.substr(checksum_pos + 1, 2);
                RCLCPP_ERROR(get_logger(), "错误：校验和部分为 '%s'，需为2位十六进制数（0-9, A-F）", checksum.c_str());
            }
        } catch (const std::exception &e) {
            // 其他异常（如未知句子类型）
            RCLCPP_ERROR(get_logger(), "其他错误: %s，原始数据: %s", e.what(), data.c_str());
        }

    }
    void init_logging() {
        std::string log_dir = get_parameter("log_directory").as_string();
        if (log_dir.empty()) {
            RCLCPP_INFO(get_logger(), "未设置日志目录，不记录日志");
            return;
        }

        try {
            // 创建日志目录（如果不存在）
            fs::create_directories(log_dir);

            // 生成UTC时间戳文件名
            auto now = std::chrono::system_clock::now();
            auto now_utc = std::chrono::system_clock::to_time_t(now);
            std::tm tm = *std::gmtime(&now_utc);
            
            std::stringstream ss;
            ss << "nmea_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".log";
            std::string log_filename = ss.str();
            std::string log_path = fs::path(log_dir) / log_filename;

            // 打开日志文件
            log_file_ = new std::ofstream(log_path);
            if (log_file_->is_open()) {
                RCLCPP_INFO(get_logger(), "日志文件已创建: %s", log_path.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "无法打开日志文件: %s", log_path.c_str());
                delete log_file_;
                log_file_ = nullptr;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "日志初始化失败: %s", e.what());
        }
    }

    void init_comm_interface() {
        std::string comm_type = get_parameter("comm_type").as_string();
        if (comm_type == "serial") {
            std::string port = get_parameter("serial_port").as_string();
            int baud = get_parameter("baud_rate").as_int();
            comm_interface_ = std::make_unique<SerialComm>(port, baud);
            RCLCPP_INFO(get_logger(), "选择串口通信: %s, 波特率: %d", port.c_str(), baud);
        }
        else if (comm_type == "tcp") {
            std::string ip = get_parameter("tcp_ip").as_string();
            int port = get_parameter("tcp_port").as_int();
            comm_interface_ = std::make_unique<TcpComm>(ip, port);
            RCLCPP_INFO(get_logger(), "选择TCP通信: %s:%d", ip.c_str(), port);
        } 
        else if (comm_type == "udp") {
            std::string ip = get_parameter("udp_ip").as_string();
            int port = get_parameter("udp_port").as_int();
            comm_interface_ = std::make_unique<UdpComm>(ip, port);
            RCLCPP_INFO(get_logger(), "选择UDP通信: %s:%d", ip.c_str(), port);
        } 
        else {
            RCLCPP_ERROR(get_logger(), "不支持的通信类型: %s", comm_type.c_str());
        }
    }

    void write_log(const std::string& data) {
        if (!log_file_ || !log_file_->is_open()) return;

        std::lock_guard<std::mutex> lock(log_mutex_);
        try {
        // -------------------------- 原有：UTC时间字符串处理 --------------------------
            auto now = std::chrono::system_clock::now();
            auto now_utc = std::chrono::system_clock::to_time_t(now);
            std::tm tm = *std::gmtime(&now_utc);// UTC时区的年月日时分秒

            // -------------------------- 新增：ROS 2时间戳（this->get_clock()->now()） --------------------------
            rclcpp::Time ros_timestamp = this->get_clock()->now(); // 获取节点当前的ROS时间
            builtin_interfaces::msg::Time stamp = ros_timestamp;   // 转换为时间消息类型
            // 构建"秒.纳秒"格式的字符串（如"1757392678.044500441"）
            std::stringstream ros_ts_ss;
            ros_ts_ss << stamp.sec << "." << std::setw(9) << std::setfill('0') << stamp.nanosec;
            std::string ros_timestamp_str = ros_ts_ss.str(); 

            // -------------------------- 拼接日志：UTC时间字符串,ROS时间戳,原始数据 --------------------------
            std::stringstream ss;
            ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << "," << ros_timestamp_str<<"," << data << "\n";

            // 写入日志文件并强制刷新（避免缓存丢失）
            *log_file_ << ss.str();
            log_file_->flush();
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "日志写入失败: %s", e.what());
        }
    }

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

                //发布nmea原始数据
                nmea_publish(data);

                // 解析AIS数据
                AIS_parse(data);
                // 写入日志
                write_log(data);
            }

            std::this_thread::sleep_for(10ms);
        }
    }

    std::unique_ptr<CommInterface> comm_interface_;
    std::thread comm_thread_;
    std::atomic<bool> running_;
    std::mutex parse_mutex_;
    double localLat_ = 0.0;
    double localLon_ = 0.0;

    // NMEA和AIS消息发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nmea_publisher_;
    rclcpp::Publisher<marnav_interfaces::msg::Ais>::SharedPtr ais_publisher_;
    rclcpp::Publisher<marnav_interfaces::msg::Gga>::SharedPtr gga_publisher_;
    rclcpp::Publisher<marnav_interfaces::msg::Rmc>::SharedPtr rmc_publisher_;

    // 新增：用于存储多片段AIS消息
    std::vector<std::unique_ptr<nmea::sentence>> VDM_fragments;
    std::mutex nmea_mutex_; // 确保线程安全

    // 日志相关成员
    std::ofstream* log_file_;
    std::string log_directory_;
    std::mutex log_mutex_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AISParserNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}