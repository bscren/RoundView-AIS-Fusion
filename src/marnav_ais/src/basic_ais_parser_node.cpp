#include <rclcpp/rclcpp.hpp>
#include <string>
#include <libserial/SerialPort.h>
using namespace LibSerial;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ais_parser_node");
    SerialPort serial_port;

    try {
        // 步骤1：打开串口
        serial_port.Open("/dev/ttyUSB0");
        RCLCPP_INFO(node->get_logger(), "✅ 串口 /dev/ttyUSB0 打开成功");

        // 步骤2：打开后立即检查是否真的处于打开状态
        if (!serial_port.IsOpen()) {
            RCLCPP_FATAL(node->get_logger(), "❌ 串口打开后状态异常，IsOpen() 返回 false");
            rclcpp::shutdown();
            return -1;
        }

        // 步骤3：设置串口参数（每步都加日志，确认参数生效）
        serial_port.SetBaudRate(BaudRate::BAUD_38400);
        RCLCPP_INFO(node->get_logger(), "✅ 波特率设置为 38400");
        
        serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        RCLCPP_INFO(node->get_logger(), "✅ 数据位设置为 8");
        
        serial_port.SetParity(Parity::PARITY_NONE);
        RCLCPP_INFO(node->get_logger(), "✅ 校验位设置为 NONE");
        
        serial_port.SetStopBits(StopBits::STOP_BITS_1);
        RCLCPP_INFO(node->get_logger(), "✅ 停止位设置为 1");
        
        serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);  // 必须设置流控（默认可能不是NONE）
        RCLCPP_INFO(node->get_logger(), "✅ 流控设置为 NONE");

    } catch (const OpenFailed& e) {
        RCLCPP_FATAL(node->get_logger(), "❌ 串口打开失败（设备不存在/被占用）: %s", e.what());
        rclcpp::shutdown();
        return -1;
    } catch (const std::exception& e) {  // 捕获参数设置等其他异常
        RCLCPP_FATAL(node->get_logger(), "❌ 串口初始化异常: %s", e.what());
        if (serial_port.IsOpen()) serial_port.Close();
        rclcpp::shutdown();
        return -1;
    }

    // 步骤4：读取数据（增加重试逻辑，避免单次超时误判）
    std::string read_data;
    const int MAX_RETRY = 3000;  // 重试3次
    bool read_success = false;

    for (int i = 0; i < MAX_RETRY; ++i) {
        try {
            RCLCPP_INFO(node->get_logger(), "🔍 第 %d 次尝试读取数据（超时1秒）", i+1);
            serial_port.ReadLine(read_data, '\n', 1000);  // 读取一行（以\n结束，超时1秒）
            
            if (!read_data.empty()) {
                RCLCPP_INFO(node->get_logger(), "✅ 读取成功！数据: %s", read_data.c_str());
                read_success = true;
                break;
            } else {
                RCLCPP_WARN(node->get_logger(), "⚠️  读取到空数据，重试...");
            }

        } catch (const ReadTimeout& e) {
            RCLCPP_WARN(node->get_logger(), "⚠️  第 %d 次读取超时: %s", i+1, e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "❌ 第 %d 次读取异常: %s", i+1, e.what());
            break;
        }
    }

    if (!read_success) {
        RCLCPP_WARN(node->get_logger(), "⚠️  多次读取失败，可能设备无数据输出（检查设备是否正常工作）");
    }

    // 步骤5：关闭串口
    // if (serial_port.IsOpen()) {
    //     serial_port.Close();
    //     RCLCPP_INFO(node->get_logger(), "✅ 串口已关闭");
    // }

    rclcpp::shutdown();
    return 0;
}