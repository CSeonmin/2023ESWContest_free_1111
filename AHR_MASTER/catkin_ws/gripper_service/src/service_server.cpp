#include "ros/ros.h"
// ROS 기본 헤더 파일
#include "gripper_service/Srvgripper.h" // SrvTutorial 서비스 파일 헤더 (빌드후 자동 생성됨)
#include <dynamixel_sdk/dynamixel_sdk.h>//다이나믹셀 사용 헤더
#include <std_msgs/String.h>

// MX-28 모터 정보
#define MOTOR_ID_1            9
#define MOTOR_ID_2           10
#define MOTOR_ID_3           11
#define MODEL_NUMBER        311
#define DEVICE_NAME       "/dev/ttyUSB1"
#define PROTOCOL_VERSION     2.0
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
// 서비스 요청이 있을 경우, 아래의 처리를 수행한다
// 서비스 요청은 req, 서비스 응답은 res로 설정하였다
bool grip(gripper_service::Srvgripper::Request &req,
gripper_service::Srvgripper::Response &res)
{
std::string data_str = std::to_string(req.data);
const char* data_cstr = data_str.c_str();
if(data_str == "1"){
      
    packetHandler->write4ByteTxRx(portHandler, MOTOR_ID_1, 116 ,1692, nullptr);
    packetHandler->write4ByteTxRx(portHandler, MOTOR_ID_2, 116 ,2296, nullptr);
    packetHandler->write4ByteTxRx(portHandler, MOTOR_ID_3, 116 ,1768, nullptr);
    ROS_INFO("Gripper ALL_OPEN");
}else if(data_str  == "2"){
    packetHandler->write4ByteTxRx(portHandler, MOTOR_ID_1, 116 ,2000, nullptr);
    packetHandler->write4ByteTxRx(portHandler, MOTOR_ID_2, 116 ,2006, nullptr);
    packetHandler->write4ByteTxRx(portHandler, MOTOR_ID_3, 116 ,2082, nullptr);
    ROS_INFO("Gripper ALL_CLOSE");
}

res.result = req.data + 2;
// 서비스 요청에 사용된 a, b 값의 표시 및 서비스 응답에 해당되는 result 값을 출력한다
ROS_INFO("request: %ld",(long int) req.data);
ROS_INFO("sending back response: %ld",(long int)res.result);
return true;
}
int main(int argc, char **argv)
{
ros::init(argc, argv, "service_server");
ros::NodeHandle nh;


    if (!portHandler->openPort()) {
        ROS_ERROR("Failed to open port!");
        return -1;
    }

    if (!portHandler->setBaudRate(1000000)) {
        ROS_ERROR("Failed to set baud rate!");
        return -1;
    }

    // MX-28 모터 활성화
    packetHandler->write1ByteTxRx(portHandler, MOTOR_ID_1, 64, 1, nullptr);
    packetHandler->write1ByteTxRx(portHandler, MOTOR_ID_2, 64, 1, nullptr);
    packetHandler->write1ByteTxRx(portHandler, MOTOR_ID_3, 64, 1, nullptr);


ros::ServiceServer gripper_service_server = nh.advertiseService("gripper_srv", grip);
ROS_INFO("ready srv server!");
ros::spin(); // 서비스 요청을 대기한다
return 0;
}
