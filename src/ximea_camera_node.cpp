#include <rclcpp/rclcpp.hpp>
#include "ximea_camera/ximea_camera.hpp"

int main(int argc, char** argv) {
    // Init ROS2
    rclcpp::init(argc, argv);
    
    // Create the node
    auto node = std::make_shared<ximea_camera::XimeaROSCam>();
    
    // Spin the node
    rclcpp::spin(node);
    
    // Shutdown
    rclcpp::shutdown();
    return 0;
}
