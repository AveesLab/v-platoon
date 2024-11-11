#include <rclcpp/rclcpp.hpp>
#include "manager.hpp"
#include "rosmanager.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    //rclcpp::executors::MultiThreadedExecutor executor; 
    //auto node_ros = std::make_shared<RosManager>();
    //auto node_carla = std::make_shared<SyncManager>();


    //executor.add_node(node_ros);
    //e//xecutor.add_node(node_carla);
    //executor.spin();
    rclcpp::spin(std::make_shared<SyncManager>()); 

    rclcpp::shutdown();
}