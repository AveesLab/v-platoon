#ifndef ROSMANAGER_HPP
#define ROSMANAGER_HPP

#include <iostream>
#include <rclcpp/rclcpp.hpp>  
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <sys/mman.h>
#include <fcntl.h>  // For O_* constants
#include <unistd.h> // For ftruncate

class RosManager : public rclcpp::Node {

public:
    RosManager();
    ~RosManager();

private:
    int size;        // 트럭의 수를 저장
    int cnt;         // 동기화된 이미지의 수를 저장
    int* shared_mem_ptr; // 공유 메모리 포인터

    // ROS 2 구독자
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr image_sync_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr TruckSizeSubscriber_;

    // 콜백 함수
    void TruckSizeSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void SyncCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // 동기화된 이미지가 모두 도착했는지 확인하고 작업을 수행하는 함수
    void CheckAndPublish();

    // 공유 메모리 관리 함수
    void InitSharedMemory();
    void CleanupSharedMemory();
};

#endif // ROSMANAGER_HPP
