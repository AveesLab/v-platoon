#include "rosmanager.hpp"
#include <sys/mman.h>
#include <sys/stat.h>  /* For mode constants */
#include <fcntl.h>     /* For O_* constants */
#include <unistd.h>    /* For ftruncate */
#include <cstring>     /* For memset */
#include <iostream>

#define SHARED_MEMORY_NAME "/sync_memory"
#define SHARED_MEMORY_SIZE sizeof(int)

RosManager::RosManager()
    : Node("ros_manager_node"), cnt(0), size(0), shared_mem_ptr(nullptr) {
    // 구독자 생성
    TruckSizeSubscriber_ = this->create_subscription<std_msgs::msg::Int32>("/numtruckss", 10, std::bind(&RosManager::TruckSizeSubCallback, this, std::placeholders::_1));
    image_sync_sub_ = this->create_subscription<std_msgs::msg::Bool>("/img_sync_signal", 10, std::bind(&RosManager::SyncCallback, this, std::placeholders::_1));

    // 공유 메모리 초기화
    InitSharedMemory();
}

RosManager::~RosManager() {
    // 공유 메모리 정리
    CleanupSharedMemory();
}

void RosManager::InitSharedMemory() {
    int shm_fd = shm_open(SHARED_MEMORY_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("Failed to create shared memory");
        exit(1);
    }

    // 공유 메모리 크기 설정
    if (ftruncate(shm_fd, SHARED_MEMORY_SIZE) == -1) {
        perror("Failed to set size for shared memory");
        exit(1);
    }

    // 공유 메모리를 매핑
    shared_mem_ptr = (int *)mmap(0, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_mem_ptr == MAP_FAILED) {
        perror("Failed to map shared memory");
        exit(1);
    }

    // 초기 값으로 0 설정
    *shared_mem_ptr = 0;
    std::cout << "Shared memory initialized and set to 0" << std::endl;
}

void RosManager::CleanupSharedMemory() {
    // 공유 메모리 언매핑
    if (munmap(shared_mem_ptr, SHARED_MEMORY_SIZE) == -1) {
        perror("Failed to unmap shared memory");
    }

    // 공유 메모리 삭제
    if (shm_unlink(SHARED_MEMORY_NAME) == -1) {
        perror("Failed to unlink shared memory");
    }

    std::cout << "Shared memory cleaned up" << std::endl;
}

void RosManager::TruckSizeSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    std::cerr << "TruckSizeSubCallback : " << msg->data << std::endl;
    this->size = msg->data;
}

void RosManager::SyncCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Synchronized images received");
    cnt++;
    CheckAndPublish();
}

void RosManager::CheckAndPublish() {
    if (cnt == this->size) {
        RCLCPP_INFO(this->get_logger(), "Publishing synchronized images");

        // 공유 메모리에 1을 씀
        *shared_mem_ptr = 1;
        RCLCPP_INFO(this->get_logger(), "Shared memory updated with value 1");

        cnt = 0;
    } else {
        RCLCPP_INFO(this->get_logger(), "Not all images arrived");
    }
}
