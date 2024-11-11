#include <iostream>
#include <fstream>
#include <string>
#include <sys/time.h>
#include <vector>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <thread>
#include <shared_carlalib.h>
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include <sys/mman.h>
#include <sys/stat.h>  /* For mode constants */
#include <fcntl.h>     /* For O_* constants */
#include <unistd.h>    /* For ftruncate */
#include <cstring>     /* For memset */
#include <thread>
#include <string>
#include <ros2_msg/msg/target.hpp>
using namespace carla::traffic_manager;
#define SHARED_MEMORY_NAME "/sync_memory"
using namespace std;
class SyncManager : public rclcpp::Node {

public:
    SyncManager();
    ~SyncManager();
private:
    std::string host = "localhost";
    uint16_t port = 2000u;
    cc::Client* client;
    cc::World* world;
    carla::time_duration time_;
    carla::rpc::EpisodeSettings settings;
    bool first = false;
    bool first_check = false;
    mutex mutex_;
    bool isNodeRunning_ = false;
    bool registered = false;
    int size = 0;
    int cnt = 0;
    float sim_time = 0.0f;
    std::vector<bool> registration_;
    std::vector<bool> sync_throttle;
    std::vector<bool> sync_steer;
    std::thread manager_Thread;
    vector<unsigned int> truck_ids;
    vector<unsigned int> trailer_ids;
    void managerInThread();
    void recordData();
    void FindAllTruck();
    void timerCallback();
    rclcpp::TimerBase::SharedPtr timer_;
    double GetDistanceBetweenActors(ActorPtr current, ActorPtr target);
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ShutdownPublisher_;
    rclcpp::Publisher<ros2_msg::msg::Target>::SharedPtr TarPub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr GapPub_;
    size_t callback_id;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr TruckSizeSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr RegistrationSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr SyncSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr SyncThrottleSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr SyncSteerSubscriber_;


    //callback
    void TruckSizeSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void RegistrationSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void SyncThrottleSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void SyncSteerSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    bool check_register();
    bool sync_received();

    //steer
    float lv_steer = 0.0f;
    float fv1_steer = 0.0f;
    float fv2_steer = 0.0f;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr LVSteerSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr FV1SteerSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr FV2SteerSubscriber_;
    void LVSteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void FV1SteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void FV2SteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg);

    //lateral_error
    float lv_error = 0.0f;
    float fv1_error =0.0f;
    float fv2_error=0.0f;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr LVErrorSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr FV1ErrorSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr FV2ErrorSubscriber_;
    void LVErrorSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void FV1ErrorSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void FV2ErrorSubCallback(const std_msgs::msg::Float32::SharedPtr msg);


    int fd;
    int* shared_mem_ptr;
};