#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>

class TruckControl : public rclcpp::Node {

public:
    TruckControl(boost::shared_ptr<carla::client::Vehicle> vehicle_, int num);

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr SyncThrottlePublisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr SyncSteerPublisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr SteerSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr VelocitySubscriber_;
    void SteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void VelocitySubCallback(const std_msgs::msg::Float64::SharedPtr msg);
    boost::shared_ptr<carla::client::Vehicle> Vehicle_;
    carla::rpc::VehicleControl control;
    std::string steer_topic_name;
    std::string velocity_topic_name;
    bool sync_ = false;
    bool sync_with_delay = false;
    int trucknum = 0;
    carla::time_duration time_;
};