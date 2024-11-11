#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>
#include "ros2_msg/msg/v2_xcam.hpp"
#include "ros2_msg/msg/v2_xcustom.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <carla/client/WorldSnapshot.h>

using namespace carla::traffic_manager;

class TruckOBU : public rclcpp::Node {

public:
    TruckOBU(boost::shared_ptr<carla::client::Vehicle> vehicle_, boost::shared_ptr<carla::client::Actor> actor,int truck_num);
    ~TruckOBU() {
        //obu->Stop();
        //obu->Destroy();

        obu_custom->Destroy();
    }
private:
    void publishV2X(const unsigned int stationid_);
    rclcpp::Publisher<ros2_msg::msg::V2XCAM>::SharedPtr v2xpublisher_;
    rclcpp::Publisher<ros2_msg::msg::V2XCUSTOM>::SharedPtr v2xcustom_publisher_;
    std::string topic_name;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr EmergencyFlagSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr caution1Subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr caution2Subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr LaneChangeSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr TimeGapSubscriber_;

    void EmergencyFlagSubCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void caution1SubCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void caution2SubCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void LaneChangeSubCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void TimeGapSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void GetVel();
    void PublishV2V();

    boost::shared_ptr<carla::client::Sensor> obu;
    boost::shared_ptr<carla::client::Sensor> obu_custom;
    boost::shared_ptr<carla::client::Actor> obu_actor;
    carla::geom::Transform obu_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> obu_bp;
    std::string msg_from = "truck";
    bool find_preceding = false;
    unsigned int msg_from_id = 0;
    void timerCallback();
    rclcpp::TimerBase::SharedPtr timer_;
    boost::shared_ptr<carla::client::ServerSideSensor> v2x_custom;
    boost::shared_ptr<carla::client::Actor> v2x_custom_actor;
    carla::geom::Transform v2x_custom_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> v2x_custom_bp;
    boost::shared_ptr<carla::client::Vehicle> Vehicle_;
    bool emergency_flag = false;
    bool caution_mode_lane1 = false;
    bool caution_mode_lane2 = false;
    bool lane_change_flag = false;
    float timegap = 0.5f;
    int truck_num_;
    int tick = 0;
    float velocity_=0;
    carla::geom::Vector3D vel_;
    size_t callback_id;


};
