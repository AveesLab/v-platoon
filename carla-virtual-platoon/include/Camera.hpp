#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <boost/shared_ptr.hpp>

#define SHARED_MEMORY_SIZE (640 * 480 * 4)

struct TimedImage {
    boost::shared_ptr<csd::Image> image;
    int timestamp;
};



class CameraPublisher : public rclcpp::Node {

public:
    CameraPublisher(boost::shared_ptr<carla::client::Actor> actor,int trucknum_);
    ~CameraPublisher() {
        for(auto cam : camera_actors) {
            cam->Destroy();
        }
    }
private:
    int cnt=0;
    int tmp;
    boost::shared_ptr<carla::client::Actor> actor_;
//    void publishImage(const csd::Image &carla_image);
    void publishImage(const csd::Image &carla_image, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr LaneImagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr CurImagePublisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr WaitLanePublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr WaitVelPublisher;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> shm_publishers_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> sync_publishers_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr image_sync_sub_;
    std::vector<boost::shared_ptr<cc::Sensor>> camera_sensors; 
    std::vector<boost::shared_ptr<cc::Actor>> camera_actors; 


    boost::shared_ptr<carla::client::Sensor> camera;
    boost::shared_ptr<carla::client::Actor> camera_actor;
    carla::geom::Transform camera_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> camera_bp;

    void publishImage1(const csd::Image &carla_image);

    int num_cameras_;
    bool sync_ = false;
    bool sync_with_delay = false;
    int tick_cnt = 0;
    int velocity_planner_period = 30;
    int velocity_planner_delay = 100;
    int path_planner_period = 30;
    int path_planner_delay = 100;
    std::vector<std::queue<TimedImage>> velocity_image_queue;
    std::queue<TimedImage> path_image_queue;
    void GetDelayParameter();
    int lcm_period;
    int gcd(int a, int b);
    int lcm(int a, int b);
    void wait_for_velocity(bool ch_);
    void wait_for_lane(bool ch_);

    float rgbcam_x;
    float rgbcam_y;
    float rgbcam_z;
    float rgbcam_pitch;
    float rgbcam_yaw;
    float rgbcam_roll;
    std::string rgbcam_sensor_tick;
    std::string rgbcam_topic_name;
    std::string role_name_;
    std::string rgbcam_image_size_x;
    std::string rgbcam_image_size_y;
    std::string rgbcam_fov;

    //shmem
    void *initialize_shared_memory(const std::string &name);
    std::vector<void *> shared_memory_ptrs_;  // shmem pointers
    void publishImage(const csd::Image &carla_image, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher, void *shared_memory, std::string name);
    std::vector<std::string> shm_name;
    void SyncSubCallback(const std_msgs::msg::Bool::SharedPtr msg);
    bool shm_com = false;
};
