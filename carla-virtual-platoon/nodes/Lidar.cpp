#include "Lidar.hpp"

LidarPublisher::LidarPublisher(boost::shared_ptr<carla::client::Actor> actor)
    : Node("lidar_node" + std::to_string(actor->GetId()), rclcpp::NodeOptions()
               .allow_undeclared_parameters(false)
           .automatically_declare_parameters_from_overrides(true)){

    rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    
    this->get_parameter_or("add_sensor/lidar_number", num_lidars_, 1);
    this->get_parameter_or("carla/sync", sync_ , false);
    

    velocity_lidar_queue.resize(num_lidars_);

    for(int i=0; i<num_lidars_; ++i){
        std::string index = std::to_string(i);
        
        this->get_parameter_or("lidar" + index + "/x",lidar_x,2.3f);
        this->get_parameter_or("lidar" + index + "/y",lidar_y,0.0f);
        this->get_parameter_or("lidar" + index + "/z",lidar_z,1.0f);
        this->get_parameter_or("lidar" + index + "/pitch",lidar_pitch, 0.0f);
        this->get_parameter_or("lidar" + index + "/yaw",lidar_yaw,0.0f);
        this->get_parameter_or("lidar" + index + "/roll",lidar_roll,0.0f);
        this->get_parameter_or("lidar" + index + "/sensor_tick",lidar_sensor_tick,std::string("0.1f"));
        this->get_parameter_or("lidar" + index + "/rotation_frequency",lidar_rotation_frequency,std::string("30.0f"));
        this->get_parameter_or("lidar" + index + "/horizontal_fov",lidar_horizontal_fov,std::string("30.0f"));
        this->get_parameter_or("lidar" + index + "/upper_fov",lidar_upper_fov,std::string("0.0f"));
        this->get_parameter_or("lidar" + index + "/lower_fov",lidar_lower_fov,std::string("0.0f"));
        this->get_parameter_or("lidar" + index + "/points_per_second",lidar_points_per_second,std::string("30000"));
        this->get_parameter_or("lidar" + index + "/range",lidar_range,std::string("30.0f"));
        this->get_parameter_or("lidar" + index + "/topic_name",lidar_topic_name,std::string("carla/lidar" + index));
        if(sync_) lidar_sensor_tick = "0.0f";

        auto publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic_name, custom_qos);
        publishers_.push_back(publisher);
    
        auto lidar_bp = blueprint_library->Find("sensor.lidar.ray_cast");
        assert(lidar_bp != nullptr);

        auto lidar_bp_modifiable = *lidar_bp;

        lidar_bp_modifiable.SetAttribute("sensor_tick", lidar_sensor_tick);
        lidar_bp_modifiable.SetAttribute("rotation_frequency",lidar_rotation_frequency);
        lidar_bp_modifiable.SetAttribute("horizontal_fov", lidar_horizontal_fov);
        lidar_bp_modifiable.SetAttribute("points_per_second", lidar_points_per_second);
        lidar_bp_modifiable.SetAttribute("upper_fov", lidar_upper_fov);
        lidar_bp_modifiable.SetAttribute("lower_fov", lidar_lower_fov);
        lidar_bp_modifiable.SetAttribute("range", lidar_range);
        lidar_bp_modifiable.SetAttribute("channels","32");
    
        auto lidar_transform = cg::Transform{cg::Location{lidar_x, lidar_y, lidar_z}, cg::Rotation{lidar_pitch, lidar_yaw, lidar_roll}}; // pitch, yaw, roll.
        auto lidar_actor = world->SpawnActor(lidar_bp_modifiable, lidar_transform, actor.get());
        auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);

        lidar_sensors.push_back(lidar);
    
        lidar->Listen([this, i](auto data) {
            std::unique_lock<std::mutex> lock(mutex_);
            auto lidar_data = boost::static_pointer_cast<carla::sensor::data::LidarMeasurement>(data);
            assert(lidar_data != nullptr);
            publishLidarData(lidar_data, publishers_[i]);

        });
      }
}


void LidarPublisher::GetDelayParameter() {
    this->get_parameter_or("period/velocity_planner", velocity_planner_period, 30);
    this->get_parameter_or("delay/velocity_planner", velocity_planner_delay , 100);

    int lcm_all = lcm(velocity_planner_period, velocity_planner_delay);

    lcm_period = lcm_all; 
}


int LidarPublisher::gcd(int a, int b) {
    while (b != 0) {
        int temp = b;
        b = a % b;
        a = temp;
    }
    return a;
}

int LidarPublisher::lcm(int a, int b) {
    return (a * b) / gcd(a, b);
}

void LidarPublisher::publishLidarData(const boost::shared_ptr<csd::LidarMeasurement> &carla_lidar_measurement, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher) {
    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    point_cloud_msg.header.stamp = this->now();
    point_cloud_msg.header.frame_id = "laser";

    // Define fields
    point_cloud_msg.fields.resize(4);
    point_cloud_msg.fields[0].name = "x";
    point_cloud_msg.fields[0].offset = 0;
    point_cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg.fields[0].count = 1;

    point_cloud_msg.fields[1].name = "y";
    point_cloud_msg.fields[1].offset = 4;
    point_cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg.fields[1].count = 1;

    point_cloud_msg.fields[2].name = "z";
    point_cloud_msg.fields[2].offset = 8;
    point_cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg.fields[2].count = 1;

    point_cloud_msg.fields[3].name = "intensity";
    point_cloud_msg.fields[3].offset = 12;
    point_cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg.fields[3].count = 1;

    // Initialize a vector to hold the LiDAR data
    std::vector<float> lidar_data;
    // Assuming each detection is 4 floats (x, y, z, intensity)
    lidar_data.reserve(carla_lidar_measurement->size() * 4);

    // Iterate over each LiDAR detection
    for (const auto& detection : *carla_lidar_measurement) {
        if (detection.intensity < 0.96) {
          //  continue;
        }
        // Extract and process each point's data (x, y, z, intensity)
        lidar_data.push_back(detection.point.x);
        lidar_data.push_back(-detection.point.y); 
        lidar_data.push_back(detection.point.z);
        lidar_data.push_back(detection.intensity);
    }

    // Assemble PointCloud2 message
    point_cloud_msg.height = 1;
    point_cloud_msg.width = lidar_data.size() / 4;
    point_cloud_msg.is_dense = false;
    point_cloud_msg.is_bigendian = false;
    point_cloud_msg.point_step = sizeof(float) * 4;
    point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;

    point_cloud_msg.data.resize(point_cloud_msg.row_step);
    memcpy(&point_cloud_msg.data[0], lidar_data.data(), lidar_data.size() * sizeof(float));
    //RCLCPP_INFO(this->get_logger(), "lidar pub");
    publisher->publish(point_cloud_msg);
 
    }

