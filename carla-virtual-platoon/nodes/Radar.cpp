#include "Radar.hpp"
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

RadarPublisher::RadarPublisher(boost::shared_ptr<carla::client::Actor> actor)
    : Node("radar_node" + std::to_string(actor->GetId()), rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)) {

    rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

    this->get_parameter_or("add_sensor/radar_number", num_radars_, 3);
    this->get_parameter_or("carla/sync", sync_ , false);
    this->get_parameter_or("carla/sync_with_delay", sync_with_delay, false);

    velocity_radar_queue.resize(num_radars_);
    //publishers_.resize(num_radars_);

    for(int i=0; i<num_radars_; ++i){
        std::string index = std::to_string(i);

        this->get_parameter_or("radar" + index + "/x", radar_x, 2.3f);
        this->get_parameter_or("radar" + index + "/y", radar_y, 0.0f);
        this->get_parameter_or("radar" + index + "/z", radar_z, 1.5f);
        this->get_parameter_or("radar" + index + "/pitch", radar_pitch, 0.0f);
        this->get_parameter_or("radar" + index + "/yaw", radar_yaw, 0.0f);
        this->get_parameter_or("radar" + index + "/roll", radar_roll, 0.0f);
        this->get_parameter_or("radar" + index + "/sensor_tick", radar_sensor_tick, std::string("0.1"));
        this->get_parameter_or("radar" + index + "/horizontal_fov", radar_horizontal_fov, std::string("40.0"));
        this->get_parameter_or("radar" + index + "/vertical_fov", radar_vertical_fov, std::string("30.0"));
        this->get_parameter_or("radar" + index + "/points_per_second", radar_points_per_second, std::string("8000"));
        this->get_parameter_or("radar" + index + "/range", radar_range, std::string("300.0"));
        this->get_parameter_or("radar" + index + "/topic_name", radar_topic_name, std::string("carla/radar" + index));
        if(sync_) radar_sensor_tick = "0.0f";

        auto publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(radar_topic_name, custom_qos);
        publishers_.push_back(publisher);

        auto radar_bp = blueprint_library->Find("sensor.other.radar");
        assert(radar_bp != nullptr);
        
        // Create a modifiable copy of the radar blueprint
        auto radar_bp_modifiable = *radar_bp;

        radar_bp_modifiable.SetAttribute("sensor_tick", radar_sensor_tick);
        radar_bp_modifiable.SetAttribute("horizontal_fov", radar_horizontal_fov);
        radar_bp_modifiable.SetAttribute("points_per_second", radar_points_per_second);
        radar_bp_modifiable.SetAttribute("vertical_fov", radar_vertical_fov);
        radar_bp_modifiable.SetAttribute("range", radar_range);

        auto radar_transform = cg::Transform{cg::Location{radar_x, radar_y, radar_z}, cg::Rotation{radar_pitch, radar_yaw, radar_roll}};
        auto radar_actor = world->SpawnActor(radar_bp_modifiable, radar_transform, actor.get());
        auto radar = boost::static_pointer_cast<cc::Sensor>(radar_actor);

        radar_sensors.push_back(radar);

        radar->Listen([this, i](auto data) {
            std::unique_lock<std::mutex> lock(mutex_);
            auto radar_data = boost::static_pointer_cast<carla::sensor::data::RadarMeasurement>(data);
            assert(radar_data != nullptr);
            publishRadarData(radar_data, publishers_[i]);
        });
   }

}

void RadarPublisher::publishRadarData(const boost::shared_ptr<csd::RadarMeasurement> &carla_radar_measurement, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher) {
    sensor_msgs::msg::PointCloud2 radar_msg;
    radar_msg.header.stamp = this->now();
    radar_msg.header.frame_id = "laser";

    radar_msg.height = 1;
    radar_msg.width = carla_radar_measurement->GetDetectionAmount();
    radar_msg.is_dense = false;
    radar_msg.is_bigendian = false;

    std::vector<sensor_msgs::msg::PointField> fields(7);

    // Define PointFields for x, y, z, Range, Velocity, AzimuthAngle, and ElevationAngle
    fields[0].name = "x";
    fields[0].offset = 0;
    fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[0].count = 1;

    fields[1].name = "y";
    fields[1].offset = 4;
    fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[1].count = 1;

    fields[2].name = "z";
    fields[2].offset = 8;
    fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[2].count = 1;

    fields[3].name = "Range";
    fields[3].offset = 12;
    fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[3].count = 1;

    fields[4].name = "Velocity";
    fields[4].offset = 16;
    fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[4].count = 1;

    fields[5].name = "AzimuthAngle";
    fields[5].offset = 20;
    fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[5].count = 1;

    fields[6].name = "ElevationAngle";
    fields[6].offset = 24;
    fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[6].count = 1;

    radar_msg.fields = fields;

    radar_msg.point_step = sizeof(float) * fields.size();
    radar_msg.row_step = radar_msg.point_step * radar_msg.width;

    std::vector<uint8_t> data(radar_msg.row_step * radar_msg.height);
  
    size_t offset = 0;
   
    for (size_t i = 0; i < carla_radar_measurement->GetDetectionAmount(); ++i) {
      const auto &detection = carla_radar_measurement->at(i);
      float x = detection.depth * std::cos(detection.azimuth) * std::cos(-detection.altitude);
      float y = detection.depth * std::sin(-detection.azimuth) * std::cos(detection.altitude);
      float z = detection.depth * std::sin(detection.altitude);

      float range = detection.depth;
      range = std::round(range * 100.0f) / 100.0f;
      float velocity = detection.velocity;
      float azimuth_angle = detection.azimuth;
      float elevation_angle = detection.altitude;
      
      memcpy(&data[offset + fields[0].offset], &x, sizeof(float));
      memcpy(&data[offset + fields[1].offset], &y, sizeof(float));
      memcpy(&data[offset + fields[2].offset], &z, sizeof(float));
      memcpy(&data[offset + fields[3].offset], &range, sizeof(float));
      memcpy(&data[offset + fields[4].offset], &velocity, sizeof(float));
      memcpy(&data[offset + fields[5].offset], &azimuth_angle, sizeof(float));
      memcpy(&data[offset + fields[6].offset], &elevation_angle, sizeof(float));
 
      offset += radar_msg.point_step;  
    }
    radar_msg.data = data;
    publisher->publish(radar_msg);
 
 }



