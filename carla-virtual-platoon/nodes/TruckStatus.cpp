#include "TruckStatus.hpp"

TruckStatusPublisher::TruckStatusPublisher(boost::shared_ptr<carla::client::Vehicle> vehicle_,boost::shared_ptr<carla::client::Actor> actor, int trucknum_)
    : Node("truck_status_node", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)),Vehicle_(vehicle_) {

    this->get_parameter_or("info_topic_name",info_topic_name,std::string("velocity_info"));
    this->actor_ = actor;
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

    AccelPublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("accel",1);
    VelocityPublisher_ = this->create_publisher<std_msgs::msg::Float32>(info_topic_name,1);
    ShutdownSubscriber = this->create_subscription<std_msgs::msg::String>("/shutdown_topic", 10, std::bind(&TruckStatusPublisher::shutdown_callback, this, std::placeholders::_1));
    DistanceSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("min_distance", 10, std::bind(&TruckStatusPublisher::DistanceSubCallback, this, std::placeholders::_1));
    CutinFlagSubscriber_ = this->create_subscription<std_msgs::msg::Bool>("cut_in_flag", 10, std::bind(&TruckStatusPublisher::CutinFlagSubCallback, this, std::placeholders::_1));
    SotifSubscriber_ = this->create_subscription<std_msgs::msg::Bool>("front_camera/attribute", 10, std::bind(&TruckStatusPublisher::SotifSubCallback, this, std::placeholders::_1));
    gettimeofday(&init_, NULL);
    //timer_100ms_record = this->create_wall_timer(100ms, std::bind(&TruckStatusPublisher::TruckStatus_record_callback, this));
    this->trucknum_ = trucknum_;
    IMUPublisher_ = this->create_publisher<ros2_msg::msg::IMU>("imu",qos);
    GnssPublisher_ = this->create_publisher<ros2_msg::msg::GNSS>("gnss",1);
    //IMU
    generate_imu();
    generate_gnss();
}

void TruckStatusPublisher::generate_imu() {
    auto imu_bp = blueprint_library->Find("sensor.other.imu");
    auto imu_bp_modifiable = *imu_bp;
    assert(imu_bp != nullptr);
    imu_bp_modifiable.SetAttribute("sensor_tick", "0.0");


    imu_transform = cg::Transform{ cg::Location{}, cg::Rotation{}}; // pitch, yaw, roll.
    imu_actor = world->SpawnActor(imu_bp_modifiable, imu_transform, actor_.get());
    imu = boost::static_pointer_cast<cc::Sensor>(imu_actor);

    imu->Listen([this](auto data) {
        auto data_ = boost::static_pointer_cast<carla::sensor::data::IMUMeasurement>(data);
        assert(data != nullptr);
        publishIMUData(data_);
    });
}

void TruckStatusPublisher::generate_gnss() {
    auto gnss_bp = blueprint_library->Find("sensor.other.gnss");
    assert(gnss_bp != nullptr);
    auto gnss_bp_modifiable = *gnss_bp;
    gnss_bp_modifiable.SetAttribute("sensor_tick", "0.0");


    gnss_transform = cg::Transform{ cg::Location{}, cg::Rotation{}}; // pitch, yaw, roll.
    gnss_actor = world->SpawnActor(gnss_bp_modifiable, gnss_transform, actor_.get());
    gnss = boost::static_pointer_cast<cc::Sensor>(gnss_actor);

    gnss->Listen([this](auto data) {
        auto data_ = boost::static_pointer_cast<carla::sensor::data::GnssMeasurement>(data);
        assert(data != nullptr);
        publishGnssData(data_);
    });
}
void TruckStatusPublisher::publishIMUData(const boost::shared_ptr<csd::IMUMeasurement> &carla_imu_measurement) {
    //ros2_msg::msg::IMU msg;
        // Iterate over each LiDAR detection
    float compass_ = carla_imu_measurement->GetCompass() * (180.0 / M_PI);
    auto gyro_ = carla_imu_measurement->GetGyroscope();
    auto accel_ = Vehicle_->GetAcceleration();
    TruckStatusPublisher_velocity_callback();
    ros2_msg::msg::IMU msg;
    msg.header.stamp = this->now();
    msg.compass = compass_;
    msg.gyro_x = gyro_.x;
    msg.gyro_y = gyro_.y;
    msg.gyro_z = gyro_.z;
    msg.accel_x = accel_.x;
    msg.accel_y = accel_.y;
    msg.accel_z = accel_.z;
    IMUPublisher_->publish(msg);
    TruckStatusPublisher_accel_callback();
    this->sim_time += 0.01f;
}

void TruckStatusPublisher::publishGnssData(const boost::shared_ptr<csd::GnssMeasurement> &carla_gnss_measurement) {
    //ros2_msg::msg::IMU msg;
    double latitude  = carla_gnss_measurement->GetLatitude();
    double longitude = carla_gnss_measurement->GetLongitude();
    double altitude = carla_gnss_measurement->GetAltitude();
    ros2_msg::msg::GNSS msg;
    msg.header.stamp = this->now();
    msg.latitude = latitude;
    msg.longitude = longitude;
    msg.altitude = altitude;
    GnssPublisher_->publish(msg);
}


void TruckStatusPublisher::TruckStatusPublisher_accel_callback() {
    acc_ = Vehicle_->GetAcceleration();
    auto message = std_msgs::msg::Float32MultiArray();
    message.data.push_back(acc_.x);
    message.data.push_back(acc_.y);
    message.data.push_back(acc_.z);
    float result_acc = acc_.x; // m/s
    acceleration_ = result_acc;
    AccelPublisher_->publish(message);
}


void TruckStatusPublisher::TruckStatusPublisher_velocity_callback() {
    vel_ = Vehicle_->GetVelocity();
    auto message = std_msgs::msg::Float32();
    float result_vel = vel_.x; // m/s
    //result_vel = std::round(result_vel * 100.0f) / 100.0f;
    message.data = result_vel;
    std::cerr << message.data  << std::endl;
    velocity_ = result_vel;
    VelocityPublisher_->publish(message);
}






void TruckStatusPublisher::DistanceSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    distance_ = msg->data;
}

void TruckStatusPublisher::CutinFlagSubCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  //std::cerr << "recv" << std::endl;
    bool data = msg->data;
    if(data) cut_in_flag = 1.0;
    else cut_in_flag = 0.0; 
}

void TruckStatusPublisher::SotifSubCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if(check) {
      sotif_flag = 0.0f;
      check = false;
    }
    else {
      sotif_flag = 1.0f;
      check = true;
    }

}

void TruckStatusPublisher::shutdown_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received shutdown message: '%s'", msg->data.c_str());
    rclcpp::shutdown(); // 종료 명령
}

void TruckStatusPublisher::TruckStatus_record_callback() {
    if(velocity_ != 0.) recordData(init_);
}

void TruckStatusPublisher::recordData(struct timeval startTime){
    struct timeval currentTime;
    char file_name[] = "SCT_log00.csv";
    static char file[128] = {0x00, };
    char buf[256] = {0x00,};
    static bool flag = false;
    double diff_time;
    log_path_ = "/home/nvidia/ros2_ws/logfiles/";
    std::ifstream read_file;
    std::ofstream write_file;
    if(!flag){
      for(int i = 0; i < 100; i++){
        file_name[7] = i/10 + '0';  //ASCII
        file_name[8] = i%10 + '0';
        sprintf(file, "%s%s", log_path_.c_str(), file_name);
        read_file.open(file);
        if(read_file.fail()){  //Check if the file exists
          read_file.close();
          write_file.open(file);
          break;
        }
        read_file.close();
      }
      write_file << "Time, Velocity, Acceleration, Distance,cut_in_flag,sotif_flag" << std::endl; 
      flag = true;
    }
    if(flag){
  //    std::scoped_lock lock(lane_mutex_, rlane_mutex_, vel_mutex_, dist_mutex_, rep_mutex_);
      gettimeofday(&currentTime, NULL);
      diff_time = ((currentTime.tv_sec - startTime.tv_sec)) + ((currentTime.tv_usec - startTime.tv_usec)/1000000.0);
      sprintf(buf, "%.10e, %.3f, %.3f, %.3f,%.3f,%.3f", diff_time, velocity_, acceleration_, distance_,cut_in_flag,sotif_flag);
      write_file.open(file, std::ios::out | std::ios::app);
      write_file << buf << std::endl;
    }
    write_file.close();
}

