#include "TruckOBU.hpp"

TruckOBU::TruckOBU(boost::shared_ptr<carla::client::Vehicle> vehicle_,boost::shared_ptr<carla::client::Actor> actor, int truck_num)
    : Node("truck_obu_node", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)) {

    rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    custom_qos.reliable();
    //timer_ = this->create_wall_timer(10ms, std::bind(&TruckOBU::timerCallback, this));
    truck_num_ = truck_num;
    Vehicle_ = vehicle_;
    //topic_name = "/truck" + std::to_string(truck_num_ + 1) + "/v2xcam";
    v2xpublisher_ = this->create_publisher<ros2_msg::msg::V2XCAM>("v2xcam", 1);
    v2xcustom_publisher_ = this->create_publisher<ros2_msg::msg::V2XCUSTOM>("v2xcustom",1);

    EmergencyFlagSubscriber_ = this->create_subscription<std_msgs::msg::Bool>("emergency_flag", 10, std::bind(&TruckOBU::EmergencyFlagSubCallback, this, std::placeholders::_1));
    caution1Subscriber_ = this->create_subscription<std_msgs::msg::Bool>("caution_mode_lane1", 10, std::bind(&TruckOBU::caution1SubCallback, this, std::placeholders::_1));
    caution2Subscriber_ = this->create_subscription<std_msgs::msg::Bool>("caution_mode_lane2", 10, std::bind(&TruckOBU::caution2SubCallback, this, std::placeholders::_1));
    LaneChangeSubscriber_ = this->create_subscription<std_msgs::msg::Bool>("lane_change_flag", 10, std::bind(&TruckOBU::LaneChangeSubCallback, this, std::placeholders::_1));
    if(truck_num == 0) TimeGapSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("timegap", 10, std::bind(&TruckOBU::TimeGapSubCallback, this, std::placeholders::_1));

    
    auto obu_bp = blueprint_library->Find("sensor.other.v2x");
    assert(obu_bp != nullptr);
    auto obu_bp_modifiable = *obu_bp;
    
    obu_bp_modifiable.SetAttribute("path_loss_model", "geometric");
    obu_bp_modifiable.SetAttribute("gen_cam_max","0.05");
    obu_bp_modifiable.SetAttribute("scenario","highway");
   // obu_bp_modifiable.SetAttribute("fixed_rate","True");
   // obu_bp_modifiable.SetAttribute("use_etsi_fading","false");

    obu_transform = cg::Transform(); // pitch, yaw, roll.
    obu_actor = world->SpawnActor(obu_bp_modifiable, obu_transform, actor.get());
    obu = boost::static_pointer_cast<cc::Sensor>(obu_actor);

    //ICRA
    msg_from += std::to_string(truck_num-1);
    while(truck_num!=0 && !find_preceding) {
       auto actor_list = world->GetActors();
       for (auto iter = actor_list->begin(); iter != actor_list->end(); ++iter) {
            ActorPtr actor = *iter;
            ActorId actor_id = actor->GetId();
            if (actor->GetTypeId().front() == 'v') {
              
                for (auto&& attribute: actor->GetAttributes()) {
                  if (attribute.GetValue() == msg_from) {
                      msg_from_id = actor_id;
                      std::cerr << "find " <<actor_id <<std::endl;
                      find_preceding = true;
                      break;
                  }
                }
            }
      } 
    }


    obu->Listen([this](auto data) {
        auto CAM = boost::static_pointer_cast<csd::CAMEvent>(data);
          for (auto& data : *CAM) {
            auto msg = data.Message; 
            if(msg.header.stationID == msg_from_id ) {
              //RCLCPP_INFO(this->get_logger(), "stationID: %ld",msg.header.stationID);
              //RCLCPP_INFO(this->get_logger(), "latitude: %ld, longitude: %ld",msg.cam.camParameters.basicContainer.referencePosition.latitude, msg.cam.camParameters.basicContainer.referencePosition.longitude);
              //RCLCPP_INFO(this->get_logger(), "Heading: %ld",msg.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.heading.headingValue);
              RCLCPP_INFO(this->get_logger(), "speed: %ld",msg.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.speed.speedValue);
              ros2_msg::msg::V2XCAM msg_;
              msg_.header.stamp = this->now();
              msg_.stationid = msg.header.stationID;
              msg_.latitude = msg.cam.camParameters.basicContainer.referencePosition.latitude;
              msg_.longitude = msg.cam.camParameters.basicContainer.referencePosition.longitude;
              msg_.heading = msg.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.heading.headingValue;
              msg_.speed = msg.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.speed.speedValue;

              v2xpublisher_->publish(msg_);
            }
            
        }
    });

    auto v2x_custom_bp = blueprint_library->Find("sensor.other.v2x_custom");
    assert(v2x_custom_bp != nullptr);
    auto v2x_custom_bp_modifiable = *v2x_custom_bp;
    v2x_custom_bp_modifiable.SetAttribute("path_loss_model", "geometric");
    v2x_custom_bp_modifiable.SetAttribute("scenario","highway");

    v2x_custom_transform = cg::Transform{ cg::Location{}, cg::Rotation{}}; // pitch, yaw, roll.
    v2x_custom_actor = world->SpawnActor(v2x_custom_bp_modifiable, v2x_custom_transform, actor.get());
    obu_custom = boost::static_pointer_cast<cc::Sensor>(v2x_custom_actor);
    v2x_custom = boost::static_pointer_cast<cc::ServerSideSensor>(v2x_custom_actor);

    v2x_custom->Listen([this](auto data) {
        auto image = boost::static_pointer_cast<csd::CustomV2XEvent>(data);
        //assert(image != nullptr);
       // publishImage(*image);
          for (auto& data : *image) {
            ros2_msg::msg::V2XCUSTOM msg_;
            // Assuming `data` has a `get()` method returning the message
            // Replace with actual method to access the message content
            auto msg = data.Message;
            auto power = data.Power;
//            RCLCPP_INFO(this->get_logger(), "Publishing: %d",msg.header.stationID);
//            RCLCPP_INFO(this->get_logger(), "Publishing: %s",msg.message);

            std::istringstream iss(msg.message);
            std::string token;

            // Parse emergency_flag
            getline(iss, token, ':');
            getline(iss, token, ',');
            token.erase(0, token.find_first_not_of(" \t\n\r\f\v"));
            msg_.emergency_flag = (token == "true");

            // Parse caution_mode_lane1
            getline(iss, token, ':');
            getline(iss, token, ',');
            token.erase(0, token.find_first_not_of(" \t\n\r\f\v"));
            msg_.caution_mode_lane1 = (token == "true");


            // Parse lane_change_flag
            getline(iss, token, ':');
            getline(iss, token, ',');
            token.erase(0, token.find_first_not_of(" \t\n\r\f\v"));
            msg_.lane_change_flag = (token == "true");

            // Parse timegap
            getline(iss, token, ':');
            iss >> msg_.timegap;

            v2xcustom_publisher_->publish(msg_);
        }
    });
    
}

void TruckOBU::timerCallback() {
    
    std::ostringstream oss;
    oss << "emergency_flag: " << (this->emergency_flag ? "true" : "false") << ", "
        << "caution_mode_lane1: " << (this->caution_mode_lane1 ? "true" : "false") << ", "
        << "lane_change_flag: " << (this->lane_change_flag ? "true" : "false") << ", "
        << "timegap: " << this->timegap;

    v2x_custom->Send(oss.str());
}



void TruckOBU::publishV2X(const unsigned int stationid_) {

}

void TruckOBU::EmergencyFlagSubCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    this->emergency_flag = msg->data;
}

void TruckOBU::caution1SubCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    this->caution_mode_lane1 = msg->data;
}

void TruckOBU::caution2SubCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    this->caution_mode_lane2 = msg->data;
}

void TruckOBU::LaneChangeSubCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    this->lane_change_flag = msg->data;
}

void TruckOBU::TimeGapSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    this->timegap = msg->data;
        std::ostringstream oss;
    oss << "emergency_flag: " << (this->emergency_flag ? "true" : "false") << ", "
        << "caution_mode_lane1: " << (this->caution_mode_lane1 ? "true" : "false") << ", "
        << "lane_change_flag: " << (this->lane_change_flag ? "true" : "false") << ", "
        << "timegap: " << this->timegap;

    v2x_custom->Send(oss.str());
}


void  TruckOBU::GetVel() {
    vel_ = Vehicle_->GetVelocity();
    velocity_ = vel_.x; // m/s
}

void  TruckOBU::PublishV2V() {
    ros2_msg::msg::V2XCAM msg_;
    msg_.header.stamp = this->now();
    msg_.stationid = 0;
    msg_.latitude =  0;
    msg_.longitude = 0;
    msg_.heading = 0;

    msg_.speed = velocity_;

    v2xpublisher_->publish(msg_);
}