#include "shared_carlalib.h"
#include "CarlaLocation.hpp"
#include "Camera.hpp"
#include "Lidar.hpp"
#include "Radar.hpp"
#include "TruckControl.hpp"
#include "TruckStatus.hpp"
#include "Spectator.hpp"
#include "TruckOBU.hpp"
#include <thread>
#include <unistd.h>
#include <sys/wait.h>
#include <csignal>  // for std::signal

std::string host = "localhost";
uint16_t port = 2000u;
cc::Client *client;
cc::World *world;
carla::SharedPtr<carla::client::BlueprintLibrary> blueprint_library;
boost::shared_ptr<carla::client::Vehicle> vehicle_trailer;
boost::shared_ptr<carla::client::Vehicle> vehicle_truck;

/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}
void signal_handler(int signal) {
    if (signal == SIGINT) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ctrl+C caught, shutting down...");
        rclcpp::shutdown();
    }
}

carla::geom::Location GetTruckLocation(int truck_num, std::string map_name) {
    float x = 0.0f , y = 0.0f ,z =0.0f;
    if (truckLocations.find(map_name) != truckLocations.end() && truckLocations[map_name].find(truck_num) != truckLocations[map_name].end()) {
        x = truckLocations[map_name][truck_num][0]+5.0f;
        y = truckLocations[map_name][truck_num][1];
        z = truckLocations[map_name][truck_num][2];
    } else {
        throw std::runtime_error("Invalid map name or truck index for location.");
    }

    return carla::geom::Location(x,y,z);
}

carla::geom::Location GetTrailerLocation(int truck_num, std::string map_name) {
    float x = 0.0f , y = 0.0f ,z =0.0f;
    if (truckLocations.find(map_name) != truckLocations.end() && truckLocations[map_name].find(truck_num) != truckLocations[map_name].end()) {
        x = truckLocations[map_name][truck_num][0];
        y = truckLocations[map_name][truck_num][1];
        z = truckLocations[map_name][truck_num][2];
    } else {
        throw std::runtime_error("Invalid map name or truck index for location.");
    }

    return carla::geom::Location(x,y,z);
}

carla::geom::Rotation GetTruckRotation(int truck_num, std::string map_name) {
    float pitch = 0.0f , yaw = 0.0f ,roll =0.0f;
    if (truckRotations.find(map_name) != truckRotations.end() && truckRotations[map_name].find(truck_num) != truckRotations[map_name].end()) {
        pitch = truckRotations[map_name][truck_num][0];
        yaw = truckRotations[map_name][truck_num][1];
        roll = truckRotations[map_name][truck_num][2];
    } else {
        throw std::runtime_error("Invalid map name or truck index for rotation.");
    }
    return carla::geom::Rotation(pitch,yaw,roll);
}

void register_to_manager(int trucknum_) {
    auto node = rclcpp::Node::make_shared("registration_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::Int32>("/registration", 10);
    sleep(1);
    auto message = std_msgs::msg::Int32();
    message.data = trucknum_;
    publisher->publish(message);
}

void sync_request(int trucknum_) {
    auto node = rclcpp::Node::make_shared("sync_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::Int32>("/sync", 10);
    sleep(1);
    auto message = std_msgs::msg::Int32();
    message.data = trucknum_;
    publisher->publish(message);
}

void notify_numtrucks_to_manager(int numtrucks_) {
    auto node = rclcpp::Node::make_shared("numtrucks_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::Int32>("/numtruckss", 10);
    sleep(1);
    auto message = std_msgs::msg::Int32();
    message.data = numtrucks_;
    RCLCPP_INFO(node->get_logger(), "Publishing: '%d'", message.data);
    publisher->publish(message);
}


void connect_to_carla(int truck_num) {
    //Connecting to CARLA server;
    std::cerr << truck_num << " Truck connecting to CARLA server" << '\n';
    client = new cc::Client(host, port);
    client->SetTimeout(40s);
    std::cerr << truck_num << " Truck connected to CARLA server" << '\n';
    world = new cc::World(client->GetWorld());
    blueprint_library = world->GetBlueprintLibrary();
    if(truck_num ==5) {
    // Get world
    carla::rpc::WeatherParameters weather = world->GetWeather();
    /*
    weather.cloudiness = 5.0f;
    weather.dust_storm = 0.0f;
    weather.fog_density = 0.0f;
    weather.fog_distance = 10.0f;
    weather.fog_falloff = -1.0f;
    weather.mie_scattering_scale = 45.0f;
    weather.precipitation =2.0f;
    weather.precipitation_deposits =0.75f;
    weather.rayleigh_scattering_scale = 0.1f;
    weather.scattering_intensity =0.0f;
    */
    weather.sun_altitude_angle =115.0f;
    weather.sun_azimuth_angle =10.0f;
   // weather.wetness=0.0331f;
  //  weather.wind_intensity=0.0f;
    world->SetWeather(weather);
    }
}

void generate_truck(int truck_num, std::string map_name) {
    std::mt19937_64 rng((std::random_device())());  

    //Get a Trailer
    auto trailer = blueprint_library->Filter("trailer");
    auto blueprint_trailer = RandomChoice(*trailer, rng);
    blueprint_trailer.SetAttribute("role_name", "trailer" + std::to_string(truck_num));

    // Get a Truck blueprint.
    auto truck = blueprint_library->Filter("dafxf");
    auto blueprint_truck = RandomChoice(*truck, rng);  
    blueprint_truck.SetAttribute("role_name", "truck" + std::to_string(truck_num));

    carla::geom::Location TruckLocation = GetTruckLocation(truck_num,map_name);
    carla::geom::Rotation TruckRotation = GetTruckRotation(truck_num,map_name);
    carla::geom::Transform Trucktransform(TruckLocation,TruckRotation);

    carla::geom::Location TrailerLocation = GetTrailerLocation(truck_num,map_name);
    carla::geom::Rotation TrailerRotation = GetTruckRotation(truck_num,map_name);
    carla::geom::Transform Trailertransform(TrailerLocation,TrailerRotation);
    

    // Spawn the truck
    auto actor_truck = world->SpawnActor(blueprint_truck, Trucktransform);
    std::cout << "Spawned " << actor_truck->GetDisplayId() << '\n';
    vehicle_truck = boost::static_pointer_cast<cc::Vehicle>(actor_truck);
    // Spawn the trailer
    auto actor_trailer = world->SpawnActor(blueprint_trailer, Trailertransform);
    std::cout << "Spawned " << actor_trailer->GetDisplayId() << '\n';
    vehicle_trailer = boost::static_pointer_cast<cc::Vehicle>(actor_trailer);   



    register_to_manager(truck_num);
    sleep(5);
    if(truck_num == 0) {
        rclcpp::executors::MultiThreadedExecutor executor; 
        auto node_obu = std::make_shared<TruckOBU>(vehicle_truck,actor_truck,truck_num);
        auto node_status = std::make_shared<TruckStatusPublisher>(vehicle_truck,actor_truck,truck_num);
        auto node_radar = std::make_shared<RadarPublisher>(actor_truck);
        auto node_lidar = std::make_shared<LidarPublisher>(actor_truck);
        auto node_control = std::make_shared<TruckControl>(vehicle_truck,truck_num);
        auto node_camera = std::make_shared<CameraPublisher>(actor_truck,truck_num);
        //auto node_spectator = std::make_shared<Spectator>(actor_truck);
        executor.add_node(node_obu);
        executor.add_node(node_status);
        executor.add_node(node_radar);
        executor.add_node(node_lidar);
        //executor.add_node(node_spectator);
        executor.add_node(node_control);
        sleep(1);
        executor.add_node(node_camera);
        register_to_manager(truck_num);
        executor.spin();  
    }
    else
    {
        rclcpp::executors::MultiThreadedExecutor executor; 
        auto node_obu = std::make_shared<TruckOBU>(vehicle_truck,actor_truck,truck_num);
        auto node_status = std::make_shared<TruckStatusPublisher>(vehicle_truck,actor_truck,truck_num);
        auto node_radar = std::make_shared<RadarPublisher>(actor_truck);
        auto node_lidar = std::make_shared<LidarPublisher>(actor_truck);
        auto node_control = std::make_shared<TruckControl>(vehicle_truck,truck_num);
        auto node_camera = std::make_shared<CameraPublisher>(actor_truck,truck_num);
        //auto node_spectator = std::make_shared<Spectator>(actor_truck);
        executor.add_node(node_obu);
        executor.add_node(node_status);
        executor.add_node(node_radar);
        executor.add_node(node_lidar);
        //executor.add_node(node_spectator);
        executor.add_node(node_control);
        sleep(1);
        executor.add_node(node_camera);
        register_to_manager(truck_num);
        executor.spin();  
    
    } 



}


int main(int argc, char *argv[]) {
    try {
        rclcpp::init(argc, argv);

        //std::signal(SIGINT, signal_handler);  // Register signal handler

        int truck_num = 1; // Default value
        std::string map_name = "IHP"; //Default map
        int numtrucks = 1;

        std::string prefix_truck_id("--truck_id=");
        std::string prefix_map_name("--map=");
        std::string prefix_numtrucks("--numtrucks=");
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg.find(prefix_truck_id) == 0) {
                truck_num = std::atoi(arg.substr(prefix_truck_id.length()).c_str());
            }
            else if (arg.find(prefix_map_name) == 0) {
                map_name = arg.substr(prefix_map_name.length());                        
            }
            else if (arg.find(prefix_numtrucks) == 0) {
                numtrucks = std::atoi(arg.substr(prefix_numtrucks.length()).c_str());
            }
        }

        if(truck_num == 0 ) {
            std::cout << "Number of trucks : " <<numtrucks << std::endl;
            std::cout << "Truck Number : " << truck_num << std::endl;
            std::cout << "Map Name : " << map_name << std::endl;            
            notify_numtrucks_to_manager(numtrucks);  
        }
        if(truck_num > 0) sleep(1);
        connect_to_carla(truck_num);
        generate_truck(truck_num,map_name);  
        vehicle_truck->Destroy();
        vehicle_trailer->Destroy(); 
        rclcpp::shutdown();
    } 
    catch (const std::exception& e) {
        std::cerr << "Unhandled Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
