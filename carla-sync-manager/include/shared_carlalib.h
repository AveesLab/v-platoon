#pragma once

#include <iostream>
#include <cmath>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <fstream>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/ServerSideSensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/client/ActorList.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <rclcpp/rclcpp.hpp>
#include <carla/rpc/VehicleControl.h>
#include <carla/rpc/EpisodeSettings.h>
#include <carla/client/WorldSnapshot.h>

namespace cc = carla::client;
using ActorId = carla::ActorId;

using namespace carla::geom;

#define EXPECT_TRUE(pred) if (!(pred)) { throw std::runtime_error(#pred); }