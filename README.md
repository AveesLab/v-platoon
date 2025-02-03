# Carla for Truck Platooning
![teaser](https://github.com/user-attachments/assets/ccd94e01-9b32-4b90-bb8e-bcfde2724d71)


# System Requirements
- **Ubuntu 20.04.** CARLA provides support for previous Ubuntu versions up to 16.04. **However** proper compilers are needed for Unreal Engine to work properly.
Dependencies for Ubuntu 18.04 and previous versions are listed separately below. Make sure to install the ones corresponding to your system.
- **130 GB disk space.** Carla will take around 31 GB and Unreal Engine will take around 91 GB so have about 130 GB free to account for both of these plus additional minor software installations.
- **Enough memory space (add swap area).** For better performance, using large (> 16 GB) memory is recommended. Setting swap area is also possible to prevent memory insufficiency.
- **An adequate GPU.** CARLA aims for realistic simulations, so the server needs at least a 6 GB GPU although 8 GB is recommended. A dedicated GPU is highly recommended for machine learning.
- **Two TCP ports and good internet connection.** 2000 and 2001 by default. Make sure that these ports are not blocked by firewalls or any other applications.
- **Python 3.8 ++**

### Before installation, read [Reference](#10-reference) below is highly recommended.

# 1. Install Dependency
```
wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add -
sudo apt-add-repository "deb http://apt.llvm.org/focal/ llvm-toolchain-focal main"
```
```
sudo apt-get update
sudo apt-get install build-essential clang-10 lld-10 g++-7 cmake ninja-build libvulkan1 python python-dev python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-10/bin/clang++ 180 &&
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-10/bin/clang 180
```
```
pip install --user setuptools &&
pip3 install --user -Iv setuptools==47.3.1 &&
pip install --user distro &&
pip3 install --user distro &&
pip install --user wheel &&
pip3 install --user wheel auditwheel
```

# 2. Install Unreal Engine 4.26
-  Clone the content for CARLA's fork of Unreal Engine 4.26 to your local computer:
    ```
    git clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git ~/UnrealEngine_4.26
    ```
- Navigate into the directory where you cloned the repository:
    ```
    cd ~/UnrealEngine_4.26
    ```
- Make the build. This may take an hour or two depending on your system.
    ```
    ./Setup.sh && ./GenerateProjectFiles.sh && make
    ```
- Open the Editor to check that Unreal Engine has been installed properly.
    ```
    cd ~/UnrealEngine_4.26/Engine/Binaries/Linux && ./UE4Editor
    ```

# 3. Install Carla 0.9.15
- Install Dependency
    ```
    sudo apt install --reinstall build-essential clang-8 lld-8 g++-7 cmake vim ninja-build libvulkan1 python-dev python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf wget libtool rsync libxml2-dev
    ```
- Clone Calra Simulator
    ```
    cd
    git clone https://github.com/carla-simulator/carla
    cd ~/carla
    ```
- Get assets
    Refer [this issue](https://github.com/carla-simulator/carla/issues/7196#issuecomment-1976496048) in case of error in **CONTENT_LINK**
    ```
    ./Update.sh
    ```
- Set Unreal Engine Environment Variables
    ```
    vi ~/.bashrc
    ```
- Insert this code in last line
    ```
    export UE4_ROOT=~/UnrealEngine_4.26
    ```
- Source Environment Variables
    ```
    source ~/.bashrc
    ```
- Build carla
    ```
    cd ~/carla
    make PythonAPI
    make launch
    ```
- We recommend to wait for shader compile after `make launch` to prevent lack of memory in futher steps

# 4. Import Truck & Trailer models
### Install vehicle models
```
git clone https://github.com/ysrhee6199/CarlaSemiTruckTrailer.git
```
0. Merge **PythonAPI** & **Unreal** in carla directory
1. In ```Content/Carla/Blueprint/Vehicle```, open the ```VehicleFactory``` file.
2. In the **Generate Definitions** tab, double click **Vehicles**.
3. In the **Details panel**, expand the **Default Value** section and add a new element to the vehicles array.
4. Fill in the **Make** and **Model** of your vehicle. For the truck name the **Make**: "DAFxf". And for the trailer name the **Make**: "trailer".
5. Fill in the **Class** value with your ```BP_<vehicle_name>``` file.
6. Optionally, provide a set of recommended colors for the vehicle. Name the "model" an "make" as below in the picture
![image](https://github.com/AveesLab/scale_truck_control_carla/assets/117966644/6f08583c-eff5-4734-87f6-a0dc88671be0)


7. Compile and save.
![image](https://github.com/AveesLab/scale_truck_control_carla/assets/117966644/ed9bc67b-2432-4cfa-ade1-21297d185d00)


### Test the vehicle
Launch CARLA, open a terminal in ```PythonAPI/examples``` and run the following command:
```
python3 manual_controlSemiTrailer.py
```

# 5. Import K-track Map
- Download files in **carla-virtual-platoon/map_package**
- Copy the files (`k-track.fbx`, `k-track.xodr`) to **~/carla/Import** directory
- In **~/carla** 
    ```
    make import
    ```
- After then,
    ```
    make launch
    ```

# 6. Install ROS 2 (Galactic)
- Set locale
    ```
    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings
    ```
- Setup sources
    ```
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    ```
    ```
    sudo apt update && sudo apt install curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
    ```
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
- Install ROS 2 packages
    ```
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-galactic-desktop
    ```
- Colcon install
    ```
    sudo apt install python3-colcon-common-extensions
    ```

# 7. Install & Run bridge
- Create carla library
    ```
    cd ~/carla
    make setup
    make LibCarla
    ```
- Create ROS2 workspace
    ```
    source /opt/ros/galactic/setup.bash
    mkdir -p ~/ros2_ws/src
    ```
- Install packages
    ```
    cd ~/ros2_ws/src
    git clone https://github.com/AveesLab/carla-virtual-platoon.git
    ```
- Set environmental variable
    ```
    vi ~/.bashrc
    ```
- Insert this code in last line
    ```
    export CARLA_ROOT=~/carla
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CARLA_ROOT/PythonAPI/carla/dependencies/lib
    ```
- Source bash configuration
    ```
    source ~/.bashrc
    ```
- Packages build
    ```
    cd ~/ros2_ws
    source /opt/ros/galactic/setup.bash
    colcon build --symlink-install
    source ./install/setup.bash
    ```

# 8. Run
-  Carla launch
    ```
    cd ~/carla
    make launch
    ```

- Run Server
    ![seame(1)](https://github.com/AveesLab/carla-virtual-platoon/assets/83895074/75f873b0-613b-4718-a1cb-8747b4c604a7)

- Run ros2 launch
    ```
    source ~/ros2_ws/install/setup.bash

    # Map = {k-track, Town04_Opt}
    ros2 launch carla-virtual-platoon carla-virtual-platoon.launch.py NumTrucks:=3 Map:=Town04_Opt

    or

    ros2 launch carla-virtual-platoon carla-virtual-platoon.launch.py NumTrucks:=3 Map:=k-track
    ```

# 9. ROS2
### Sensor related parameters
You can change the names and parameters of the sensors(camera,lidar,etc) inside `config/config.yaml`.

#### Published Topics

* **`/truck{n}/front_camera`** ([sensor_msgs/msg/Image])

    Publishes image data captured by the Carla simulator.

* **`/truck{n}/front_lidar`** ([sensor_msgs/msg/PointCloud2])

    Publishes PointCloud data generated by lidar

* **`/truck{n}/velocity`** ([std_msgs/msg/Float32])

    Publishes the current velocity (m/s) of truck{n} every millisecond.
  
#### Subscribed Topics

* **`/truck{n}/steer_control`** ([std_msgs/msg/Float32])

    Subscribes to steering control values (in degrees).

* **`/truck{n}/throttle_control`** ([std_msgs/msg/Float32])

    Subscribes to control values for throttle. If the value is greater than zero, it controls the throttle; if it's less than zero, it applies the brake.

# 10. Reference
- Carla [Linux Build](https://carla.readthedocs.io/en/0.9.15/build_linux/) Official Documentation
- ROS2 Installation ([Galactic](https://docs.ros.org/en/galactic/Installation.html), [Foxy](https://docs.ros.org/en/foxy/Installation.html)) Documentation
