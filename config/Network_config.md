# ROS 2 Humble – Lab Network Setup with CycloneDDS Unicast

## 1. Network topology and ROS 2 setup

This document describes the network setup of a teaching lab with a mobile robot and a control PC, both running ROS 2 Humble and CycloneDDS.

### 1.1 Physical network

- **WiFi router (mobile router / hotspot)**

  - Provides DHCP and fixed IP addresses in the `192.168.1.x` network.
- **Robot 1**

  - Hardware: Raspberry Pi 4
  - OS: Ubuntu Server 22.04
  - ROS: ROS 2 Humble
  - IP address (fixed): `192.168.1.14`
- **Control PC**

  - Hardware: PC with Ubuntu 22.04
  - ROS: ROS 2 Humble
  - IP address (fixed): `192.168.1.15`

Both machines are connected to the same WiFi router and are in the same Layer-2 network.

You can verify connectivity with:

```bash
ping 192.168.1.14    # from the PC to the robot
ping 192.168.1.15    # from the robot to the PC
````
### 1.2 ROS 2 environment (.bashrc)
Both the robot and the PC use a similar .bashrc configuration (user: student):

````bash
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /home/student/Desktop/ROS2_rUBot_mecanum_ws/install/setup.bash
cd /home/student/Desktop/ROS2_rUBot_mecanum_ws

export ROS_DOMAIN_ID=1               # group/domain ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MODEL_PATH=/home/student/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
export QT_QPA_PLATFORM=xcb           # Best for RVIZ2
export ROS_LOCALHOST_ONLY=0          # allow communication with other machines
````

With this configuration, both machines:

- Share the same ROS domain (ROS_DOMAIN_ID=1).

- Use CycloneDDS as the RMW implementation.

- Are allowed to communicate over the network (ROS_LOCALHOST_ONLY=0).


## 2. CycloneDDS peer-to-peer unicast configuration
To make the system robust and independent of multicast quirks and startup order, we explicitly configure CycloneDDS to use unicast peer discovery between the PC and the robot.

With this configuration:

- Each machine uses its own fixed IP address for DDS.

- Each machine is given the other machine as a peer (explicit unicast address).

- Multicast can still be allowed, but discovery no longer depends on it.

We will:

- Store configuration files inside the ROS 2 workspace:
    ````text
    /home/student/Desktop/ROS2_rUBot_mecanum_ws/config/cyclonedds_pc.xml
    /home/ubuntu/ROS2_rUBot_mecanum_ws/config/cyclonedds_robot.xml
    ````
- CycloneDDS configuration file for the PC `cyclonedds_pc.xml` with the following content:
    ````xml
    <?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
    <Domain id="any">
        <General>
        <!-- Bind CycloneDDS to the PC's IP address -->
        <NetworkInterfaceAddress>192.168.1.15</NetworkInterfaceAddress>
        <AllowMulticast>true</AllowMulticast>
        </General>
        <Discovery>
        <!-- Explicitly add the robot as a unicast peer -->
        <Peers>
            <Peer address="192.168.1.14"/>
        </Peers>
        </Discovery>
    </Domain>
    </CycloneDDS>
    ````
    Notes:

    NetworkInterfaceAddress: Can be the IP (192.168.1.15) or an interface name (e.g. wlp2s0). This ensures DDS uses the correct network interface.

    Peers: Here we tell the PC to directly contact the robot at 192.168.1.14 via unicast.

- CycloneDDS configuration file for the robot. `cyclonedds_robot.xml` with the following content:
    ````xml
    <?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
    <Domain id="any">
        <General>
        <!-- Bind CycloneDDS to the robot's IP address -->
        <NetworkInterfaceAddress>192.168.1.14</NetworkInterfaceAddress>
        <AllowMulticast>true</AllowMulticast>
        </General>
        <Discovery>
        <!-- Explicitly add the PC as a unicast peer -->
        <Peers>
            <Peer address="192.168.1.15"/>
        </Peers>
        </Discovery>
    </Domain>
    </CycloneDDS>
    ````
    Again:

    NetworkInterfaceAddress ensures the robot uses the WiFi network interface associated with 192.168.1.14.

    Peers tells the robot to directly talk to the PC at 192.168.1.15.

- Update `.bashrc` to use these configs: We now need to tell CycloneDDS to load these XML files using the CYCLONEDDS_URI environment variable.

    - On the PC: Edit the `.bashrc` for user student and verify it contains the following lines:

        ````bash
        source /opt/ros/humble/setup.bash
        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
        source /home/student/Desktop/ROS2_rUBot_mecanum_ws/install/setup.bash
        cd /home/student/Desktop/ROS2_rUBot_mecanum_ws
        export GAZEBO_MODEL_PATH=/home/student/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
        export QT_QPA_PLATFORM=xcb           # Best for RVIZ2
        export ROS_DOMAIN_ID=1               # group/domain ID
        export ROS_LOCALHOST_ONLY=0          # allow communication with other machines
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI=file:///home/student/Desktop/ROS2_rUBot_mecanum_ws/config/cyclonedds_pc.xml
        ````
    - On the robot:

        ````bash
        source /opt/ros/humble/setup.bash
        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
        source /home/ubuntu/ROS2_rUBot_mecanum_ws/install/setup.bash
        cd /home/ubuntu/ROS2_rUBot_mecanum_ws
        export GAZEBO_MODEL_PATH=/home/ubuntu/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
        export QT_QPA_PLATFORM=xcb           # Best for RVIZ2
        export ROS_DOMAIN_ID=1               # group/domain ID
        export ROS_LOCALHOST_ONLY=0          # allow communication with other machines
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI=file:///home/ubuntu/ROS2_rUBot_mecanum_ws/config/cyclonedds_robot.xml
        ````
        > You will have to connect to the robot via SSH to edit its `.bashrc`.
        > Sometimes you will have to regenerate the keys to connect to the raspberry pi. Type on a `cmd`terminal on the PC:
        ````bash
        ssh-keygen -R rUBot_IP_ADDRESS
        ````

## 3. How to use and test
- Start the robot:

- Boot the PC.

    - Log in as student.

    - Open a terminal:

        ````bash
        source ~/.bashrc
        ros2 topic list
        ````
        >You should now see the robot’s topics even if the PC was powered on before or after the robot. The order should no longer matter.

- If something still looks wrong:

    - Restart the ROS 2 daemon on the PC:

        ````bash
        ros2 daemon stop
        ros2 topic list   # this will implicitly restart the daemon
        ````
        >If the unicast configuration is correct and the IP addresses are reachable, this setup should be very robust against router multicast issues and startup order.

## 4. Adapting this setup to other robots/PCs
If you replicate this setup for multiple robots and PCs in the lab:

Each robot and PC must have:

- A unique IP address (e.g., 192.168.1.14, 192.168.1.24, etc.).

- Matching ROS_DOMAIN_ID for each robot–PC pair or group.

- For each pair:

    - Create a dedicated CycloneDDS XML file on each side.

    - Set NetworkInterfaceAddress to each machine’s IP.

    - Add the peer(s) in the <Peers> section with the corresponding IPs.


This approach keeps the lab configuration clear, reproducible, and independent of the router’s multicast behavior.

## 5. Time Synchronization

Make sure PC and Robot (Raspberry Pi) have the same system time using built-in NTP.


Matching time helps with:

- TF transforms

- Logging

- Sensor fusion

- Debugging timestamped data

(ROS 2 discovery does NOT depend on clock sync, but TF and logs do.)

### On the PC (Ubuntu 22) and robot (Raspberry Pi Ubuntu 22):

- Check current time sync status
    ````bash
    timedatectl status
    ````

- It should show:

    - System clock synchronized: yes

    - NTP service: active

    - Time zone: Europe/Madrid (or your timezone)

- If not, enable NTP:
    ````bash
    # Enable automatic NTP time synchronization
    sudo timedatectl set-ntp true
    # Set your timezone
    sudo timedatectl set-timezone Europe/Madrid
    # Check again
    timedatectl status
    ````


### On Docker containers

No extra configuration needed.

Containers inherit the host’s system clock, so:

- If the PC clock is correct → the container clock is correct.

- Mounting /etc/localtime is only for correct timezone display, not time sync.

    ````yaml
    volumes:
    - /etc/localtime:/etc/localtime:ro   # timezone only (not time sync)
    ````

# Install ROS2 environment on GEI lab computers

We will use a proper docker-compose setup to run ROS2 Humble with CycloneDDS on the lab computers.

First of all, a proper Docker Image has been created with the custom configuration on Dockerfile and uploades to my Docker Hub account.
- In the folder where your Dockerfile is:
    ````bash
    docker build -t ros2-humble-biorobub-pc:latest .
    ````
- Tag the image for Docker Hub
    ````bash
    docker tag ros2-humble-biorobub-pc:latest manelpuig/ros2-humble-biorobub-pc:latest
    ````
- Login to Docker Hub from terminal and follow instructions to autenticate
    ````bash
    docker login
    ````
- Push the image to Docker Hub
    ````bash
    docker push manelpuig/ros2-humble-biorobub-pc:latest
    ````
- Once the image is pushed properly to my Dockerhub account, I have to delete the local image tag:
    ````bash
    docker rmi ros2-humble-biorobub-pc:latest
    ````
Students in the lab they only need to run:
- Unzip the `ros2-humble-biorobub.zip` file in a /home/user/Desktop/rob folder on Linux PC

- review the:
    - `ROS_DOMAIN_ID` variable in the `docker-compose.yml` file and `Dockerfile` to match your lab setup.
    - IPs in `cyclonedds_pc.xml` files to match your lab setup.
- Open a terminal in the `/home/user/Desktop/rob/ros2-humble-biorobub` folder and run:
    ````bash
    xhost +local:root            # permet X11 per al container
    cd ~/Desktop/ros2-humble-biorobub
    docker-compose up -d
    docker exec -it pc_humble bash
    ros2 topic list
    ````

## Previous installation

Becauseof the PCs are frozen, we have first to install:
- Install Docker and docker-compose (or docker compose plugin):
    ````bash
    sudo apt install docker.io docker-compose
    ````

- Add students to the docker group so they can run Docker without sudo:
    ````shell
    sudo usermod -aG docker student
    ````
    (Then they log out and log back in.)

- Install Vscode from apt:
    ````bash
    sudo snap install --classic code
    ````

If this is done once in the base frozen image, students won’t need to reinstall it each day.
