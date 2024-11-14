Original setup guide: https://rebrand.ly/ros/install

NOTE: I recommend opening this guide up within the VirtualBox. Here's the short link for easy access: [rebrand.ly/ros/quick-install](https://rebrand.ly/ros/quick-install)

# Fix sudo access
```bash
su
adduser <username> sudo
exit
```

NOTE: `<username>` = The username that you see when you login (e.g. ros2)
* For example, if your username was `randomrobot3` you would type `adduser randomrobot3 sudo`
  
# Restart system
```bash
shutdown -r 0
```

# Set locale to UTF-8
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

# Update system

```bash
sudo apt update
sudo apt full-upgrade -y
```

# Setup sources

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

# Install ROS 2 & Gazebo

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install ros-humble-ros-gz -y
```

# Configure ROS 2
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

# Verify installation
```bash
ros2 doctor
```