# MacOS VM Setup
This guide provides step-by-step instructions for setting up ROS 2 on a Ubuntu 22.04 (or newer version) VM on a MacOS-based computer.

## Ubuntu VM Setup
Below are detailed guidelines for settingn up an Ubuntu VM on your Macbook. Before proceeding, it is recommended to visit https://www.parallels.com/products/desktop/resources/ to check whether or not your computer meets the minimum system requirements.

1. Go to https://www.parallels.com/products/desktop/welcome-trial/ to download the Parallels Desktop 18 for Mac. The Parallels Desktop is an application for running Windows on Mac without rebooting.
2. Open the downloaded image and double-click the install box.
3. Omit the "Get Windows 11 from Microsoft" page if you see one. Click "Skip" to navigate to "Create New" page. Under "Free Systems", click "Download Ubuntu Linux" and then click "Continue". ![Step 3](/macos_vm_setup/resources/images/pic1.jpg)
4. Check the unpacked size (roughly 6 GB) and ensure the sufficiency of your computer's storage. After that, click "Download" and wait for the download to finish.
5. After the download completes, you should see a Ubuntu interface as shown below. Click the "Parallels" icon in the middle to set up a password for your Ubuntu Linux. Retype the password to confirm. ![Step 5](/macos_vm_setup/resources/images/pic2.jpg)
6. Now you are in. Enter your passsword once again in the box on the upper left corner and click "OK". It initializes the Parallels Tools Installation. ![Step 6](/macos_vm_setup/resources/images/pic3.jpg)
7. While the installation is ongoing, walk through the settings regarding legal notice and privacy until a page says "You're ready to go!". Click "Done". ![Step 7](/macos_vm_setup/resources/images/pic4.jpg)
8. As the installation is finished, click "Restart" to restart (of course) Linux to activate some newly installed features. ![Step 8](/macos_vm_setup/resources/images/pic5.jpg)
9. Once again, click the icon in the middle and type your password. ![Step 9](/macos_vm_setup/resources/images/pic6.jpg)
10. Now you have successfully set up your Ubuntu VM and can start to do stuff on it. ![Step 10](/macos_vm_setup/resources/images/pic7.jpg)

## ROS 2 installation
The following instructions on ROS 2 installation are sourced from https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html. Note that this project's controller works well on "humble" distribution of ROS 2. While other distributions like "foxy" and "iron" might seem compatible, they could introduce unforeseen errors.

1. Set locale:
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
2. Ensure that the Ubuntu Universe repository is enabled:
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```
3. Add the ROS 2 GPG key with apt:
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
4. Add the repository to your sources list:
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
5. Install common packages:
```
sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
```
6. Install packages specifically for Ubuntu 22.04:
```
sudo apt install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures
```
7. Create a workspace and clone all repos:
```
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
```
8. Check if your system is up to date:
```
sudo apt upgrade
```
9. Install necessary dependencies:
```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
```
10. Build the code in the workspace:
```
cd ~/ros2_humble/
colcon build --symlink-install
```
11. Source the setup script:
```
. ~/ros2_humble/install/local_setup.bash
```
12. 
13. 

