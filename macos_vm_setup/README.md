# MacOS VM Setup
This guide provides step-by-step instructions for setting up ROS 2 on a Ubuntu 22.04 (or newer version) VM on a MacOS-based computer.

## Ubuntu VM Setup
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
1. Set locale:
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
2. 
3. no
4. maybe
5. so
