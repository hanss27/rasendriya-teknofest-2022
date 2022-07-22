# Rasendriya
!['Rasendriya Logo'](https://github.com/rizkymille/rasendriya-teknofest-2021/blob/main/docs/logo.png)  
Rasendriya is a lightweight fixed wing UAV from AUAV UI for Tübitak International Unmanned Aerial Vehicle (UAV) Teknofest Competition in Afyonkarahisar, Turkey.

For Teknofest 2022, Rasendriya has two missions: Flying in 8-figure track, and flying in oval track for 3 laps while dropping two payloads in a designated dropzone. The designated dropzone must be found autonomously by using computer vision system.

## Setup
### Prerequisites
- ROS Noetic Ninjemys

#### Companion Computer
- Ubuntu Server with Lubuntu 20.04 LTS for Raspberry Pi 4B

#### Libraries
- OpenCV 4.2.0

#### ROS Packages
- MAVROS

### Headless Setup

#### vnc Remote Desktop  
Install vnc server  
`sudo apt update`  
`sudo apt install x11vnc`  
Run vnc server  
`sudo x11vnc -display :0 -auth guess`  

## Launching Headless
### SSH  
Connect your ground control station to vehicle SBC via ssh  
`ssh device_name@ip_address`  
For odroid:  
`ssh ubuntu@10.107.213.213`  

### ROS
`roslaunch rasendriya mission.launch`  

## ROS configure
Set devel path in bashrc first  
`sudo nano ~/.bashrc`  
then add this in the last line  
`source /home/pi/catkin_ws/devel/setup.bash`
