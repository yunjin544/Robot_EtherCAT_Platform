# Robot_EtherCAT_Platform
## If You use this project , You must install Xenomai kernel!!

Reference :   
https://github.com/saga0619/rtnet_soem.git(start_rtnet.sh & red_test)  
https://github.com/OpenEtherCATsociety/SOEM.git(red_test)  
https://source.denx.de/Xenomai/xenomai/-/wikis/home(Xenomai API)  

Requirement : Xenomai : 3.2.1   
              Linux   : 5.4.180  
              Ubuntu  : 20.04  
              SOEM    : 1.4.0  
              RT-net  : NIC Card ( Test done! : I210 & I340 )  // WGI210AT & E1G44HT  
              ROS1    : Noetic  
              
How to Run(Example)
```
cd {your_ROS_ws}
```
```
git clone https://github.com/yunjin544/Robot_EtherCAT_Platform.git
```
```
cd {your_ROS_ws} && catkin_make
```
```
roscd && cd lib/robot_ecat_master
```
```
sudo ./RT_Ecat_master rteth0 rteth1
```
              
To-do
1. Debugging Distributed Clock Mode     


