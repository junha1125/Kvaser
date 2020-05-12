# Kvaser
This is for Real Time transmission from ROS Topic Data to the Other PC  
And this is Based on Ros Package in Autoware, Kavser.  
you should Build Autowar_msgs, Autowar_can_msgs First  

### 1. You need to download some packages from 
https://github.com/junha1125/Kvaser/tree/junha1125-English-Commend/Kvaser-0930   

### 2. Move Folders
Move “autoware_can_msgs”,“autoware_msgs” Folders to ~/catkin_ws/src, and Build using $ cd ~/catkin_ws %% catkin_make.
![image](https://user-images.githubusercontent.com/46951365/81674758-ac930200-9488-11ea-908e-b6a6a935caee.png)  

### 3. After Building successfully 
Move “kvaser” Folder to ~/catkin_ws/src, Build in the same way.
using $ cd ~/catkin_ws %% catkin_make.

### 4. Now you have a complete workspace. 
Therefore type commands on the Ubuntu terminal step by step.  
![image](https://user-images.githubusercontent.com/46951365/81674774-b1f04c80-9488-11ea-86ee-464c42ed1a3f.png)
```sh
$ rosrun kvaser can_tx_to_raw
(ps. You had to connect Kvaser Device and check if your ROS is subscribing /can_tx)
$ rosrun kvaser can_Mobileye_Decoder
$ rosrun kvaser GPS_tude_to_XY
```
## Supporting Explanation 
- can_tx_to_raw : this node has a role converting /can_tx(real kvaser data) into /can_raw(important data among these).
- can_Mobileye_Decoder : this node subscribe /can_raw topics and decode available information like Obstacle Number, Obstacle Position, Lane Distance from my vehicle etc.. thereafter Publish /Mobileye_new topic.
- GPS_tude_to_XY : this node subscribe GPS data(latitude/longitude), CAN data(Mobileye/NiroCan), Lidar data(Point cloud data). Calculate using the data so that we get my vehicle&obstacle information and XY-coordinate which we can utilize on Uc-win Road.



 




