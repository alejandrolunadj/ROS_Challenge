# ROS Challenge

Project files for the ROS challenge proposed by Ekumen recruiters.


```
The project addresses the following ROS concepts: Nodes, Topics,
Messages, Services and Actions through implementations in C++, 
RQT Python and HTML/JavaScript using official ROS tools, 
libraries and APIs.
```

## Documentation

All related documentation can be found in the folder "docs" as follows:

Chanllenge.pdf			ROS challenge proposed docuemnt.
ROS_install.txt			Guide to install ROS and his dependencies
Project_build.txt		Guide to download the sourcecode and build the project.
Project_run.txt			Quick guide to run each challenge point.
ROS_commands.txt		Quick guide to the most used commands in ROS. 
ROS_concepts.pdf		Explains the ROS concepts needed for the challenge.
ROS_project.pdf			Explains how the challenge was approached and shows each 
                        point with details.

	

## Requirements

`Ubuntu 18.04`, `ROS Melodic`, `Python`, `RQT`, `QT5`, `ROS bridge`


## How to install ROS and dependencies

Open the terminal and execute the following commands:


	$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

	$ sudo apt install curl

	$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

	$ sudo apt update

	$ sudo apt install ros-melodic-desktop-full

	$ sudo apt install ros-melodic-vrpn-client-ros

	$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

	$ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

	$ sudo rosdep init

	$ rosdep update

	$ sudo apt install ros-melodic-rqt
	
	$ sudo apt install ros-melodic-rqt-common-plugins

	$ sudo apt install qt5-default

	$ sudo apt install ros-melodic-rosbridge-suite


## How to build the project


First we must create our ROS project workspace and clone the sourcecode from the repository.

Open a new terminal (this is important) and execute the following commands:

	$ cd ~

	$ git clone https://github.com/alejandrolunadj/ROS_Challenge ros_ws

	$ cd ros_ws
	
	
Now we will compile for the fist time using the following command:
	
	$ catkin_make

From now to compile we must do the followinng commands

	$ cd ~/ros_ws

	$ catkin_make


## How to run


For run each demo, you must first open a new terminal and execute:

	$ roscore
	
	
	And then in another new terminal execute:
	
	$ rosrun turtlesim turtlesim_node
	
	
	Finally in another new terminal run the any project demo, for example:
	
	$rosrun turtlesim turtle_teleop_key 
	
	
To run the challenge points, please follow the instructions described
in the file: Project_run.txt	


	
	
		
	
	
	
	
	
	
	
	
	
	
	
	
	



