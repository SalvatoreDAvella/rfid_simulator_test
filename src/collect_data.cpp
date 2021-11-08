#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <rfid_sensor/TagArray.h>
#include <rfid_sensor/Tag.h>

std::vector<std::ofstream> value_files;
int count = 0;

void rfid_callback(const rfid_sensor::TagArray::ConstPtr &data, int index) {

	for (int i = 0; i < data->ntags; i++) 
	{
		value_files[index] << data->header.stamp << "," << data->antennaPose.position.x << "," << data->antennaPose.position.y << "," << data->antennaPose.position.z << "," << 
						data->antennaPose.orientation.x << "," << data->antennaPose.orientation.y << "," << data->antennaPose.orientation.z << "," << data->antennaPose.orientation.w
						<< "," << data->tags[i].name << "," << data->tags[i].dist << "," << data->tags[i].phi << "," << data->tags[i].rssi << std::endl;
	}
}

int main(int argc, char** argv) {
	if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " /gazebo/antenna1_robot/data" << std::endl;
        return 1;
    }

	ros::init(argc, argv, "collect_data");
	ros::NodeHandle nh;
	ros::Rate rate(30);

	std::vector<ros::Subscriber> subscribers;

	for (int i = 0; i < argc-1; i++)
	{
		ros::Subscriber sub;
		subscribers.push_back(sub);
		std::ofstream file;
		std::string input = argv[i+1];
		std::string app = input.substr(input.find_first_of("/") + 1);
		int first = app.find_first_of("/");
		int last = app.find_last_of("/");
		std::string antenna_name = app.substr(first + 1, last-first-1);
		file.open(antenna_name + ".csv");
		value_files.push_back(std::move(file));
		value_files[i] << "time" << "," << "posx" << "," << "posy" << "," << "posz" << "," << 
						"q1" << "," << "q2" << "," << "q3" << "," << "q4"
						<< "," << "name" << "," << "dist" << "," << "fase" << "," << "rssi" << std::endl;
	}

	ros::AsyncSpinner spinner(1);
	spinner.start();

	for (int i = 0; i < argc-1; i++)
		subscribers[i] = nh.subscribe<rfid_sensor::TagArray>(argv[i+1], 10, boost::bind(rfid_callback, _1, i));

	while (ros::ok()) {
		rate.sleep();
	}

	for (int i = 0; i < argc-1; i++)
		value_files[i].close();

	spinner.stop();
}