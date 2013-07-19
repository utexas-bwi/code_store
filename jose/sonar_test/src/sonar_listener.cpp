#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sonar_test/SegbotSensorsStatus.h"
#include <sensor_msgs/PointCloud2.h>

#define WIDTH 5
#define HEIGHT 1
#define BEGENDIAN false
#define DENSE true
#define FIELDS 3
#define ANGLE 25
#define RADIAN 0.0174532925
#define BASE_DISTANCE 0.125
#define SENSORS 5
#define Z 0

ros::Publisher pub;

void chatterCallback(const sonar_test::SegbotSensorsStatus::ConstPtr& msg)
{
	ROS_INFO("I HEARD [%f %f %f %f %f]",msg->sonar1,msg->sonar2,msg->sonar3,msg->sonar4,msg->sonar5);		

	sensor_msgs::PointCloud2 point_cloud_msg;
	
	//make an array to make it easier. Include the distance from base
	float sensor_data[SENSORS];

	//copy the raw data in meters
	sensor_data[0]=msg->sonar1;
	sensor_data[1]=msg->sonar2;
	sensor_data[2]=msg->sonar3;
	sensor_data[3]=msg->sonar4;
	sensor_data[4]=msg->sonar5;

	//check if any of them is 0. If so, assume that there is nothing there. Also add the base distance
	for(int i = 0; i<SENSORS;i++){
		if(sensor_data[i] == 0) sensor_data[i] = std::numeric_limits<float>::infinity();
		else sensor_data[i] = sensor_data[i] + BASE_DISTANCE;
	}



    point_cloud_msg.header.frame_id = "/base_footprint";
    point_cloud_msg.header.stamp = ros::Time::now();	
	
	//set the constant parameters of the pointcloud2 msg
	point_cloud_msg.height = HEIGHT;
	point_cloud_msg.width = WIDTH;
	point_cloud_msg.is_bigendian = BEGENDIAN;
	point_cloud_msg.is_dense = DENSE;
	point_cloud_msg.point_step = FIELDS*sizeof(float);

	point_cloud_msg.row_step = point_cloud_msg.point_step*WIDTH;
	point_cloud_msg.fields.resize(FIELDS);	
	point_cloud_msg.data.resize(WIDTH*point_cloud_msg.point_step);		

	
	point_cloud_msg.fields[0].name = "x";
	point_cloud_msg.fields[1].name = "y";
	point_cloud_msg.fields[2].name = "z";

	

	//offsets are all 4 because a float is 4 bytes
	int offset = 0;
	for(size_t d = 0; d<point_cloud_msg.fields.size();++d,offset+=4){
		point_cloud_msg.fields[d].count = 1;
		point_cloud_msg.fields[d].offset = offset;
		point_cloud_msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
	}


	//update the dots in data

	//angle in radians:
	float angle = ANGLE*RADIAN;
	float first_angle = -1*(SENSORS-1)/2 * angle;


	for(int sensor = 0; sensor<SENSORS;sensor++){
		int ps = point_cloud_msg.point_step;
		float x = sensor_data[sensor]*sin(first_angle+sensor*angle);
		float y = sensor_data[sensor]*cos(first_angle+sensor*angle);
		float z = Z;

		//x
		memcpy(&point_cloud_msg.data[sensor*ps + point_cloud_msg.fields[0].offset],&x,sizeof(float));
		//y
		memcpy(&point_cloud_msg.data[sensor*ps + point_cloud_msg.fields[1].offset],&y,sizeof(float));
		//z, always the same
		memcpy(&point_cloud_msg.data[sensor*ps + point_cloud_msg.fields[2].offset],&z,sizeof(float));
	
	}

	pub.publish(point_cloud_msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "sonar_listener");
	ros::NodeHandle n;
	pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud",10);
	ros::Subscriber sub = n.subscribe("sonarReadings",1000,chatterCallback);
	ros::spin();


		

	return 0;
}
