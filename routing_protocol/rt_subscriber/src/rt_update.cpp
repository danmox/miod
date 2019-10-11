#include <ros/ros.h>
#include <routing_msgs/PRTableEntry.h>
#include <routing_msgs/ProbGateway.h>
#include <routing_msgs/NetworkUpdate.h>
#include <fstream>
#include <vector>
#include <iostream>


// Import all the necessary ROS libraries and import the Int32 message from the std_msgs package

int main(int argc, char** argv) {

    ros::init(argc, argv, "rt_update"); // Initiate a Node named 'rt_update'


    routing_msgs::NetworkUpdate rt_u;
    routing_msgs::PRTableEntry rt_e;
    routing_msgs::ProbGateway rt_g;

    //////////
    // reading rt data from test file
    std::ifstream myFile;
    std::vector<std::string> vec;
    std::string value;
    std::string line;
    myFile.open("/home/mishanya/PycharmProjects/untitled/routing_tables_upd.txt", std::ios::app); 
    std::cout << "Init complete. Opening the file"<<std::endl;
    if (myFile.is_open()){
        std::cout << "File is open."<<std::endl;
        while (std::getline(myFile, line)){
		//std::cout << "line is " << line <<std::endl;
		std::istringstream ss(line);
		int i=0;
		while (getline(ss,value, ' ')){
			//std::cout << "value is " << value <<std::endl;
			if (i==0){
				rt_e.node = value;
			} else if (i==1) {
				rt_e.src = value;
			} else if (i==2) {
				rt_e.dest = value;
			} else if ((i%2)!=0) {
				rt_g.IP = value;
			} else {
				rt_g.prob = std::atof(value.c_str());
				rt_e.gateways.push_back(rt_g);     	
			}
			i++;
		}
		rt_u.routes.push_back(rt_e);
		rt_e = routing_msgs::PRTableEntry();
        }
        myFile.close();
    }
    ////////

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<routing_msgs::NetworkUpdate>("rt_upd", 1000); // Create a publisher 
    ros::Rate loop_rate(2); // Set a publish rate of 2 Hz

    
    while (ros::ok()) // Create a loop that will go until someone stops the program execution
    {
        pub.publish(rt_u); // Publish the message within the 'count' variable
        ros::spinOnce();
        loop_rate.sleep(); // Make sure the publish rate maintains at 2 Hz
    }
    
    return 0;
}
