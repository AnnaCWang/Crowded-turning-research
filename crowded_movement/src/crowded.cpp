#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <queue>
#include <signal.h>
#include <cmath>
#include <vector>

/*******************************************************
*                    ROS Headers                       *
********************************************************/
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include "people_msgs/PositionMeasurementArray.h"
#include "people_msgs/PositionMeasurement.h"

/*******************************************************
*                   Service Headers                    *
********************************************************/
#include "bwi_services/SpeakMessage.h"
#include "actionlib_msgs/GoalStatus.h"
#include <move_base/move_base.h>
#include <std_srvs/Empty.h>

/*******************************************************
*                 Global Variables                     *
********************************************************/
ros::Publisher signal_pub;
ros::Publisher blocked_pub;

ros::Subscriber global_path;
ros::Subscriber subPath;
ros::Subscriber robot_pose;
ros::Subscriber status;
ros::Subscriber robot_goal;
ros::Subscriber end_goal;
ros::Subscriber cmd_vel;
ros::Subscriber movement_subscriber;
ros::Subscriber leg_tracker;

nav_msgs::Path current_path;
nav_msgs::Path current_subPath;
geometry_msgs::Pose current_pose;
geometry_msgs::Pose end_pose;
geometry_msgs::Twist current_vel;

std::vector<geometry_msgs::PoseStamped> prev_subPaths_array;

actionlib_msgs::GoalStatusArray r_goal;
people_msgs::PositionMeasurementArray legs_pos;

bool heard_path = false;
bool heard_subPath = false;
bool heard_pose = false;
bool heard_goal = false; 
bool block_detected = false;
bool is_blocked = false;
bool changeDistance_blocked = false;
bool leg_blocked = false;

void sig_handler(int sig) {
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


/*******************************************************
*                 Callback Functions                   *
********************************************************/
void path_cb(const nav_msgs::Path::ConstPtr& msg)
{
    current_path = *msg;
    heard_path = true;
    prev_subPaths_array.clear();
	int maxElements = 0;
	maxElements = (current_subPath.poses).size() < 10 ? (current_subPath.poses).size() : 10;
	//transferring current elements to prev_elements array
	while (maxElements > 0 && (current_subPath.poses).size() > maxElements) {
		prev_subPaths_array[maxElements - 1] = (current_subPath.poses)[maxElements - 1];
		maxElements--;
	} 
	//receiving new current subpath
	current_subPath = *msg;
	heard_subPath = true;
	
}


// status code 1 moving in progress
// status code 4 is failed to get a plan - enters recovery mode (don't know how to prevent recovery mode)
// status code 3 is code succeded
//recieves status of bot
void status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr& msg_goal)
{
    r_goal = *msg_goal;
    heard_goal = true;
}

//receives current pose 
void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    geometry_msgs::PoseWithCovarianceStamped new_pose = *msg;
    current_pose = new_pose.pose.pose;
    heard_pose = true;
}

//recieves end pose
void end_goal_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
    geometry_msgs::Pose new_pose = *msg;
    end_pose = new_pose;
    heard_pose = true;
}

//recieves current velocity
void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    current_vel = *msg;
}

//recieves positions of legs in view
void leg_cb(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{
	legs_pos = *msg;
	//legs_pos_array = legs_pos.people;
}

//FIX DISTANCE METHODS SOMEHOW
//returns distance between two geometry_msgs::Pose objects
double getDistance(geometry_msgs::Pose target, geometry_msgs::Pose other)
{
		return sqrt( pow((target.position.x - other.position.x),2) + pow((target.position.y - other.position.y),2));
}

//returns distance between two objects, geometry_msgs::Pose object and geometry_msgs::PoseStamped object
double getDistance2(geometry_msgs::Pose target, geometry_msgs::PoseStamped other)
{
		return sqrt( pow((target.position.x - other.pose.position.x),2) + pow((target.position.y - other.pose.position.y),2));
}

//returns distance between two points, geometry_msgs::Point objects
double getDistance3(geometry_msgs::Point target, geometry_msgs::Point other)
{
		return sqrt( pow((target.x - other.x),2) + pow((target.y - other.y),2));
}

bool getAverageDiffGlobalPath() 
{
	//finding the avg distances between coordinates of the previous and the current <geometry_msgs::Pose> vectors
	int size = (current_subPath.poses).size() <= prev_subPaths_array.size() ? (current_subPath.poses).size() : prev_subPaths_array.size();

	int zChange = 0;
	double avgDistance = 0;
	for (unsigned int val = 0; val < size; val++) 
	{
		avgDistance += getDistance2((current_subPath.poses)[val].pose, prev_subPaths_array.at(val));
		zChange += std::abs((current_subPath.poses)[val].pose.position.z - prev_subPaths_array.at(val).pose.position.z);
	}

	avgDistance /= size;
	zChange  /= size;
	return avgDistance < .004 && zChange > 0.1;
}

bool blockLegs() 
{
	geometry_msgs::Point leg_point;
	geometry_msgs::Point avg_pt;
	double avgDistance = 0;
	for (unsigned int i = 0; i < (legs_pos.people).size(); i++) 
	{
		leg_point = ((legs_pos.people)[i]).pos;
		//don't know if this works for all situations
		avgDistance = getDistance3 (leg_point, avg_pt);
		if (avgDistance < 0.5) {
			return true;
		}
	}
	return false;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "crowded_movement");
    ros::NodeHandle n;

    ros::Rate loop_rate(30);
    ros::Rate inner_rate(100);
    ros::Rate recovered_check(2);
    
    signal(SIGINT, sig_handler);
    
    srand(time(NULL));
    time_t now = time(0);
    
    double check_pose;
    int old_size = 0;
    //counts number in queue that is greater for distance
    int num_greater = 0;

	//set up service client
	ros::ServiceClient speak_message_client = n.serviceClient<bwi_services::SpeakMessage>("/speak_message_service/speak_message");
    bwi_services::SpeakMessage speak_srv;

    ros::ServiceClient init_count_client = n.serviceClient<std_srvs::Empty>("move_base/init_replan_count");
    std_srvs::Empty init_count_srv;


    //publisher for state of client
    blocked_pub = n.advertise<std_msgs::Bool>("/blocked_state", 1000);
    std_msgs::Bool blocked;
    blocked.data = false;

    // Sets up subscribers
    global_path = n.subscribe("/move_base/Global_planner/plan", 1, path_cb);
    robot_pose  = n.subscribe("/amcl_pose", 1, pose_cb);
    robot_goal  = n.subscribe("/move_base/status", 1, status_cb);
    end_goal    = n.subscribe("/led_study/blocked_goal", 1, end_goal_cb); 
    cmd_vel     = n.subscribe("/cmd_vel", 1, cmd_vel_cb);
    leg_tracker = n.subscribe("/leg_detector/leg_tracker_measurements", 1, leg_cb);

    // Sets up action get_count_client
    std::queue<double> pose_queue;
    double distanceToGoal;
    double check = 0;
    distanceToGoal = getDistance(end_pose, current_pose);
    // Waits for current path and pose to update
    init_count_client.call(init_count_srv);
    
    //while there is no destination current or no heard pose, will sleep
    while(!heard_path || !heard_pose)
    {
		blocked.data = false;
        blocked_pub.publish(blocked);
        ros::spinOnce();
        loop_rate.sleep();  
    }
    
    
    while(ros::ok())
    {
        // Updates current path and pose
		old_size = current_path.poses.size(); 
        ros::spinOnce();

        //command velocity and distance and make a queue

        //ROS_INFO_STREAM(r_goal.status_list[0].status);
		printf("%s","oouter ros inner loop\n");

		//if there is a goal
		// if heard subpath, then robot must be moving to goal
        if(heard_goal == true && heard_subPath == true)
        {
			printf("%s","Heard goal- inside ros inner loop\n");

            //TODO check if getting closer to goal by euclidian distance
            //Check constant if correct for variability
            //ROS_INFO_STREAM("distanceToGoal " << distanceToGoal);
            //ROS_INFO_STREAM("getDistance " << getDistance());
            if(!(pose_queue.size() < 10)){
                pose_queue.pop();
            }
            pose_queue.push(getDistance(end_pose, current_pose)); 
            check_pose =  pose_queue.front();
            num_greater = 0;
            for(int i = 1; i < 8; i++){
                //do math to compare diff between end and the poses from each point- what is this for?
                if(check_pose > (double) pose_queue.front()){
                   // time_for_blocked = true; 
                }
            }
            //ROS_INFO_STREAM("out of loop distance to goal " <<  getDistance());
            check = distanceToGoal-getDistance(end_pose, current_pose);
            
			//test with calculating distance between new global paths and turning values
			changeDistance_blocked = getAverageDiffGlobalPath() && blockLegs();
	
         /* 
            ROS_INFO_STREAM("--------------------------pose size: " << old_size);
            ROS_INFO_STREAM("--------------------------pose size: " << current_path.poses.size());
            ROS_INFO_STREAM("--------------------------dist size: " << getDistance());*/
            is_blocked = ((current_path.poses.size() > 5 || check > 1 ) && (current_vel.linear.x == 0 && current_vel.linear.y == 0 && current_vel.linear.z == 0 ) && blockLegs());

			/* 
			 * Checks blocked if - 
			 * path is small but not close to goal and blocked by legs
			 * in recovery mode- plan was aborted so status :: 4
			 * having similar paths while turning more - and blocked by legs				//EXPLAIN BETTER
			 * not making progress while it still has a path and blocked by legs */
            while(((current_path.poses.size() < 5) && getDistance(end_pose, current_pose) > .5 && blockLegs()) 
            || changeDistance_blocked  
            || (r_goal.status_list[0].status == 4)
            || is_blocked)
            {
                //check = -check;
                //ROS_INFO_STREAM("distanceToGoal " << distanceToGoal);
                //ROS_INFO_STREAM("getDistance " << getDistance());
				printf("%s","Inside blocked loop- NOT GOOD RIGHT NOW. ABORTT\n");

                blocked.data = true;
                blocked_pub.publish(blocked);

               /* 
                ROS_INFO_STREAM("difference in loop " << (check));
                ROS_INFO_STREAM("distanceToGoal " << distanceToGoal);
                ROS_INFO_STREAM("in loop distance to goal " <<  getDistance());
                ROS_INFO_STREAM("--------------------------dist size: " << getDistance());*/
                check = distanceToGoal-getDistance(end_pose, current_pose);
                is_blocked = ((current_path.poses.size() > 5 || check > 1 ) && (current_vel.linear.x == 0 && current_vel.linear.y == 0 && current_vel.linear.z == 0 ) && blockLegs());
				if (!block_detected) 
				{
					speak_srv.request.message = "My path is Blocked, please clear a path for me";
					speak_message_client.call(speak_srv);
				}
                block_detected = true;
                is_blocked = (getDistance(end_pose, current_pose) > .5 && (current_vel.linear.x == 0 && current_vel.linear.y == 0 && current_vel.linear.z == 0 ) && blockLegs());
                inner_rate.sleep();
                ros::spinOnce();
               /* 
                ROS_INFO_STREAM("--------------------------pose size old : " << old_size);
                ROS_INFO_STREAM("--------------------------pose size: " << current_path.poses.size());
                ROS_INFO_STREAM("--------------------------pose size diff : " << old_size - current_path.poses.size());*/
            } 
            blocked.data = false;;
            blocked_pub.publish(blocked);
            recovered_check.sleep();

            if (block_detected)
            {
                block_detected = false;
            }
            heard_goal = false;
        } 
        init_count_client.call(init_count_srv);
        distanceToGoal = getDistance(end_pose, current_pose);
        loop_rate.sleep();
    } //end outer while loop
    return 0;
}

