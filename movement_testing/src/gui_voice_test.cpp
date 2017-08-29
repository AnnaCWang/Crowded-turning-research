/*******************************************************
*                    ROS Headers                       *
********************************************************/
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

/*******************************************************
*                   Service Headers                    *
********************************************************/
#include "bwi_services/SpeakMessage.h"
#include "bwi_msgs/QuestionDialog.h"

/*******************************************************
*                 Global Variables                     *
********************************************************/
ros::Publisher signal_pub;

ros::Subscriber turn_sub;
ros::Subscriber blocked_sub;

std_msgs::String turn_msg;
std_msgs::Bool blocked_msg;

/*******************************************************
*                 Callback Functions                   *
********************************************************/
// Updates the current turn data
void turn_cb(const std_msgs::String::ConstPtr& msg)
{
    turn_msg = *msg;
}
// Updates the blocked data
void blocked_cb(const std_msgs::Bool::ConstPtr& msg)
{
    blocked_msg = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gui_voice_test");
    ros::NodeHandle n;

    ros::Rate loop_rate(30);

    // Sets up service clients
    ros::ServiceClient speak_message_client = n.serviceClient<bwi_services::SpeakMessage>("/speak_message_service/speak_message"); 
    bwi_services::SpeakMessage speak_srv;
    
    ros::ServiceClient gui_client = n.serviceClient<bwi_msgs::QuestionDialog>("question_dialog"); 
    bwi_msgs::QuestionDialog gui_srv;

    // Sets up subscribers
    turn_sub = n.subscribe("turn_state", 1, turn_cb);
    blocked_sub = n.subscribe("blocked_state", 1, blocked_cb);


    // if there are no turns or no blocks in the path currently
    while(turn_msg.data.compare("No turning")|| !blocked_msg.data)  
    {
		gui_srv.request.type = 0; 
        gui_srv.request.message = ""; 
        gui_client.call(gui_srv);
        ros::spinOnce();
    }

    double stop_yaw = 0;

    while(ros::ok())
    {
        // Updates current path and pose
        ros::spinOnce();

        
        //robot is not blocked but the robot is currently turning 
		if (!blocked_msg.data && turn_msg.data.compare("Turning left") && turn_msg.data.compare("Turning right")) 
		{
			if (turn_msg.data.compare("Turning left")) 
			{
				//robot is turning left 
				speak_srv.request.message = "Turning Left";
                speak_message_client.call(speak_srv);  
                gui_srv.request.type = 0; 
                gui_srv.request.message = "Turning Left"; 
                gui_client.call(gui_srv); 
			} else 
			{
				//robot is turning right
				speak_srv.request.message = "Turning right";
                speak_message_client.call(speak_srv);
                gui_srv.request.type = 0; 
                gui_srv.request.message = "Turning Right"; 
                gui_client.call(gui_srv);  
			}
		}
		
		//robot is blocked and robot is not currently turning
		if (blocked_msg.data&& turn_msg.data.compare("No turning")) 
		{
			speak_srv.request.message = "My path is Blocked, please clear a path for me";
            speak_message_client.call(speak_srv);
            gui_srv.request.type = 0; 
            gui_srv.request.message = "My path is Blocked, please clear a path for me"; 
            gui_client.call(gui_srv); 
		}
		
		//robot is blocked and robot is currently turning
		if (blocked_msg.data && turn_msg.data.compare("Turning left") && turn_msg.data.compare("Turning right")) 
		{
			if (turn_msg.data.compare("Turning left")) {
				//robot is turning left 
				speak_srv.request.message = "I need to turn left. My path is Blocked, please clear a path for me";
                speak_message_client.call(speak_srv);
                gui_srv.request.type = 0; 
                gui_srv.request.message = "I need to turn left. My path is Blocked, please clear a path for me"; 
                gui_client.call(gui_srv);   
			} else {
				//robot is turning right
				speak_srv.request.message = "I need to turn right. My path is Blocked, please clear a path for me";
                speak_message_client.call(speak_srv);  
                gui_srv.request.type = 0; 
                gui_srv.request.message = "I need to turn right. My path is Blocked, please clear a path for me"; 
                gui_client.call(gui_srv); 
			}
		}
        loop_rate.sleep();
    }

    return 0;
}
