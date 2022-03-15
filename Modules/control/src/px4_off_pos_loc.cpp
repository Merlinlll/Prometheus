#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

//roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
//rosrun prometheus_control px4_off_pos

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
mavros_msgs::CommandBool arm_cmd;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void pos_cb(const ros::TimerEvent &e){
    static int p;
    if(arm_cmd.response.success){
        if(p<5){
            pose.pose.position.x = pose.pose.position.x+2;
            if(p%2==0){
                pose.pose.position.y=1;
            }
            else{
                pose.pose.position.y=-1;
            }
            p++;
        } 
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 3;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Timer pos_pub = nh.createTimer(ros::Duration(4.0), pos_cb);
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
    
   