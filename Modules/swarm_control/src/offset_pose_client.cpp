#include "ros/ros.h"
#include "prometheus_msgs/GetOffsetPose.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offset_pose_client");
    ros::NodeHandle n;
    int swarm_num;
    ros::param::param<int>("~swarm_num", swarm_num, 0);
    ros::ServiceClient client = n.serviceClient<prometheus_msgs::GetOffsetPose>("/get_offset_pose");
    ros::Publisher uav2_offsetpose_pub = n.advertise<prometheus_msgs::OffsetPose>("/uav2/offset_pose", 10);
    ros::Publisher uav3_offsetpose_pub = n.advertise<prometheus_msgs::OffsetPose>("/uav3/offset_pose", 10);
    ros::Publisher uav4_offsetpose_pub = n.advertise<prometheus_msgs::OffsetPose>("/uav4/offset_pose", 10);
    ros::Publisher uav5_offsetpose_pub = n.advertise<prometheus_msgs::OffsetPose>("/uav5/offset_pose", 10);
    ros::Publisher uav6_offsetpose_pub = n.advertise<prometheus_msgs::OffsetPose>("/uav6/offset_pose", 10);
    ros::Publisher uav7_offsetpose_pub = n.advertise<prometheus_msgs::OffsetPose>("/uav7/offset_pose", 10);
    ros::Publisher uav8_offsetpose_pub = n.advertise<prometheus_msgs::OffsetPose>("/uav8/offset_pose", 10);
    ros::Publisher uav9_offsetpose_pub = n.advertise<prometheus_msgs::OffsetPose>("/uav9/offset_pose", 10);
    ros::Publisher uav10_offsetpose_pub = n.advertise<prometheus_msgs::OffsetPose>("/uav10/offset_pose", 10);
    prometheus_msgs::GetOffsetPose srv;
    srv.request.uav_num = swarm_num;
    while(ros::ok())
    {
        if(client.call(srv))
        {
            for(int i=1; i<swarm_num; i++)
            {
                std::cout << "UAV[" << i+1 << "]" << "x is [" << srv.response.uavs_offset_pose[i].x << "]" << std::endl;
                std::cout << "UAV[" << i+1 << "]" << "y is [" << srv.response.uavs_offset_pose[i].y << "]" << std::endl;
            }
            std::cout << "Please check data, input 0 to again, input other to continue" << std::endl;
            int continue_flag;
            std::cin >> continue_flag;
            if(continue_flag == 0)
            {
                continue;
            }
            uav2_offsetpose_pub.publish(srv.response.uavs_offset_pose[1]);
            uav3_offsetpose_pub.publish(srv.response.uavs_offset_pose[2]);
            uav4_offsetpose_pub.publish(srv.response.uavs_offset_pose[3]);
            uav5_offsetpose_pub.publish(srv.response.uavs_offset_pose[4]);
            uav6_offsetpose_pub.publish(srv.response.uavs_offset_pose[5]);
            uav7_offsetpose_pub.publish(srv.response.uavs_offset_pose[6]);
            uav8_offsetpose_pub.publish(srv.response.uavs_offset_pose[7]);
            uav9_offsetpose_pub.publish(srv.response.uavs_offset_pose[8]);
            uav10_offsetpose_pub.publish(srv.response.uavs_offset_pose[9]);
            ros::shutdown();
        }
        else
        {

            ROS_ERROR("Failed to call service get_offset_pose");
            ros::shutdown();
        }
    } 
}