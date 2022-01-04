#include "ros/ros.h"
#include <Eigen/Eigen>
#include <vector>
#include <mavros/frame_tf.h>
#include "prometheus_msgs/GetOffsetPose.h"
#include "sensor_msgs/NavSatFix.h"
#include <GeographicLib/Geocentric.hpp>

std::vector<sensor_msgs::NavSatFix> uavs_global_position(10);

bool getoffsetpose(prometheus_msgs::GetOffsetPose::Request &req, prometheus_msgs::GetOffsetPose::Response &res)
{
    if(req.uav_num == 1)
    {
        return false;
    }
    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    Eigen::Vector3d uav1_gps;
    Eigen::Vector3d uav1_ecef;
    uav1_gps = {uavs_global_position[0].latitude, uavs_global_position[0].longitude, uavs_global_position[0].altitude};
    earth.Forward(uavs_global_position[0].latitude, uavs_global_position[0].longitude, uavs_global_position[0].altitude, uav1_ecef.x(), uav1_ecef.y(), uav1_ecef.z());
    for(int i=1; i<req.uav_num; i++)
    {
        Eigen::Vector3d uavs_ecef;
        Eigen::Vector3d enu_offset;
        Eigen::Vector3d ecef_offset;
        earth.Forward(uavs_global_position[i].latitude, uavs_global_position[i].longitude, uavs_global_position[i].altitude, uavs_ecef.x(), uavs_ecef.y(), uavs_ecef.z());
        ecef_offset = uavs_ecef - uav1_ecef;
        enu_offset = mavros::ftf::transform_frame_ecef_enu(ecef_offset, uav1_gps);
        res.uavs_offset_pose[i].x = enu_offset[0];
        res.uavs_offset_pose[i].y = enu_offset[1];
        std::cout << "UAV[" << i+1 << "]" << "x is [" << enu_offset[0] << "]" << std::endl;
        std::cout << "UAV[" << i+1 << "]" << "y is [" << enu_offset[1] << "]" << std::endl;
    }
}

void uav1_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[0] = *msgs;
}

void uav2_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[1] = *msgs;
}

void uav3_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[2] = *msgs;
}

void uav4_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[3] = *msgs;
}

void uav5_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[4] = *msgs;
}

void uav6_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[5] = *msgs;
}

void uav7_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[6] = *msgs;
}

void uav8_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[7] = *msgs;
}

void uav9_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[8] = *msgs;
}

void uav10_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[9] = *msgs;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_offset_pose");
    ros::NodeHandle n;
    ros::ServiceServer srv = n.advertiseService("/get_offset_pose", getoffsetpose);
    ros::Subscriber uav1_global_sub = n.subscribe("/uav1/mavros/global_position/global", 10, uav1_global_position);
    ros::Subscriber uav2_global_sub = n.subscribe("/uav2/mavros/global_position/global", 10, uav2_global_position);
    ros::Subscriber uav3_global_sub = n.subscribe("/uav3/mavros/global_position/global", 10, uav3_global_position);
    ros::Subscriber uav4_global_sub = n.subscribe("/uav4/mavros/global_position/global", 10, uav4_global_position);
    ros::Subscriber uav5_global_sub = n.subscribe("/uav5/mavros/global_position/global", 10, uav5_global_position);
    ros::Subscriber uav6_global_sub = n.subscribe("/uav6/mavros/global_position/global", 10, uav6_global_position);
    ros::Subscriber uav7_global_sub = n.subscribe("/uav7/mavros/global_position/global", 10, uav7_global_position);
    ros::Subscriber uav8_global_sub = n.subscribe("/uav8/mavros/global_position/global", 10, uav8_global_position);
    ros::Subscriber uav9_global_sub = n.subscribe("/uav9/mavros/global_position/global", 10, uav9_global_position);
    ros::Subscriber uav10_global_sub = n.subscribe("/uav10/mavros/global_position/global", 10, uav10_global_position);

    ros::spin();
}