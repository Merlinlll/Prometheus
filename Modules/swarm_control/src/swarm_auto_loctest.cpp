//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

//topic 头文件
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

#define NODE_NAME "swarm_auto_test"
#define MAX_NUM 10
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::DroneState state_uav[MAX_NUM+1];
Eigen::Vector3d pos_uav[MAX_NUM+1];                     // 邻居位置
Eigen::Vector3d vel_uav[MAX_NUM+1];                     // 邻居速度
ros::Subscriber all_state_sub[MAX_NUM+1];

int swarm_num_uav;
string uav_name[MAX_NUM+1];
int uav_id[MAX_NUM+1];
prometheus_msgs::SwarmCommand swarm_command[MAX_NUM+1];
ros::Publisher command_pub[MAX_NUM+1];

int controller_num;
float formation_size;
bool sim_mode;

Eigen::Vector3f virtual_leader_pos;
Eigen::Vector3f virtual_leader_vel;
float virtual_leader_yaw;

int formation_num = 1;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();
void pub_formation_command();
void all_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg, int uav_id)
{
    state_uav[uav_id] = *msg;
    pos_uav[uav_id]  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    vel_uav[uav_id]  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");
    ros::Rate sub_rate(10);
    nh.param<int>("swarm_num_uav", swarm_num_uav, 1);
    // 0代表位置追踪模式，1代表速度追踪模式，2代表加速度追踪模式 
    nh.param<int>("controller_num", controller_num, 0);
    nh.param<float>("formation_size", formation_size, 1.0);
    nh.param<bool>("sim_mode",sim_mode,true);
    
    virtual_leader_pos << 0.0,0.0,2.0;
    virtual_leader_yaw = 0.0;
    
    for(int i = 1; i <= swarm_num_uav; i++) 
    {
        //【发布】阵型
        command_pub[i] = nh.advertise<prometheus_msgs::SwarmCommand>("/uav" + std::to_string(i) + "/prometheus/swarm_command", 1); 
    }

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);
    // Waiting for input
    int start_flag = 0;

    printf_param();
    //解锁
    while(sim_mode && (start_flag == 0))
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to disarm all the UAVs."<<endl;
        cin >> start_flag;

        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Idle;
            swarm_command[i].yaw_ref = 999;
            command_pub[i].publish(swarm_command[i]); //【发布】阵型
        }
    }
    //起飞
    start_flag = 0;
    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to takeoff all the UAVs."<<endl;
        cin >> start_flag;

        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Takeoff;
            swarm_command[i].yaw_ref = 0.0;
            command_pub[i].publish(swarm_command[i]); //【发布】阵型
        }
    }

    float trajectory_total_time;

    cout << "Please enter 1 to launch task."<<endl;
    cin >> start_flag;
    formation_num=1;
    pub_formation_command();   
    // 判断是否达到takeoff高度
    while (pos_uav[1][2]<1.8 || pos_uav[2][2]<1.8 || pos_uav[3][2]<1.8)
    {
        //订阅所有飞机状态
        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            all_state_sub[i] = nh.subscribe<prometheus_msgs::DroneState>("/uav"+std::to_string(i)+ "/prometheus/drone_state", 10, boost::bind(all_state_cb,_1,i));
            ros::spinOnce();
            sub_rate.sleep();
        }
    }
    // 发布第一个航点
    virtual_leader_pos[0]=10;
    virtual_leader_pos[1]=0;
    virtual_leader_pos[2]=3;
    virtual_leader_yaw=0;
    pub_formation_command();
    // 判断是否达到第一个航点
    while (pos_uav[1][0]<10.8 || pos_uav[2][0]<9.8 || pos_uav[3][0]<8.8)
    {
        //订阅所有飞机状态
        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            all_state_sub[i] = nh.subscribe<prometheus_msgs::DroneState>("/uav"+std::to_string(i)+ "/prometheus/drone_state", 10, boost::bind(all_state_cb,_1,i));
            ros::spinOnce();
            sub_rate.sleep();
        }
    }
    // 变换队形
    formation_num=2;
    pub_formation_command();
    ros::Duration(5.0).sleep();
    // 发布第二个航点
    virtual_leader_pos[0]=20;
    virtual_leader_pos[1]=0;
    virtual_leader_pos[2]=3;
    virtual_leader_yaw=0;
    pub_formation_command();
    // 判断是否达到第二个航点
    while (pos_uav[1][0]<20 || pos_uav[2][0]<19 || pos_uav[3][0]<18)
    {
        //订阅所有飞机状态
        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            all_state_sub[i] = nh.subscribe<prometheus_msgs::DroneState>("/uav"+std::to_string(i)+ "/prometheus/drone_state", 10, boost::bind(all_state_cb,_1,i));
            ros::spinOnce();
            sub_rate.sleep();
        }
    }
    ros::Duration(1.0).sleep();
    // 到达后降落
    for(int i = 1; i <= swarm_num_uav; i++) 
    {
        swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Land;
        command_pub[i].publish(swarm_command[i]); //【发布】阵型
    }
    cout << "Land"<<endl;
    
    return 0;
}

void pub_formation_command()
{
    if(formation_num == 1)
    {
        cout << "Formation shape: [ One_column ]"<<endl;
        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            swarm_command[i].swarm_shape = prometheus_msgs::SwarmCommand::One_column;
        }
    }
    else if(formation_num == 2)
    {
        cout << "Formation shape: [ Triangle ]"<<endl;
        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            swarm_command[i].swarm_shape = prometheus_msgs::SwarmCommand::Triangle;
        }
    }
    else if(formation_num == 3)
    {
        cout << "Formation shape: [ Square ]"<<endl;
        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            swarm_command[i].swarm_shape = prometheus_msgs::SwarmCommand::Square;
        }
    }
    else if(formation_num == 4)
    {
        cout << "Formation shape: [ Circular ]"<<endl;
        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            swarm_command[i].swarm_shape = prometheus_msgs::SwarmCommand::Circular;
        }
    }
    else
    {
        cout << "Wrong formation shape!"<<endl;
    }

    if(controller_num == 0)
    {
        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Position_Control;
        }
    }
    else if(controller_num == 1)
    {
        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Velocity_Control;
        }
    }
    else if(controller_num == 2)
    {
        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Accel_Control;
        }
    }
    // cout << "controller_num: " << controller_num << endl;

    for(int i = 1; i <= swarm_num_uav; i++) 
    {
        swarm_command[i].swarm_size = formation_size;
        swarm_command[i].position_ref[0] = virtual_leader_pos[0] ; 
        swarm_command[i].position_ref[1] = virtual_leader_pos[1] ;
        swarm_command[i].position_ref[2] = virtual_leader_pos[2] ;  
        swarm_command[i].velocity_ref[0] = virtual_leader_vel[0] ; 
        swarm_command[i].velocity_ref[1] = virtual_leader_vel[1] ; 
        swarm_command[i].velocity_ref[2] = virtual_leader_vel[2] ; 
        swarm_command[i].yaw_ref = virtual_leader_yaw;
        command_pub[i].publish(swarm_command[i]);
    }
    cout << "virtual_leader_pos: " << virtual_leader_pos[0] << "m "<<  virtual_leader_pos[1] << "m "<<  virtual_leader_pos[2] << "m "<< endl;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Formation Flight Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "swarm_num_uav   : "<< swarm_num_uav <<endl;
    cout << "controller_num   : "<< controller_num <<endl;
    cout << "formation_size : "<< formation_size << " [m] "<< endl;
}