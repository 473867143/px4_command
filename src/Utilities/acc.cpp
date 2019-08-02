//头文件
#include <ros/ros.h>
#include <iostream>
#include <math_utils.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <math_utils.h>


using namespace std;
sensor_msgs::Imu acc;
sensor_msgs::Imu acc_last;
geometry_msgs::Point vel_acc;


void acc_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    acc = *msg;

    float dt;

    dt = (acc.header.stamp.sec - acc_last.header.stamp.sec) + (acc.header.stamp.nsec - acc_last.header.stamp.nsec)/10e9;


    dt = constrain_function2(dt, 0.0001,0.01);



    vel_acc.x = vel_acc.x + acc.linear_acceleration.x * dt;
    vel_acc.y = vel_acc.y + acc.linear_acceleration.y * dt;
    vel_acc.z = vel_acc.z + (acc.linear_acceleration.z-9.8f) * dt;


    acc_last = acc;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "acc");
    ros::NodeHandle nh("~");

    ros::Subscriber acc_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1000, acc_cb);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Point>("/px4_command/vel_acc", 10);


    // 频率
    ros::Rate rate(100.0);

    vel_acc.x = 0.0;
    vel_acc.y = 0.0;
    vel_acc.z = 0.0;

    



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        ros::spinOnce();

        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>[Fake Vicon]<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

        cout.setf(ios::fixed);

        cout << "Vel_acc: " << " " << fixed <<setprecision(5)<< vel_acc.x << " [ m/s ] "<< vel_acc.y <<" [ m/s ] "<<vel_acc.z<<" [ m/s ] "<<endl;

        vel_pub.publish(vel_acc);
        rate.sleep();
    }

    return 0;

}
