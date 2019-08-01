/***************************************************************************************************************************
* move.cpp
*
* Author: Qyp
*
* Update Time: 2019.3.16
*
* Introduction:  test function for sending command.msg
***************************************************************************************************************************/
#include <ros/ros.h>

#include <iostream>
#include <px4_command/ControlCommand.h>
#include <command_to_mavros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
using namespace std;
 
ros::Publisher move1_pub;
ros::Publisher move2_pub;
ros::Publisher move3_pub;
int Num_StateMachine;
px4_command::ControlCommand Command_Now;
px4_command::ControlCommand Command_Now1;
px4_command::ControlCommand Command_Now2;
px4_command::ControlCommand Command_Now3;
Eigen::Vector3d delta_uav1 = Eigen::Vector3d(2.0,0.0,0.0);
Eigen::Vector3d delta_uav2 = Eigen::Vector3d(-2.0,2.0,0.0);
Eigen::Vector3d delta_uav3 = Eigen::Vector3d(-2.0,-2.0,0.0);
void formation_control(px4_command::ControlCommand Command_leader);
void generate_com(int sub_mode, float state_desired[4]);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multiple_move");
    ros::NodeHandle nh;

    move1_pub = nh.advertise<px4_command::ControlCommand>("/uav1/px4_command/control_command", 10);
    move2_pub = nh.advertise<px4_command::ControlCommand>("/uav2/px4_command/control_command", 10);
    move3_pub = nh.advertise<px4_command::ControlCommand>("/uav3/px4_command/control_command", 10);


    ros::ServiceClient uav1_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");
    ros::ServiceClient uav1_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    ros::ServiceClient uav2_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav2/mavros/set_mode");
    ros::ServiceClient uav2_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav2/mavros/cmd/arming");
    ros::ServiceClient uav3_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav3/mavros/set_mode");
    ros::ServiceClient uav3_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav3/mavros/cmd/arming");

    mavros_msgs::SetMode uav1_mode_cmd;
    mavros_msgs::CommandBool uav1_arm_cmd;
    uav1_arm_cmd.request.value = true;

    mavros_msgs::SetMode uav2_mode_cmd;
    mavros_msgs::CommandBool uav2_arm_cmd;
    uav2_arm_cmd.request.value = true;

    mavros_msgs::SetMode uav3_mode_cmd;
    mavros_msgs::CommandBool uav3_arm_cmd;
    uav3_arm_cmd.request.value = true;


    int flag_1;
    float state_desired[4];
    int sub_mode;

    Num_StateMachine = 0;
    //----------------------------------
    //input
    while(ros::ok())
    {
        switch (Num_StateMachine)
        {
            // input
            case 0:
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
                cout << "Input the flag:  0 for Move_ENU，1 for Move_Body，2 for Land,3 for Disarm ,4 for Hold, 5 for PPN Land, 6 for Idle,7 for Takeoff to default height, 8 for set offboard and arm"<<endl;
                cin >> flag_1;

                if (flag_1 == 2)
                {
                    Num_StateMachine = 1;
                    break;
                }

                if (flag_1 == 3)
                {
                    Num_StateMachine = 2;
                    break;
                }

                if (flag_1 == 4)
                {
                    Num_StateMachine = 5;
                    break;
                }

                if (flag_1 == 6)
                {
                    Num_StateMachine = 7;
                    break;
                }

                if (flag_1 == 7)
                {
                    Num_StateMachine = 8;
                    break;
                }

                if (flag_1 == 8)
                {
                    Num_StateMachine = 9;
                    break;
                }


                //如果是机体系移动
                if(flag_1 == 1)
                {
                    Num_StateMachine = 4;
                }//惯性系移动
                else if(flag_1 == 0)
                {
                    Num_StateMachine = 3;
                }

                if (flag_1 == 5)
                {
                    Num_StateMachine = 6;
                }

                cout << "Input the sub_mode:  # 0 for xy/z position control; 3 for xy/z velocity control"<<endl;
                cin >> sub_mode;

                cout << "Please input next setpoint [x y z yaw]: "<< endl;

                cout << "setpoint_t[0] --- x [m] : "<< endl;
                cin >> state_desired[0];
                cout << "setpoint_t[1] --- y [m] : "<< endl;
                cin >> state_desired[1];
                cout << "setpoint_t[2] --- z [m] : "<< endl;
                cin >> state_desired[2];
                cout << "setpoint_t[3] --- yaw [du] : "<< endl;
                cout << "500 for input again: "<< endl;
                cin >> state_desired[3];

                //500  重新输入各数值
                if (state_desired[3] == 500)
                {
                    Num_StateMachine = 0;
                }

                break;

        //Land
        case 1:
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode = command_to_mavros::Land;

            formation_control(Command_Now);
            Num_StateMachine = 0;
            break;

        //Disarm
        case 2:
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode = command_to_mavros::Disarm;
            formation_control(Command_Now);
            Num_StateMachine = 0;
            break;

        //Move_ENU
        case 3:
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode = command_to_mavros::Move_ENU;
            generate_com(sub_mode, state_desired);
            formation_control(Command_Now);
            Num_StateMachine = 0;
            break;

        //Move_Body
        case 4:
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode = command_to_mavros::Move_Body;
            generate_com(sub_mode, state_desired);
            formation_control(Command_Now);
            Num_StateMachine = 0;
            break;

        //Hold
        case 5:
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode = command_to_mavros::Hold;
            formation_control(Command_Now);
            Num_StateMachine = 0;
            break;

        //PPN_land
        case 6:
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode = command_to_mavros::PPN_land;
            generate_com(sub_mode, state_desired);
            formation_control(Command_Now);
            Num_StateMachine = 0;
            break;

        //Idle
        case 7:
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode = command_to_mavros::Idle;
            formation_control(Command_Now);
            Num_StateMachine = 0;
            break;

        //Takeoff
        case 8:
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode = command_to_mavros::Takeoff;
            formation_control(Command_Now);
            Num_StateMachine = 0;
            break;

        //Trajectory_Tracking
        case 9:

                uav1_mode_cmd.request.custom_mode = "OFFBOARD";
                uav1_set_mode_client.call(uav1_mode_cmd);
                cout << "Setting to OFFBOARD Mode..." <<endl;

                uav1_arm_cmd.request.value = true;
                uav1_arming_client.call(uav1_arm_cmd);

                cout << "Arming..." <<endl;

                uav2_mode_cmd.request.custom_mode = "OFFBOARD";
                uav2_set_mode_client.call(uav2_mode_cmd);
                cout << "Setting to OFFBOARD Mode..." <<endl;
                uav2_arm_cmd.request.value = true;
                uav2_arming_client.call(uav2_arm_cmd);

                cout << "Arming..." <<endl;

                uav3_mode_cmd.request.custom_mode = "OFFBOARD";
                uav3_set_mode_client.call(uav1_mode_cmd);
                cout << "Setting to OFFBOARD Mode..." <<endl;
                uav3_arm_cmd.request.value = true;
                uav3_arming_client.call(uav3_arm_cmd);

                cout << "Arming..." <<endl;

            Num_StateMachine = 0;
            break;

        }

        sleep(0.2);
    }

    return 0;
}



void formation_control(px4_command::ControlCommand Command_leader)
{
    Command_Now1 = Command_leader;
    Command_Now2 = Command_leader;
    Command_Now3 = Command_leader;

    Command_Now1.Reference_State.position_ref[0] = Command_Now.Reference_State.position_ref[0] + delta_uav1[0];
    Command_Now1.Reference_State.position_ref[1] = Command_Now.Reference_State.position_ref[1] + delta_uav1[1];
    Command_Now1.Reference_State.position_ref[2] = Command_Now.Reference_State.position_ref[2] + delta_uav1[2];

    Command_Now2.Reference_State.position_ref[0] = Command_Now.Reference_State.position_ref[0] + delta_uav2[0];
    Command_Now2.Reference_State.position_ref[1] = Command_Now.Reference_State.position_ref[1] + delta_uav2[1];
    Command_Now2.Reference_State.position_ref[2] = Command_Now.Reference_State.position_ref[2] + delta_uav2[2];

    Command_Now3.Reference_State.position_ref[0] = Command_Now.Reference_State.position_ref[0] + delta_uav3[0];
    Command_Now3.Reference_State.position_ref[1] = Command_Now.Reference_State.position_ref[1] + delta_uav3[1];
    Command_Now3.Reference_State.position_ref[2] = Command_Now.Reference_State.position_ref[2] + delta_uav3[2];

    move1_pub.publish(Command_Now1);
    move2_pub.publish(Command_Now2);
    move3_pub.publish(Command_Now3);

}



// float32[3] pos_sp
// float32[3] vel_sp
// float32 yaw_sp
void generate_com(int sub_mode, float state_desired[4])
{

    static int comid = 1;
    Command_Now.Reference_State.Sub_mode  = sub_mode;

//# sub_mode 2-bit value:
//# 0 for position, 1 for vel, 1st for xy, 2nd for z.
//#                   xy position     xy velocity
//# z position       	0b00(0)       0b10(2)
//# z velocity		0b01(1)       0b11(3)

    if((sub_mode & 0b10) == 0) //xy channel
    {
        Command_Now.Reference_State.position_ref[0] = state_desired[0];
        Command_Now.Reference_State.position_ref[1] = state_desired[1];
        Command_Now.Reference_State.velocity_ref[0] = 0;
        Command_Now.Reference_State.velocity_ref[1] = 0;
    }
    else
    {
        Command_Now.Reference_State.position_ref[0] = 0;
        Command_Now.Reference_State.position_ref[1] = 0;
        Command_Now.Reference_State.velocity_ref[0] = state_desired[0];
        Command_Now.Reference_State.velocity_ref[1] = state_desired[1];
    }

    if((sub_mode & 0b01) == 0) //z channel
    {
        Command_Now.Reference_State.position_ref[2] = state_desired[2];
        Command_Now.Reference_State.velocity_ref[2] = 0;
    }
    else
    {
        Command_Now.Reference_State.position_ref[2] = 0;
        Command_Now.Reference_State.velocity_ref[2] = state_desired[2];
    }

    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;


    Command_Now.Reference_State.yaw_ref = state_desired[3]/180.0*M_PI;
    Command_Now.Command_ID = comid;
    comid++;
}
