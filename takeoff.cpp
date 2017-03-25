#include "ros/ros.h"
#include <math.h>
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"

ardrone_autonomy::Navdata CurPose;
geometry_msgs::Twist CmdVel;
std_msgs::Empty blank;
int altitude;


void CurPoseCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    altitude = msg->altd;

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "takeoff");
    ros::NodeHandle n;

    ros::Subscriber CurPose_sub = n.subscribe("/ardrone/navdata", 10, CurPoseCallback);
    ros::Publisher Twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10,true);
    ros::Publisher takeoff=n.advertise<std_msgs::Empty>("/ardrone/takeoff", 10,true);
    ros::Publisher land=n.advertise<std_msgs::Empty>("/ardrone/land", 10,true);

    float dest_height;

    if (argc==1)
            land.publish(blank);
    else
        dest_height=atof(argv[1]);

    float PropTermLin, InTermLin=0, DerTermLin;
    float kp_v=atof(argv[2]), ki_v=atof(argv[3]), kd_v=atof(argv[4]);
    float SetVel;
    double t0, t1;

    float ErrorHeight, PrevErrorHeight=0;

    ros::Rate loop_rate(10);
    int count=0;
    takeoff.publish(blank);
    while (ros::ok()){
        float ErrorHeight = dest_height - altitude;
        printf("%f\t%f\n", ros::Time::now().toSec(), ErrorHeight);

        PropTermLin= kp_v * ErrorHeight;
        InTermLin+= ki_v * ErrorHeight * 0.1;
        DerTermLin= kd_v * (ErrorHeight - PrevErrorHeight)*10;

        PrevErrorHeight= ErrorHeight;

        SetVel= (PropTermLin + InTermLin + DerTermLin)/10000.0;

        CmdVel.linear.z= SetVel;
    	Twist_pub.publish(CmdVel);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
