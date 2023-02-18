#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/State.h"
using namespace std;

ros::Publisher position_setpoint_pub;
ros::Subscriber mavrosState;

bool isArmed, isOffboard;
bool generateSetpoints = false; 
//char numProcedures = 21;
char numProcedures = 16;
void callback(const mavros_msgs::State &msg)
{
  isArmed = msg.armed;
  if(msg.mode == "OFFBOARD")
  {
    isOffboard = true;
  }
}

int findProcedureNum(double currentTime, double t0, double dt, double numProcedures)
{
  int procedureNum;
  procedureNum = min(trunc((currentTime - t0)/dt),numProcedures);
  return procedureNum;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "setpoint_generator");
  ros::NodeHandle nh;
  mavrosState = nh.subscribe("/mavros/state",2,callback); 
  position_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",2);
  ros::Rate loop_rate(100);

  geometry_msgs::PoseStamped setpoint;

  int count = 1;
  int procedureNum = 0;
  double dt = 5.0; //time between setpoints
  double t0;
  /*float setpoints[numProcedures][7] = {{0.0,0.0,0.5,0.0,0.0,0.0,1.0}, //1
                            {0.0,0.0,1.0,0.0,0.0,0.0,1.0}, 
                            {0.0,0.0,0.5,0.0,0.0,0.0,1.0}, 
                            {0.0,0.0,1.0,0.0,0.0,0.0,1.0},
                            {0.0,0.0,0.5,0.0,0.0,0.0,1.0}, //5
                            {0.0,0.0,1.0,0.0,0.0,0.0,1.0}, //heave
			                {1.0,0.0,1.0,0.0,0.0,0.0,1.0},
                            {-1.0,0.0,1.0,0.0,0.0,0.0,1.0},
                            {1.0,0.0,1.0,0.0,0.0,0.0,1.0},
                            {-1.0,0.0,1.0,0.0,0.0,0.0,1.0}, //10
                            {0.0,0.0,1.0,0.0,0.0,0.0,1.0}, //roll
			                {0.0,1.0,1.0,0.0,0.0,0.0,1.0},
                            {0.0,-1.0,1.0,0.0,0.0,0.0,1.0},
                            {0.0,1.0,1.0,0.0,0.0,0.0,1.0},
                            {0.0,-1.0,1.0,0.0,0.0,0.0,1.0}, //15
                            {0.0,0.0,1.0,0.0,0.0,0.0,1.0}, //pitch
  							{0.0,0.0,1.0,0.0,0.0,0.6428,0.7660},
                            {0.0,0.0,1.0,0.0,0.0,-0.6428,0.7660},
                            {0.0,0.0,1.0,0.0,0.0,0.6428,0.7660}, 
                            {0.0,0.0,1.0,0.0,0.0,-0.6428,0.7660}, //20
                            {0.0,0.0,1.0,0.0,0.0,0.0,1.0}}; // yaw*/
  
  //float setpoints[numProcedures][7] = {0.0,0.0,1.0,0.0,0.0,0.0,1.0};
  
 float setpoints[numProcedures][7] = {{0.0,0.0,1.0,0.0,0.0,0.0,1.0},
				      {0.0,0.0,1.5,0.0,0.0,0.0,1.0},
				      {0.0,0.0,0.5,0.0,0.0,0.0,1.0},
				      {0.0,0.0,1.5,0.0,0.0,0.0,1.0},
				      {0.0,0.0,0.5,0.0,0.0,0.0,1.0},
				      {0.0,0.0,1.5,0.0,0.0,0.0,1.0},
				      {0.0,0.0,0.75,0.0,0.0,0.0,1.0},
				      {0.0,0.0,1.25,0.0,0.0,0.0,1.0},
				      {0.0,0.0,0.75,0.0,0.0,0.0,1.0},
				      {0.0,0.0,1.25,0.0,0.0,0.0,1.0},
				      {0.0,0.0,0.75,0.0,0.0,0.0,1.0},
				      {0.0,0.0,1.1,0.0,0.0,0.0,1.0},
				      {0.0,0.0,0.9,0.0,0.0,0.0,1.0},
				      {0.0,0.0,1.1,0.0,0.0,0.0,1.0},
				      {0.0,0.0,0.9,0.0,0.0,0.0,1.0},
				      {0.0,0.0,0.3,0.0,0.0,0.0,1.0}};

  double currentTime;
  while(ros::ok())
  { 
    ros::spinOnce();
    	
    if(isArmed && isOffboard){
      if(!generateSetpoints)
	  {
        t0 = ros::Time::now().toSec();
	    generateSetpoints = true;
	  }
      currentTime = ros::Time::now().toSec();
      procedureNum = findProcedureNum(currentTime, t0, dt, numProcedures);
      
      setpoint.header.stamp = ros::Time::now();
      setpoint.header.seq = count;
    
      setpoint.pose.position.x = setpoints[procedureNum][0];
      setpoint.pose.position.y = setpoints[procedureNum][1];
      setpoint.pose.position.z = setpoints[procedureNum][2];
      setpoint.pose.orientation.x = setpoints[procedureNum][3];
      setpoint.pose.orientation.y = setpoints[procedureNum][4];
      setpoint.pose.orientation.z = setpoints[procedureNum][5];
      setpoint.pose.orientation.w = setpoints[procedureNum][6];

      position_setpoint_pub.publish(setpoint);
      ros::spinOnce();
      count++;
      loop_rate.sleep();
    } else
    {
	  setpoint.header.stamp = ros::Time::now();
      setpoint.header.seq = count;
    
      setpoint.pose.position.x = 0.0;
      setpoint.pose.position.y = 0.0;
      setpoint.pose.position.z = 0.1;
      setpoint.pose.orientation.x = 0.0;
      setpoint.pose.orientation.y = 0.0;
      setpoint.pose.orientation.z = 0.0;
      setpoint.pose.orientation.w = 1.0;

      position_setpoint_pub.publish(setpoint);
      ros::spinOnce();
      count++;
      loop_rate.sleep();
    }
  }
}
