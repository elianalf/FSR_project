#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include "Eigen/Dense"
bool go=false;

using namespace std;

#define pi 3.14
#define Ts 0.01
#define lin_vel_max 0.6
double x_start=0;
double y_start=0;
double x_goal=2; //GOAL
double y_goal=2;

class MOVER {
    public:
        MOVER();
        void odom_cb( nav_msgs::OdometryConstPtr );
      void nav_loop();
      void run();
				
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
     
        ros::Publisher twist_pub;
      double _yaw;
        geometry_msgs::Point odom_pos;
        std::vector< geometry_msgs::Point > point_list;
        
};


MOVER::MOVER(){
   twist_pub = _nh.advertise<geometry_msgs::Twist>("/p3dx_1/cmd_vel", 1000);
   _odom_sub = _nh.subscribe("/p3dx_1/odom", 0 , &MOVER::odom_cb,this);
   
}


void MOVER::odom_cb(nav_msgs::OdometryConstPtr odom_ptr) {
    //cout << "Odom!" << endl;
    odom_pos.x = odom_ptr->pose.pose.position.x;
    odom_pos.y = odom_ptr->pose.pose.position.y;
   //cout<<"x pos: "<<odom_pos.x<<endl;
   tf::Quaternion q(odom_ptr->pose.pose.orientation.x, odom_ptr->pose.pose.orientation.y, odom_ptr->pose.pose.orientation.z,  odom_ptr->pose.pose.orientation.w);
  // cout<<"orientation q:"<<endl;
  // cout<<odom_ptr->pose.pose.orientation.x<<" "<<odom_ptr->pose.pose.orientation.y<<" "<<odom_ptr->pose.pose.orientation.z<<" "<<odom_ptr->pose.pose.orientation.w<<endl;
  // cout<<"orient rpy:"<<endl;
   double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, _yaw);
   // cout<<roll<<" "<<pitch<<" "<<_yaw<<endl;
    
   go=true;
}

void MOVER::nav_loop(){

   double dx=0;
  double dy=0;
  double ro=0;
  double gamma=0;
  double delta=0;
  double teta=0;
   geometry_msgs::Twist t;
   t = geometry_msgs::Twist();
  int k1=1;
  int k2=1;
  int k3=1;
   Eigen::Vector2d curr_p;
   ros::Rate r(100); //10 milliseconds
   Eigen::Vector2d des_p;
   curr_p(0)=0;
   curr_p(1)=0;
   des_p(0)=2;
   des_p(1)=2;
   Eigen::Vector2d dir = des_p - curr_p;
   int n_step=0;
   double interval;
   interval=dir.norm() / lin_vel_max; //time interval
   n_step=interval/Ts;//n step
   cout<<"n step"<<n_step<<endl;
   dir = dir / dir.norm();
   
   cout<<"dir "<<dir<<endl;
   cout<<"yaw "<<_yaw<<endl;
   while((des_p - curr_p).norm() > 0.05){
        // curr_p= curr_p + dir*(1.0/400.0); //4.46 pag 185
         // curr_p= curr_p + dir*(1.0/n_step); //4.46 pag 185
       //  cout<< "Current position "<< curr_p.transpose()<<endl;
       // x_goal = curr_p(0);
       //y_goal = curr_p(1);
        //dx=x_goal-odom_pos.x;
      // dy=y_goal-odom_pos.y;
       dx=odom_pos.x-x_goal;
       dy=odom_pos.y-y_goal;
       cout<<"dx "<<dx<<" dy "<<dy<<endl;
       teta=_yaw;
       cout<<"teta "<<teta<<endl;
       ro=sqrt(dx*dx+dy*dy);
       //cout<<"ro "<<ro<<endl;
       gamma=atan2(dy,dx) - teta + pi;
       delta=gamma+teta;
        //cout<<"gamma "<<gamma<<endl;
        
       t.linear.x=k1*ro*cos(gamma);
       if(gamma<0.01){
         t.angular.z=0;
       }
       else{
        t.angular.z=k2*gamma+k1*(sin(gamma)*cos(gamma)/gamma)*(gamma+k3*delta);
       }
       // cout<<"v "<<t.linear.x<<" w "<<t.angular.z<<endl;
       
       twist_pub.publish(t);
       r.sleep();
   }
   cout<<"end"<<endl;
   t.linear.x=0;
   t.angular.z=0;
   twist_pub.publish(t);
   r.sleep();
    
  
  while (ros::ok()){r.sleep(); }
}


void MOVER::run(){
   boost::thread ctrl_loop_t( &MOVER::nav_loop, this );
   cout<<"cio"<<endl;
    ros::spin();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mover");
  cout<<"1"<<endl;
   MOVER m;
   cout<<"3"<<endl;
   m.run();
  char cmd[50];
  ros::Rate loop_rate(10);
  
  
 
  /*  switch (cmd[0]) {
    case 'w':
      t.linear.x += 0.5;
      break;
    case 's':
      t.linear.x -= 0.5;
      break;
    case 'a':
      t.angular.z += 1;
      break;
    case 'd':
      t.angular.z -= 1;
      break;
    default:
      t = geometry_msgs::Twist();
      break;
    }*/
    

 
 

  return 0;
}
