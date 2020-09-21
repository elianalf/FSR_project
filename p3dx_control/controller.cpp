#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include "Eigen/Dense"
#include "nav_msgs/Path.h"

using namespace std;

#define pi 3.14
#define Ts 0.01
#define lin_vel_max 0.6
#define nNodesPath 2
double x_start=0;
double y_start=0;
double x_goal[nNodesPath]; //GOAL
double y_goal[nNodesPath];

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
    
}

void MOVER::nav_loop(){
   cout<<"5"<<endl;
   double dx=0;
  double dy=0;
  double ro=0;
  double gamma=0;
  double delta=0;
  double teta=0;
   geometry_msgs::Twist t;
   t = geometry_msgs::Twist();
   Eigen::Vector2d curr_p;
   ros::Rate r(100); //10 milliseconds
   Eigen::Vector2d des_node;
   curr_p(0)=0;
   curr_p(1)=0;
   geometry_msgs::PoseStamped c_p;
   nav_msgs::Path p;
   Eigen::Vector2d dir;
   vector<geometry_msgs::Twist> pd ;//x_dot_des and y_dot_des
   vector<geometry_msgs::Twist> v_des ;// v_des and w_des
   vector<double> teta_des;
   double xdd;
   double ydd;
   int n_step[nNodesPath];
   double interval;
   x_goal[0]=1;
   y_goal[0]=1;
   x_goal[1]=0;
   y_goal[1]=2;
   cout<<"n "<<n_step[0]<<endl;
   int i=1;
   c_p.pose.position.x =x_start;
   c_p.pose.position.y = y_start;
   p.poses.push_back(c_p);
   t.linear.x=0;
   t.linear.y=0;
   pd.push_back(t);
   v_des.push_back(t);
   teta_des.push_back(0);
   
   for(int j=0;j<nNodesPath;j++){
      des_node(0)=x_goal[j];
       des_node(1)=y_goal[j];
       dir = des_node - curr_p;
       interval=dir.norm() / lin_vel_max; //time interval
       n_step[j]=interval/Ts;//n step
     while((des_node - curr_p).norm() > 0.05){
       
       dir = des_node - curr_p;
       dir = dir / dir.norm();
       curr_p+= dir*(1.0/n_step[j]); //4.46 pag 185
       //cout<< "Current position "<< curr_p.transpose()<<endl;
       c_p.pose.position.x = curr_p(0);
       c_p.pose.position.y = curr_p(1);
       p.poses.push_back(c_p);
       cout<<i<<" P "<<p.poses[i].pose.position.x<<" "<<p.poses[i-1].pose.position.x<<endl;
       
       t.linear.x=(p.poses[i].pose.position.x-p.poses[i-1].pose.position.x)/Ts;
       t.linear.y=(p.poses[i].pose.position.y-p.poses[i-1].pose.position.y)/Ts;
       cout<< "vel "<<t.linear.x<<" "<<t.linear.y<<endl;
       pd.push_back(t);
       
       xdd=(pd[i].linear.x-pd[i-1].linear.x)/Ts;
      ydd=(pd[i].linear.y-pd[i-1].linear.y)/Ts;
     
       t.linear.x=sqrt(pd[i].linear.x*pd[i].linear.x+pd[i].linear.y*pd[i].linear.y); //v des
       t.angular.z=(ydd*pd[i].linear.x-xdd*pd[i].linear.y)/(pd[i].linear.x*pd[i].linear.x+pd[i].linear.y*pd[i].linear.y); //w des
       v_des.push_back(t);
       cout<< "v w "<<t.linear.x<<" "<<t.angular.z<<endl;
       double teta_buf=atan2(pd[i].linear.y,pd[i].linear.x);
       teta_des.push_back(teta_buf);
         cout<<"teta "<<teta_des[i]<<endl;
       i++;
     }
   }
   
   
   
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
