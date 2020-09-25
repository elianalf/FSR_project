#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "nav_msgs/Path.h"

using namespace std;

#define pi 3.14
#define Ts 0.01
#define lin_vel_max 0.6
#define ang_vel_max 0.3
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
 
  double teta=0;
   geometry_msgs::Twist t;
   geometry_msgs::Twist v;
   geometry_msgs::Twist cmd_v;
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
   int n_step;
   double teta_new=0;
   int nstep=0;
   double teta_old=0;
   double teta_e=0;
   double interval;
   x_goal[0]=1;
   y_goal[0]=1;
   x_goal[1]=0;
   y_goal[1]=2;
   
   //initialization
   c_p.pose.position.x =x_start;
   c_p.pose.position.y = y_start;
   p.poses.push_back(c_p);
   t.linear.x=0;
   t.linear.y=0;
  
   pd.push_back(t);
   v_des.push_back(t);
   teta_des.push_back(0);
   double teta_buf=0;
   //plan trajectory
   int i=1;
   for(int j=0;j<nNodesPath;j++){
      des_node(0)=x_goal[j];
       des_node(1)=y_goal[j];
       dir = des_node - curr_p;
       teta_old=teta_new;
       teta_new=atan2(dir(1),dir(0));
       interval=dir.norm() / lin_vel_max; //time interval
       n_step=interval/Ts;//n step
       
       //xdd=(pd[i].linear.x-pd[i-1].linear.x)/Ts;
       //ydd=(pd[i].linear.y-pd[i-1].linear.y)/Ts;
       //cout<< "xdd ydd "<<xdd<<" "<<ydd<<endl;
       teta_e=teta_new-teta_old;
       if(fabs((teta_new+3.14) - (teta_old+3.14)) > 3.14){ v.angular.z = ang_vel_max * ((teta_e>0)?-1:1); }
       else { v.angular.z = ang_vel_max * ((teta_e>0)?1:-1); }
          
       interval=abs(teta_e) / ang_vel_max; //time interval
       nstep=interval/Ts;
       v.linear.x=0;
       cout<< "v w_ "<<v.linear.x<<" "<<v.angular.z<<endl;
       //teta_buf=teta_old;
       for(int k=0;k<nstep;k++){
         //teta_buf+= teta_buf
         teta_des.push_back(teta_new);
         p.poses.push_back(c_p);
         pd.push_back(t);
         v_des.push_back(v);
         i++;
       }
       cout<<"size "<<v_des.size()<<" "<<p.poses.size()<<endl;
     while((des_node - curr_p).norm() > 0.05){
       
       dir = des_node - curr_p;
       dir = dir / dir.norm();
       curr_p+= dir*(1.0/n_step); //4.46 pag 185
       //cout<< "Current position "<< curr_p.transpose()<<endl;
       c_p.pose.position.x = curr_p(0);
       c_p.pose.position.y = curr_p(1);
       p.poses.push_back(c_p);
       //cout<<i<<" P "<<p.poses[i].pose.position.x<<" "<<p.poses[i-1].pose.position.x<<endl;
       
       
       t.linear.x=(p.poses[i].pose.position.x-p.poses[i-1].pose.position.x)/Ts;
       t.linear.y=(p.poses[i].pose.position.y-p.poses[i-1].pose.position.y)/Ts;
       //cout<< "vel "<<t.linear.x<<" "<<t.linear.y<<endl;
       pd.push_back(t);
       
       
       //teta_buf=atan2(pd[i].linear.y,pd[i].linear.x);
       teta_des.push_back(teta_new);
       //cout<<"teta "<<teta_des[i]<<endl;
       //teta_buf=teta_des[i]-teta_des[i-1];
       v.angular.z = 0;
       v.linear.x=sqrt(pd[i].linear.x*pd[i].linear.x+pd[i].linear.y*pd[i].linear.y); //v des
       v_des.push_back(v);
       //cout<< "v_ w "<<v.linear.x<<" "<<v.angular.z<<endl;
       //t.angular.z=(ydd*pd[i].linear.x-xdd*pd[i].linear.y)/(pd[i].linear.x*pd[i].linear.x+pd[i].linear.y*pd[i].linear.y); //w des
       
    
       i++;
     }
     curr_p+= dir*(1.0/n_step);
   }
   
   i=0;
   int path_size=v_des.size();
  
   
   Eigen::Matrix3d cos_sin;
   Eigen::Vector3d q_e;
   Eigen::Vector3d e;
   double u1=0;
   double u2=0;
   int k1=1.4;
   int k2=0;
   int k3=1.4;
   
   for(i=0;i<path_size;i++){
   teta=_yaw;
   cos_sin(0,0)=cos(teta);
   cos_sin(0,1)=sin(teta);
   cos_sin(0,2)=0;
   cos_sin(1,0)=-sin(teta);
   cos_sin(1,1)=cos(teta);
   cos_sin(1,2)=0;
   cos_sin(2,0)=0;
   cos_sin(2,1)=0;
   cos_sin(2,2)=1;
   e<< (p.poses[i].pose.position.x -odom_pos.x), (p.poses[i].pose.position.y-odom_pos.y), (teta_des[i]-teta);
   e=cos_sin*q_e;
   if(v_des[i].linear.x==0){
      cmd_v.linear.x=0;
   
   }
   else{
      k2=(1-v_des[i].angular.z*v_des[i].angular.z)/(v_des[i].linear.x*v_des[i].linear.x);
      u1=-k1*e(0);
      u2=-k2*e(1)-k3*e(2);
      cmd_v.linear.x=v_des[i].linear.x*cos(e(2)) - u1;
   
   }
   cmd_v.angular.z=v_des[i].angular.z - u2;
   
   
   twist_pub.publish(cmd_v);
   r.sleep();
   } 
  cmd_v.angular.z=0;
  cmd_v.linear.x=0; 
   twist_pub.publish(cmd_v);
   
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
