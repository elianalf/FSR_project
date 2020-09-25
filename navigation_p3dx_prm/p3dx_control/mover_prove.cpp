#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"

double x_start=0;
double y_start=0;
int indexStartNode=0;
double x_goal=1; //GOAL
double y_goal=1;
int N=2;

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mover");
  ros::NodeHandle nh;

  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/p3dx_1/cmd_vel", 1000);

  geometry_msgs::Twist cmd_msg;
  char cmd[50];
  ros::Rate loop_rate(10);
 cmd_msg = geometry_msgs::Twist();
  geometry_msgs::PoseStamped c_p;
   Eigen::Vector2d curr_p;
  
   Eigen::Vector2d des_p;
  
   Eigen::Vector2d dir = des_p - curr_p;
   dir = dir / dir.norm(); //versor(normalized vector)
   
   nav_msgs::Path p;
   double Ts=0.01;
   
   
 /*  lenght_path=+n_step[j]
  for(j<N_points){
    for(i<length_path){
      if(i<t[j-1])
        s[j][i]=0;
        sd=0
        sdd=0
      else if(i>t[j-1]&&i<t[j])
        s[j][i]+=(pj-pj-1).norm/n_step[j];
        sd[]=(s[j][i]-s[j]s[i-1])/Ts;
        
      else
         s[j][i]=(pj-pj-1).norm;
         sd=0
         sdd=0
      }
   }  */ 
      
   int i=0;
   for(int j=1;j<N_points;j++){  
      curr_p(0)=path_points[j-1];
      curr_p(1)=path_points[j-1];
      des_p(0)=path_points[j];
      des_p(1)=path_points[j];
      dir = des_p - curr_p;
      teta=atan2(dir.y,di.x);
      while(){
      
      
      }
    /*  curr_p(0)=path_points[j-1];
      curr_p(1)=path_points[j-1];
      des_p(0)=path_points[j];
      des_p(1)=path_points[j];
      dir = des_p - curr_p;*/
      dir = dir / dir.norm();
      while((des_p - curr_p).norm() > 0.001){
         
         
       curr_p+= dir*(1.0/n_step[j]); //4.46 pag 185
       cout<< "Current position "<< curr_p.transpose()<<endl;
       c_p.pose.position.x = curr_p(0);
       c_p.pose.position.y = curr_p(1);
       p.poses.push_back(c_p);
       xd[i]=(p.poses[i].pose.position.x-p.poses[i-1].pose.position.x)/Ts;
       yd[i]=(p.poses[i].pose.position.y-p.poses[i-1].pose.position.y)/Ts;
       i++;
     }
     end_=n_step[j]+end_;
     begin=end_;
  
   }     
   
   
   
  /*while (ros::ok())
  { 
   double dx=x_goal-x_start;
      double dy=y_goal-y_start;
      d=sqrt(dx*dx+dy*dy); */
     /* Eigen::Vector2d time;
      time<<2,4;*/

   
     while((des_p - curr_p).norm() > 0.001){
      curr_p= curr_p + dir*(1.0/200.0); //4.46 pag 185
      cout<< "Current position "<< curr_p.transpose()<<endl;
       c_p.pose.position.x = curr_p(0);
      c_p.pose.position.y = curr_p(1);
      //c_p.pose.position.z = curr_p(2);
      //c_p.pose.orientation = starting_pose.orientation;
      p.poses.push_back(c_p);
     }
    curr_p(0)=1;
   curr_p(1)=1;
   des_p(0)=2;
   des_p(1)=2;
   dir = des_p - curr_p;
   dir = dir / dir.norm();
   while((des_p - curr_p).norm() > 0.001){
      curr_p+= dir*(1.0/200.0); //4.46 pag 185
      cout<< "Current position "<< curr_p.transpose()<<endl;
       c_p.pose.position.x = curr_p(0);
      c_p.pose.position.y = curr_p(1);
      //c_p.pose.position.z = curr_p(2);
     // c_p.pose.orientation = starting_pose.orientation;
      p.poses.push_back(c_p);
   }
   

   
   while(ros::ok()){
   
   
   
   /* switch (cmd[0]) {
    case 'w':
     cmd_msg.linear.x += 0.5;
      break;
    case 's':
     cmd_msg.linear.x -= 0.5;
      break;
    case 'a':
     cmd_msg.angular.z += 1;
      break;
    case 'd':
     cmd_msg.angular.z -= 1;
      break;
    default:
     cmd_msg = geometry_msgs::Twist();
      break;*/
    }

    twist_pub.publish(t);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
