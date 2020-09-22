#include "ros/ros.h"
#include <iostream>
#include "nav_msgs/OccupancyGrid.h"
#include "A_STAR.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "nav_msgs/Path.h"

using namespace std;

#define _width 276
#define _height 416
#define pi 3.14
#define Ts 0.01
#define lin_vel_max 0.6
#define ang_vel_max 0.3



double x_goal=-4; //GOAL
double y_goal=8;
int indexStartNode=0;
double x_start=0;
double y_start=0;

   
class NAV_MAIN{
   public:
	   NAV_MAIN(); 
	   void _map_cb(nav_msgs::OccupancyGrid vec_map);
	   void run();
	   void path_planning();
	   void odom_cb( nav_msgs::OdometryConstPtr );
	    void nav_loop();
	    void plan_trajectory();
   private:
      ros::NodeHandle n;
      ros::Subscriber _topic_sub;	
      matrix_map map ;
     geometry_msgs::Twist cmd_v;
     _PRM prm;
     A_STAR a_star;
     ros::Subscriber _odom_sub;
     ros::Publisher twist_pub;
     double _yaw;
     geometry_msgs::Point odom_pos;
     vector<geometry_msgs::Twist> v_des ;// v_des and w_des
     vector<double> teta_des;
     nav_msgs::Path p;
     int nNodesPath=0;
     vector<double> x_path;
     vector<double> y_path;
     bool go;
};


NAV_MAIN::NAV_MAIN(){
   go=false;
   _topic_sub = n.subscribe("/costmap/costmap/costmap", 10, &NAV_MAIN::_map_cb,this);
   twist_pub = n.advertise<geometry_msgs::Twist>("/p3dx_1/cmd_vel", 1000);
    _odom_sub = n.subscribe("/p3dx_1/odom", 0 , &NAV_MAIN::odom_cb,this);
}

void NAV_MAIN::odom_cb(nav_msgs::OdometryConstPtr odom_ptr) {
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


void NAV_MAIN::plan_trajectory(){
   geometry_msgs::Twist t;
   geometry_msgs::Twist v;
   t = geometry_msgs::Twist();
   Eigen::Vector2d curr_p;
   ros::Rate r(100);        //10 milliseconds
   Eigen::Vector2d des_node;
   curr_p(0)=0;
   curr_p(1)=0;
   geometry_msgs::PoseStamped c_p;
   Eigen::Vector2d dir;
   vector<geometry_msgs::Twist> pd ;//x_dot_des and y_dot_des
   double xdd;
   double ydd;
   int n_step;
   double teta_new=0;
   int nstep=0;
   double teta_old=0;
   double teta_e=0;
   double interval;
   
   
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
   for(int j=1;j<nNodesPath;j++){
      des_node(0)=x_path[j];
       des_node(1)=y_path[j];
       dir = des_node - curr_p;
       
       teta_old=teta_new;
       teta_new=atan2(dir(1),dir(0));
       
       interval=dir.norm() / lin_vel_max; //time interval
       n_step=interval/Ts;//n step
       //xdd=(pd[i].linear.x-pd[i-1].linear.x)/Ts;
       //ydd=(pd[i].linear.y-pd[i-1].linear.y)/Ts;
       //cout<< "xdd ydd "<<xdd<<" "<<ydd<<endl;
       
       teta_e=teta_new-teta_old;
       cout<<"teta e" <<teta_e<<endl;
       if(fabs((teta_new+3.14) - (teta_old+3.14)) > 3.14){ v.angular.z = ang_vel_max * ((teta_e>0)?-1:1); }
       else { v.angular.z = ang_vel_max * ((teta_e>0)?1:-1); }
        cout<<"w angl "<< v.angular.z<<endl; 
        
       interval=abs(teta_e) / ang_vel_max; //time interval
       nstep=interval/Ts;
       //cout<<j<< "n step angle"<<nstep<<endl;
       v.linear.x=0;
       //cout<<j<< "v w_ "<<v.linear.x<<" "<<v.angular.z<<endl;
       //teta_buf=teta_old;
       
       for(int k=0;k<nstep;k++){
         teta_des.push_back(teta_new);
         p.poses.push_back(c_p);
         pd.push_back(t);
         v_des.push_back(v);
         i++;
       }
       cout<<"size "<<v_des.size()<<" "<<p.poses.size()<<endl;
     while((des_node - curr_p).norm() > 0.04){
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
   
}

void NAV_MAIN::nav_loop(){
   cout<<"5"<<endl;
   ros::Rate r(100);
   while(!go){
      r.sleep();   
   }
   plan_trajectory();
   int i=0;
   int traj_size=v_des.size();
   double k=0;
   Eigen::Matrix3d cos_sin;
   Eigen::Vector3d q_e;
   Eigen::Vector3d e;
   double u1=0;
   double u2=0;
   int k1=1.4;
   int k2=1;
   int k3=1.4;
   double teta=0;
   for(i=0;i<traj_size;i++){
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
      //cout<<"P DES: "<<p.poses[i].pose.position.x <<" "<<p.poses[i].pose.position.y<<" "<<teta_des[i]<<endl;
      e=cos_sin*q_e;
      
      if(e(2)==0){k=0;}
      else{k=sin(e(2))/e(2);}
      
      u1=-k1*e(0);
      u2= - k2*v_des[i].linear.x*e(1)*k - k3*e(2);
      cmd_v.linear.x=v_des[i].linear.x*cos(e(2)) - u1;
      cmd_v.angular.z=v_des[i].angular.z - u2;
      
      twist_pub.publish(cmd_v);
      r.sleep();
   } 
  cmd_v.angular.z=0;
  cmd_v.linear.x=0; 
   twist_pub.publish(cmd_v);
   
   go=false;
  while (ros::ok()){r.sleep(); }
}

/*
void NAV_MAIN::nav_loop(){
   cout<<"5"<<endl;
   ros::Rate r(100);
   while(!go){
      r.sleep();   }
   plan_trajectory();
   int i=0;
   int traj_size=v_des.size();
   Eigen::Matrix3d cos_sin;
   Eigen::Vector3d q_e;
   Eigen::Vector3d e;
   double u1=0;
   double u2=0;
   int k1=1.4;
   int k2=0;
   int k3=1.4;
   double teta=0;
   for(i=0;i<traj_size;i++){
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
      cout<<"P DES: "<<p.poses[i].pose.position.x <<" "<<p.poses[i].pose.position.y<<" "<<teta_des[i]<<endl;
      e=cos_sin*q_e;
      if(v_des[i].linear.x==0){
         cmd_v.linear.x=0;
         k2=0;
      }
      else{
         k2=(1-v_des[i].angular.z*v_des[i].angular.z)/(v_des[i].linear.x*v_des[i].linear.x);
         u1=-k1*e(0);
         cmd_v.linear.x=v_des[i].linear.x*cos(e(2)) - u1;
      }
      u2=-k2*e(1)-k3*e(2);
      cmd_v.angular.z=v_des[i].angular.z - u2;
      twist_pub.publish(cmd_v);
      r.sleep();
   } 
  cmd_v.angular.z=0;
  cmd_v.linear.x=0; 
   twist_pub.publish(cmd_v);
   
   go=false;
  while (ros::ok()){r.sleep(); }
}
*/


void NAV_MAIN::_map_cb(nav_msgs::OccupancyGrid vec_map){
   //cout<<"Vector size "<<vec_map.data.size()<<endl;
   for(int i=0;i<_height;i++){
      for(int j=0;j<_width;j++){
         map[_height-1-i][j]=vec_map.data[i*_width + j];
      }
   }
  
  path_planning();
}


void NAV_MAIN::path_planning(){
  
   cout<<"Gaol position: x "<<x_goal<<" y "<<y_goal<<endl;
   if((abs(x_goal)>6.9)||(abs(y_goal)>10.4)){
      cout<<"Outside the map. ";
      cout<<" "<<endl;
    }
     else if(x_goal==x_start&&y_goal==y_start){
      cout<<"start=goal, already in this position"<<endl;
    }
    else{
       int is_collision_free = prm.Collision_checking(x_goal,y_goal,map);
      cout<<"Is the goal position collision free?"<<endl;
      if(is_collision_free==1){
            cout<<"No"<<endl;
      }
      else if(is_collision_free==-1){
         cout<<"Unexplored zone "<<endl;
      }
      else if(is_collision_free==0){
         cout<<"Yes"<<endl;
    
          prm.nodes_list[0].x=x_start;
          prm.nodes_list[0].y=y_start;
          prm.buildRoadMap(map);
          int list_s = prm.nodes_list.size();
          cout<<"FINISH THE MAP. SIZE "<<list_s<<endl;
        
         
         /* for(int i=0; i<100;i++) {
           cout<<i<<") ";
            for(int j=0;j<100;j++){
              cout<<prm.AdjacencyMatrix[i][j]<<"|"; 
            }
            cout<<endl;
          }*/
       
       int indexGoalNode = prm.ConnectStartGoalToRoadmap(map,  x_goal,  y_goal, prm.nodes_list); 
       cout<<"Index goal node "<<indexGoalNode<<" "<<prm.nodes_list[indexGoalNode].x<<" "<<prm.nodes_list[indexGoalNode].y<<endl;
         bool existPath = a_star.FindPathAStar( indexGoalNode, indexStartNode, prm.nodes_list, prm.AdjacencyMatrix);
         cout<<"Is there a path? "<<endl;
         if(existPath){
            cout<<"yes"<<endl;
            int i=indexGoalNode;
            cout<<"Path (from the end)"<<endl;
            cout<<prm.nodes_list[indexGoalNode].x<<" "<<prm.nodes_list[indexGoalNode].y<<endl;
            x_path.insert( x_path.begin() ,  prm.nodes_list[i].x);
            y_path.insert( y_path.begin() ,  prm.nodes_list[i].y);
            while ( i!= indexStartNode){
                 i=prm.nodes_list[i].parent;  
                 x_path.insert ( x_path.begin() ,  prm.nodes_list[i].x);
                 y_path.insert ( y_path.begin() ,  prm.nodes_list[i].y);
                 cout<<prm.nodes_list[i].x<<" "<<prm.nodes_list[i].y<<" "<<i<<endl;
            }
            cout<<"verify"<<endl;
            nNodesPath=x_path.size();
            for (int j=0;j<nNodesPath;j++){
               cout<<x_path[j]<<" "<<y_path[j]<<endl;
            
            }
            go=true;
           cout<<" "<<endl;
         }
         else{
            cout<<"no"<<endl;
            
         }
     }  
   }
}



void NAV_MAIN::run(){
   //boost::thread ctrl_loop_t( &MOVER::path_planning, this );
   boost::thread ctrl_loop_t( &NAV_MAIN::nav_loop, this );
   ros::spin();
}


int main(int argc, char** argv) {
   ros::init(argc, argv, "Define_goal_node" ) ;
   NAV_MAIN m;
   while (ros::ok()){
      m.run();
   }
  //  cin.getline(cmd, 50);
   return 0 ;
}

