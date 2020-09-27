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
#define lin_vel_max 0.5
#define ang_vel_max 0.3



double x_goal=-4; //GOAL
double y_goal=8;
int indexStartNode=0;
double x_start=0;
double y_start=0;
double des_yaw=0;
    
class NAV_MAIN{
   public:
	   NAV_MAIN(); 
	   void _map_cb(nav_msgs::OccupancyGrid vec_map);
	   void run();
	   void path_planning();
	   void odom_cb( nav_msgs::OdometryConstPtr );
	    void nav_loop();
	    void plan_trajectory();
	    bool check_goal();
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
     double b;
     vector<geometry_msgs::Twist> y12_des ;// v_des and w_des
     vector<geometry_msgs::Twist> y12dot_des ;
};


NAV_MAIN::NAV_MAIN(){
   go=false;
   b=0.5;
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


//FOR THE FIRST 2 CONTROLLERS
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
       
       
       teta_e=teta_new-teta_old;
       cout<<"teta new" <<teta_new<<endl;
       if(fabs((teta_new+3.14) - (teta_old+3.14)) > 3.14){ v.angular.z = ang_vel_max * ((teta_e>0)?-1:1); }
       else { v.angular.z = ang_vel_max * ((teta_e>0)?1:-1); }
       // cout<<"w angl "<< v.angular.z<<endl; 
        
       interval=abs(teta_e) / ang_vel_max; //time interval
       nstep=interval/Ts;
       //cout<<j<< "n step angle"<<nstep<<endl;
       v.linear.x=0;
       //cout<<j<< "v w_ "<<v.linear.x<<" "<<v.angular.z<<endl;
       
       for(int k=0;k<nstep;k++){
         teta_des.push_back(teta_new);
         p.poses.push_back(c_p);
         pd.push_back(t);
         v_des.push_back(v);
         i++;
       }
       cout<<"size "<<v_des.size()<<" "<<p.poses.size()<<endl;
       
     //while((des_node - curr_p).norm() > 0.01){
      for(int o=0; o<n_step;o++){
       dir = des_node - curr_p;
       dir = dir / dir.norm();
       curr_p+= dir*(1.0/n_step); //4.46 pag 185
      
       cout<< "Current position "<< curr_p.transpose()<<endl;
       c_p.pose.position.x = curr_p(0);
       c_p.pose.position.y = curr_p(1);
       p.poses.push_back(c_p);
       //cout<<i<<" P "<<p.poses[i].pose.position.x<<" "<<p.poses[i-1].pose.position.x<<endl;
       t.linear.x=(p.poses[i].pose.position.x-p.poses[i-1].pose.position.x)/Ts;
       t.linear.y=(p.poses[i].pose.position.y-p.poses[i-1].pose.position.y)/Ts;
      // cout<< "vel "<<t.linear.x<<" "<<t.linear.y<<endl;
       pd.push_back(t);
       //teta_buf=atan2(pd[i].linear.y,pd[i].linear.x);
       teta_des.push_back(teta_new);
       //cout<<"teta "<<teta_des[i]<<endl;
       //teta_buf=teta_des[i]-teta_des[i-1];
       v.angular.z = 0;
       v.linear.x=sqrt(pd[i].linear.x*pd[i].linear.x+pd[i].linear.y*pd[i].linear.y); //v des
       v_des.push_back(v);
       cout<< "Vd Wd "<<v.linear.x<<" "<<v.angular.z<<endl;
       //t.angular.z=(ydd*pd[i].linear.x-xdd*pd[i].linear.y)/(pd[i].linear.x*pd[i].linear.x+pd[i].linear.y*pd[i].linear.y); //w des
       i++;
     }
     //curr_p+= dir*(1.0/n_step);
   }
     teta_old=teta_new;
       teta_new=des_yaw;
       
       teta_e=teta_new-teta_old;
       cout<<"teta new" <<teta_new<<endl;
       if(fabs((teta_new+3.14) - (teta_old+3.14)) > 3.14){ v.angular.z = ang_vel_max * ((teta_e>0)?-1:1); }
       else { v.angular.z = ang_vel_max * ((teta_e>0)?1:-1); }
       // cout<<"w angl "<< v.angular.z<<endl; 
        
       interval=abs(teta_e) / ang_vel_max; //time interval
       nstep=(interval/Ts - 1);
       //cout<<j<< "n step angle"<<nstep<<endl;
       v.linear.x=0;
       int cost_=ang_vel_max/nstep;
       //cout<<j<< "v w_ "<<v.linear.x<<" "<<v.angular.z<<endl;
       if(v.angular.z<0)
         cost_=-cost_;
       for(int k=0;k<nstep;k++){
         v.angular.z= v.angular.z - cost_;
         teta_des.push_back(teta_new);
         p.poses.push_back(c_p);
         pd.push_back(t);
         v_des.push_back(v);
         i++;
         
       }
      
   sleep(1);
}


  //NON LINEAR CONTROL

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
   int k1=1.9;
   int k2=2.1;
  int k3=1.9;
  double ro=0;
  double delta=0.00001;
  double gamma=0.000001;
  //1.8 2 1.8 ERROR:   -0.0913248 0.00684787 -0.0230394
    //1.9 2.1 1.9 ERROR  -0.0864483 0.00439498 -0.0217074
    // 1.9 2.2 1.9 ERROR: -0.0922016 0.00687282 -0.0233763
   double teta=0;
   double er_norm=1000;
  // while (er_norm>0.3){
   for(i=0;i<(traj_size-1);i++){
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
      q_e<< (p.poses[i].pose.position.x -odom_pos.x), (p.poses[i].pose.position.y-odom_pos.y), (teta_des[i]-teta);
      //cout<<"P DES: "<<p.poses[i].pose.position.x <<" "<<p.poses[i].pose.position.y<<" "<<teta_des[i]<<endl;
      //cout<<"P CURRENT: "<<odom_pos.x <<" "<<odom_pos.y<<" "<<teta<<endl;
      e=cos_sin*q_e;
      er_norm=sqrt(((x_goal-q_e(0))*(x_goal-q_e(0)))+((y_goal-q_e(1))*(y_goal-q_e(1))));
      cout<<"ERROR: "<<q_e.transpose()<<endl;
    if(abs(e(2))>3.14)
       e(2)=-e(2);
      
      u1=-k1*e(0);
      cmd_v.linear.x=v_des[i].linear.x*cos(e(2)) - u1;
      
      if(e(2)==0){k=0;}
      else{ k=sin(e(2))/e(2); }
      u2= - k2*v_des[i].linear.x*e(1)*k - k3*e(2);
      cmd_v.angular.z=v_des[i].angular.z - u2;
      
      twist_pub.publish(cmd_v);
      r.sleep();
      i++;
   } 
  /* k1=0.1;
   k2=1;
   k3=1;
   ro=sqrt((q_e(0)*q_e(0))+(q_e(1)*q_e(1)));
   while(abs(q_e(2))>0.04){
      teta=_yaw;
      q_e<< ( odom_pos.x-p.poses[i].pose.position.x), (odom_pos.y-p.poses[i].pose.position.y), (teta-des_yaw);
      ro=sqrt((q_e(0)*q_e(0))+(q_e(1)*q_e(1)));
      //cout<<"ERROR 2: "<<q_e.transpose()<<endl;
      gamma= atan2(q_e(1),q_e(0)) - teta + pi;
		delta = gamma + teta - des_yaw;
		cmd_v.linear.x=k1*ro*cos(gamma);
		k=0.01;
		cmd_v.angular.z=k2*gamma + k1*k*(gamma+k3*delta);
		twist_pub.publish(cmd_v);
		cout<<"cmd "<<cmd_v.linear.x<<cmd_v.angular.z<<endl;
		r.sleep();
		if(i<traj_size) i++;
   }*/
   
  cmd_v.angular.z=0;
  cmd_v.linear.x=0; 
  twist_pub.publish(cmd_v);
  r.sleep();
  twist_pub.publish(cmd_v); 
  go=false;
  while (ros::ok()){r.sleep(); }
}





/*
 //plan FOR FB LINEARIZATION

void NAV_MAIN::plan_trajectory(){
cout<<"4"<<endl;
   geometry_msgs::Twist t;
   geometry_msgs::Twist y12;
   t = geometry_msgs::Twist();
   Eigen::Vector2d curr_p;
   ros::Rate r(100);        //10 milliseconds
   Eigen::Vector2d des_node;
   curr_p(0)=0;
   curr_p(1)=0;
   geometry_msgs::PoseStamped c_p;
   Eigen::Vector2d dir;
   
   double xdd;
   double ydd;
   int n_step;
   double teta_new=0;
   double teta_old=0;
   double teta_e=0;
   double interval;
   
   //initialization
   c_p.pose.position.x =x_start;
   c_p.pose.position.y = y_start;
   p.poses.push_back(c_p);
   t.linear.x=0;
   t.linear.y=0;
   y12dot_des.push_back(t);
   y12_des.push_back(t);
   teta_des.push_back(0);
   double teta_buf=0;
   
   //plan trajectory
   int i=1;
   for(int j=1;j<nNodesPath;j++){
       des_node(0)=x_path[j];
       des_node(1)=y_path[j];
       dir = des_node - curr_p;
       
       teta_new=atan2(dir(1),dir(0));
       
       interval=dir.norm() / lin_vel_max; //time interval
       n_step=interval/Ts;//n step
       cout<<"size "<<y12_des.size()<<" "<<p.poses.size()<<endl;
       
     while((des_node - curr_p).norm() > 0.009){
       dir = des_node - curr_p;
       dir = dir / dir.norm();
       curr_p+= dir*(1.0/n_step); //4.46 pag 185
       cout<< "Current position "<< curr_p.transpose()<<endl;
       c_p.pose.position.x = curr_p(0);
       c_p.pose.position.y = curr_p(1);
       p.poses.push_back(c_p);
       //cout<<i<<" P "<<p.poses[i].pose.position.x<<" "<<p.poses[i-1].pose.position.x<<endl;
       
       y12.linear.x = p.poses[i].pose.position.x + b*cos(teta_new); //y1:des= x_des + b* cos(teta_des)
       y12.linear.y = p.poses[i].pose.position.y + b*sin(teta_new);
       //y12.linear.x = p.poses[i].pose.position.x ; //y1:des= x_des + b* cos(teta_des)
       //y12.linear.y = p.poses[i].pose.position.y ; 
       y12_des.push_back(y12);
       
       t.linear.x=(y12_des[i].linear.x-y12_des[i-1].linear.x)/Ts;
       t.linear.y=(y12_des[i].linear.y-y12_des[i-1].linear.y)/Ts;
      // cout<< "Aux vel "<<t.linear.x<<" "<<t.linear.y<<endl;
       y12dot_des.push_back(t);  //x_dot y_dot
       
       
       i++;
     }
     //curr_p+= dir*(1.0/n_step);
   }
   sleep(3);
}


//FB LINEARIZATION

void NAV_MAIN::nav_loop(){
  
   ros::Rate r(100);
   while(!go){
      r.sleep();   }
   plan_trajectory();
    cout<<"BEGIN LOOP"<<endl;
   int i=0;
   int traj_size=y12dot_des.size();
   double u1=0;
   double u2=0;
   int k1=1.8; //best  k= 1.5 b= 0.9 ERROR -0.259543 0.0585811
   int k2=1.8 ; //k=2 b=0.5 ERROR 0.0602071 -0.0179005
   double y1=0;
   double y2=0;
   double teta=0;
   cout<<"PATH SIZE "<<traj_size<<endl;;
   for(i=0;i<traj_size;i++){
      teta=_yaw;
      y1= odom_pos.x + b*cos(teta);
      y2 = odom_pos.y + b*sin(teta);
      cout<<"ERROR "<<p.poses[i].pose.position.x-odom_pos.x<<" "<<p.poses[i].pose.position.y-odom_pos.y<<endl;
      //cout<< "Aux vel "<<y12dot_des[i].linear.x<<" "<<y12dot_des[i].linear.y<<endl;
      u1=y12dot_des[i].linear.x + k1*(y12_des[i].linear.x - y1 );
      u2=y12dot_des[i].linear.y + k2*(y12_des[i].linear.y - y2);
      //cout<< "e "<<y12_des[i].linear.x - y1<<" "<<y12_des[i].linear.y - y2<<endl;
      //cout<< "U1 U2 "<<u1<<" "<<u2<<endl;
      
      cmd_v.linear.x = cos(teta)*u1 + sin(teta)*u2;
      cmd_v.angular.z = -sin(teta)*u1/b + cos(teta)*u2/b;
      //cout<< "COMMAND V W "<<cmd_v.linear.x<<" "<<cmd_v.angular.z<<endl;
      
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




 bool NAV_MAIN::check_goal(){ 
   cout<<"Gaol position: x "<<x_goal<<" y "<<y_goal<<endl;
   if((abs(x_goal)>6.9)||(abs(y_goal)>10.4)){
      cout<<"Outside the map. ";
      cout<<" "<<endl;
      return false;
    }
     else if(x_goal==x_start&&y_goal==y_start){
      cout<<"start=goal, already in this position"<<endl;
      return false;
    }
    else{
       int is_collision_free = prm.Collision_checking(x_goal,y_goal,map);
      cout<<"Is the goal position collision free?"<<endl;
      if(is_collision_free==1){
            cout<<"No"<<endl;
            return false;
      }
      else if(is_collision_free==-1){
         cout<<"Unexplored zone "<<endl;
         return false;
      }
      else if(is_collision_free==0){
         cout<<"Yes"<<endl;
         return true;
       }  
   }    
}
   
   
void NAV_MAIN::path_planning(){   
     bool goal_ok=false;
     while (!goal_ok){
        cout << "Please, enter the coordinates of the goal position."<<endl;
        cout<<"Choose X belonging to [-6.8;6.8]: ";
        cin>>x_goal;
        cout<<"Choose Y belonging to [-10.3;10.3]: ";
        cin>>y_goal;
        goal_ok=check_goal();
     } 
     
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
         cout<<"YES"<<endl;
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
         sleep(1);
         go=true;
        cout<<" "<<endl;
      }
      else{
         cout<<"NO"<<endl;
         
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
    
   return 0 ;
}
//
