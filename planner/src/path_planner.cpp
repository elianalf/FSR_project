#include "ros/ros.h"
#include <iostream>
#include "nav_msgs/OccupancyGrid.h"
#include "PRM.h"

using namespace std;

#define _width 276
#define _height 416



class NAV_MAIN{
   public:
	   NAV_MAIN(); 
	   void _map_cb(nav_msgs::OccupancyGrid vec_map);
	   void run();
	   void path_planning();
   private:
      ros::NodeHandle n;
      ros::Subscriber _topic_sub;	
      matrix_map map ;
     float goal_x=0;
     float goal_y=0;
     _PRM prm;
   
  
};


NAV_MAIN::NAV_MAIN(){
   _topic_sub = n.subscribe("/costmap/costmap/costmap", 10, &NAV_MAIN::_map_cb,this);
   
}


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
   double goal_x=-2;                                                   //GOAL
   double goal_y=-2;
   cout<<"Gaol position: x "<<goal_x<<" y "<<goal_y<<endl;
   if((abs(goal_x)>6.9)||(abs(goal_y)>10.4)){
      cout<<"Outside the map. ";
      cout<<" "<<endl;
    }
    else{
       int is_collision_free=prm.Collision_checking(goal_x,goal_y,map);
          
         cout<<"Is the goal position collision free?"<<endl;
         if(is_collision_free==1){
               cout<<"No"<<endl;
         }
         else if(is_collision_free==0){
            cout<<"Yes"<<endl;
         }
         else if(is_collision_free==-1){
            cout<<"Unexplored zone "<<endl;
         }
    }
    
    prm.nodes_list[0].x=0;
    prm.nodes_list[0].y=0;
    prm.buildRoadMap(map);
    int list_s=prm.nodes_list.size();
    cout<<"FINISH. SIZE "<<list_s<<endl;
    
   
    for(int i=0; i<100;i++) {
     cout<<i<<") ";
      for(int j=0;j<100;j++){
        cout<<prm.AdjacencyMatrix[i][j]<<"|"; 
      }
      cout<<endl;
    }
     cout<<" "<<endl;
}
/*
int NAV_MAIN::Collision_checking(){
   
   int i=0;
   int j=0;
   goal_x=4.9;
   goal_y=0.1;
   cout<<"goal "<<goal_x<< " "<<goal_y<<endl;
   
   if((abs(goal_x)>6.9)||(abs(goal_y)>10.4)){
      cout<<"Outside the map. ";
      return 0;}
      
   i = (10.4/0.05) - (goal_y/0.05); //in the map y=[-10.4m ,10.4m]  
   j = (6.9/0.05) + (goal_x/0.05);  //in the map x = [-6.9m , 6.9m]
   
   if (map[i][j]>30){
      cout<<"The value in map_matrix: "<<i<<" "<<j<<" "<<map[i][j]<<endl;
      return 1;
   }
   else if(map[i][j]<0){
      cout<<"The value in map_matrix: "<<i<<" "<<j<<" "<<map[i][j]<<endl;
      return -1;
   }
   else {
       cout<<"The value in map_matrix: "<<i<<" "<<j<<" "<<map[i][j]<<endl;
       return 0;
   }
   
}

*/

void NAV_MAIN::run(){
   ros::spin();
}


int main(int argc, char** argv) {
   ros::init(argc, argv, "Define_goal_node" ) ;
   NAV_MAIN m;
   m.run();
   
   return 0 ;
}

