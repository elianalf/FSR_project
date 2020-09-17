#include "ros/ros.h"
#include <iostream>
#include "nav_msgs/OccupancyGrid.h"
#include "PRM.h"

using namespace std;

#define _width 276
#define _height 416

double x_start=0;
double y_start=0;


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
     float x_goal=0;
     float y_goal=0;
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
   double x_goal=-2;                                                   //GOAL
   double y_goal=-2;
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
         else if(is_collision_free==0){
            cout<<"Yes"<<endl;
         }
         else if(is_collision_free==-1){
            cout<<"Unexplored zone "<<endl;
         }
    
    
    prm.nodes_list[0].x=0;
    prm.nodes_list[0].y=0;
    prm.buildRoadMap(map);
    int list_s = prm.nodes_list.size();
    cout<<"FINISH. SIZE "<<list_s<<endl;
   
   
  /*  for(int i=0; i<100;i++) {
     cout<<i<<") ";
      for(int j=0;j<100;j++){
        cout<<prm.AdjacencyMatrix[i][j]<<"|"; 
      }
      cout<<endl;
    }*/
     cout<<" "<<endl;
     }
}

void NAV_MAIN::run(){
   ros::spin();
}


int main(int argc, char** argv) {
   ros::init(argc, argv, "Define_goal_node" ) ;
   NAV_MAIN m;
   m.run();
   
   return 0 ;
}

