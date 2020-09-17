#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

using namespace std; 
class NODE{

   public:
      NODE();
      
   double x;
   double y;
   int parent;
   double cost_g;
   double cost_f;
   

};

NODE::NODE(){
    x=0;
    y=0;
    parent=-1;
    cost_g=-1;
   cost_f=-1;
}
