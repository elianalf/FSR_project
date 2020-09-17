#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "PRM.h"

#define num_nodes 400
#define StepCollisionFreePath 0.05
#define start_pos 0

typedef double adj_matrix[num_nodes][num_nodes];

using namespace std; 
class A_STAR{

   public:
      A_STAR();
      ~A_STAR();
      int ConnectStartGoalToRoadmap(matrix_map map, double x, double y)
   
   private:
      int num_connections;
      int list_size=0;
      _PRM _prm;
      int indexNbest;
      vector<int> open_indexlist;
      int openlist_size;
};

