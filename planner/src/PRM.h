#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#define _width 276
#define _height 416
#define num_nodes 400
#define StepCollisionFreePath 0.05
#define start_pos 0

typedef int matrix_map[_height][_width];
typedef double adj_matrix[num_nodes][num_nodes];

using namespace std; 
class _PRM {

   public:
      _PRM();
      ~_PRM();
      geometry_msgs::Point GenerateRandNode();
      void CreateConnections(matrix_map map );
      int Collision_checking(double n_x,double n_y,  matrix_map map);
      bool isCollisionFreePath( matrix_map map, double dx, double dy, double len, double xS, double yS);
      void buildRoadMap(matrix_map map);
      bool CheckIfNodeIsNew(double nodex,double nodey);
      vector< geometry_msgs::Point > nodes_list;
      adj_matrix AdjacencyMatrix;
   private:
      int num_connections;
      int list_size=0;
      
};

