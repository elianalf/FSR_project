#include "ros/ros.h"
#include "PRM.h"
#include <iostream>
#include <vector>


using namespace std; 
class A_STAR{

   public:
      A_STAR();

      
      bool FindPathAStar(int, int index_node_s, vector< NODE >& , adj_matrix );
      void FindExtractNbest(vector<NODE>);
   private:
      int num_connections;
      int list_size=0;
      _PRM _prm;
      
      int Nbest;
      vector<int> open_list;
      int openlist_size;
};

