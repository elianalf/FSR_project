#include "A_STAR.h"

A_STAR::A_STAR(){
   Nbest=-1;
}


bool A_STAR::FindPathAStar(int indexGoalNode, int index_node_s, vector< NODE > &nodes_list, adj_matrix AdjacencyMatrix){ // indexes in nodes_list of the closest node to goal in the roadmap and closest node to start
   Nbest=0;
   bool path_found=false;
   open_list.push_back(index_node_s);
   openlist_size=open_list.size();
   nodes_list[0].cost_g=0;
     double dx = 0;
     double dy =0;
    double h =0;
   while((openlist_size>0)&&(!path_found)){
   
      FindExtractNbest(nodes_list);
      
      //cout<<"New node Nbest "<<Nbest<<" "<<nodes_list[Nbest].x<<" "<<nodes_list[Nbest].y<<endl;
      
      //check if the new Nbest node is the goal node 
      if(Nbest == indexGoalNode){
         cout<<"FOUND A PATH"<<endl;
         path_found=true;
      }
      else{
         for (int k=0; k< nodes_list[Nbest].AdjacencyVector.size(); k++){
            int j=nodes_list[Nbest].AdjacencyVector[k];
            if(nodes_list[j].cost_g==0){     
               nodes_list[j].cost_g = nodes_list[Nbest].cost_g + nodes_list[Nbest].ArcCost[k]; //MARK AS VISITED ASSIGNING A COST
       
       
     /*    for(int j=0;j<num_nodes;j++){
            if(AdjacencyMatrix[Nbest][j]>0){ // >0 means there is a connection between node number "Nbest" and node number "j"
               //if cost_g is different from -1 the node j is visited because it has a cost
              // cout<<"Nbest has connection with"<<j<<endl;
               if(nodes_list[j].cost_g==0){  //UNVISITED
                  //cout<<"Node is unvisited, defining the cost and adding to OPEN"<<endl;
                  nodes_list[j].cost_g =   nodes_list[Nbest].cost_g + AdjacencyMatrix[Nbest][j]; //MARK AS VISITED ASSIGNING A COST*/
                  
                  
                  
                  
                  //cout<<"NODE UNVISITED"<<j<<" cost g "<<nodes_list[j].cost_g<<endl;
                  nodes_list[j].parent=Nbest; //DEFINE THE PARENT IN THE TREE
                  open_list.push_back(j);     //INSERT THE NODE J IN OPEN 
                  dx = nodes_list[indexGoalNode].x -   nodes_list[j].x;
                   dy = nodes_list[indexGoalNode].y -   nodes_list[j].y;
                   h = sqrt(dx*dx+dy*dy);
                  nodes_list[j].cost_f = nodes_list[j].cost_g + h ; 
               }  
               else if(nodes_list[Nbest].cost_g +nodes_list[Nbest].ArcCost[k] < nodes_list[j].cost_g){ // VISITED
                  //THE COST FROM START TO NBEST + THE DISTANCE BETWEEN J AND NBEST IS LESS THAN THE ONE SAVED MEANS THERE IS A BETTER PATH 
                   nodes_list[j].parent=Nbest;
                   nodes_list[j].cost_g = nodes_list[Nbest].cost_g+nodes_list[Nbest].ArcCost[k];
                   //cout<<"NODE "<<j<<" NEW cost g "<<nodes_list[j].cost_g<<endl;
                   //cout<<"Node is visited but there is a better path, upgrating the info"<<endl;
                    
                   int indexInOpen=0;
                   //CHECK IF THE NODE J IS IN OPEN 
                   while((open_list[indexInOpen] != j)&&(indexInOpen < open_list.size())){
                     indexInOpen++;
                    
                   } 
                   if(indexInOpen == open_list.size()){                       //NODE J IS NOT IN OPEN
                        open_list.push_back(j);  //INSERT THE NODE J IN OPEN 
                   }
                   
                  //upgrade (if is open) or assign a value (if it's not in open) of the f cost
                    dx = nodes_list[indexGoalNode].x -   nodes_list[j].x;
                    dy = nodes_list[indexGoalNode].y -   nodes_list[j].y;
                    h = sqrt(dx*dx+dy*dy);
                    nodes_list[j].cost_f = nodes_list[j].cost_g + h ; 
                    //cout<<"cos f "<<endl;
                } 
            }
         
            openlist_size=open_list.size();   
        }
    } 
  
   return path_found;
}



void A_STAR::FindExtractNbest(vector< NODE > nodes_list){

   int node_number=-1;
 /*  for(int i=0; i<open_list.size();i++){
       cout<<"open list content "<<open_list[i]<<endl;
   }
*/
   int indexInOpen=0;
  // cout<<"openlist size "<<openlist_size<<endl;
   Nbest=open_list[0];
   for(int i=0; i<openlist_size;i++){
      node_number = open_list[i];
     // cout<<"node in openlist "<<open_list[i]<<endl;
      if(nodes_list[node_number].cost_f<nodes_list[Nbest].cost_f){
         Nbest = node_number; // nodeBest that we'll need as index for the nodes_list = name of the node (identified by a number) 
         indexInOpen= i;

      }
   }
  // cout<<"So extract this node: "<<open_list[indexInOpen]<<" from the OPEN list in index "<<indexInOpen<<endl;
   vector<int>::iterator it=open_list.begin() +indexInOpen;
   open_list.erase(it );

  /* for(int i=0; i<open_list.size();i++){
      cout<<"open list content "<<open_list[i]<<endl;
   }*/
}




