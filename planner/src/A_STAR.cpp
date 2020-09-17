A_STAR::A_STAR(){
   indexNbest=-1;

}


void A_STAR::FindPathAStar(double x_ng, double y_ng){ //node goal in the roadmap
   indexNbest=0;
   bool path_found=false;
   open_indexlist.push_back(0);
   openlist_size=open_list.size();
   
   while((openlist_size>0)||(path_found)){
   
      indexNbest=FindExtractNbest();
      if((nodes_list[indexNbest].x==x_ng)&&(nodes_list[indexNbest].y==y_ng)){
         path_found=true;
      }
      for(j=0;j<num_nodes;j++){
         if(AdjacencyMatrix[indexNbest][j]>0){ // >0 means there is a connection between node number "indexNbest" and node number "j"
            if(cost_g!=0){
            
            }
            else{
            
            }// if cost_g is different from 0 the node is visited because it has a cost  
      }
      
      openlist_size=open_indexlist.size();   
   }

}


void FindExtractNbest(){
   
   indexNbest=0; 
   int node_number=-1;
   int index=0;
   for(int i=0; i<openlist_size;i++){
      node_number = open_indexlist[i];
      if(nodes_list[node_number].cost_f<nodes_list[indexNbest].cost_f){
         indexNbest = node_number;
         index=i;
      }
   }
   cout<<"Nbest found is at index "<<indexNbest<<" in list of nodes"<<endl;
   cout<<"So extract this index: "<<open_indexlist[index]<<" from the OPEN list"<<endl;
   open_indexlist.erase[index];
}


int A_STAR::ConnectStartGoalToRoadmap(matrix_map map, double x, double y)
{
  double minD = 11;
  int minI = -1;
  for (int i = 0; i < _prm.list_size; i++) {
    double dx = x - nodes_list[i].x;
    double dy = y - nodes_list[i].y;
    double d = sqrt(dx*dx+dy*dy);
    
    if (d < minD && _prm.isCollisionFreePath( matrix_map map, dx, dy, d, nodes_list[i].x, nodes_list[i].y){
      minD = d;
      minI = i;
    }
  }
 
  return minI;
}


