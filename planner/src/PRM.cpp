#include "PRM.h"

_PRM::_PRM()
{ //INITIALIZATION
 for(int i=0; i< num_nodes; i++){
    for(int j=0; j< num_nodes;j++){
      AdjacencyMatrix[i][j]=0;
    }
   }
   nodes_list.resize(1);
   num_connections=0;
}


_PRM::~_PRM()
{
  
}


NODE  _PRM::GenerateRandNode()
{  NODE n;
  //Range of integer x [-6;6] and y [-10;10] and range of decimal x [0.0, 0.1, ..., 0.9] y [0.0, ..., 0.4]
  int expon= rand()%2 +1;
  double decimal= rand()%10;
   n.x = (rand() % 7 + decimal/10)*pow(-1, expon); 
   expon= rand()%2 +1;  
   decimal=rand()%10;
  n.y = (rand() % 11 + decimal/10)*pow(-1, expon);
  if(n.y>10.4){n.y=10;}   
  

  return n;
}


int  _PRM::Collision_checking(double n_x,double n_y,  matrix_map map){
   int i=0;
   int j=0;
   i = (10.4/0.05) - (n_y/0.05); //in the map y=[-10.4m ,10.4m]  
   j = (6.9/0.05) + (n_x/0.05);  //in the map x = [-6.9m , 6.9m]
   //cout<<"The value in map_matrix: "<<i<<" "<<j<<" "<<map[i][j]<<endl;
   if (map[i][j]>30){ //OBSTACLE
      return 1;
   }
   else if(map[i][j]<0){ //UNEXPLORED
     
      return -1;
   }
   else { //FREE SPACE 
       
       return 0;
   }
    
}


void _PRM::CreateConnections(matrix_map map){
      list_size= nodes_list.size();
      
      double dist=0;
      double dx=0;
      double dy=0;
      double new_nodex = nodes_list.back().x;
      double new_nodey = nodes_list.back().y;
      
      for(int i=0; i< (list_size-1); i++){
         dx= nodes_list[i].x - new_nodex;
         dy= nodes_list[i].y - new_nodey;
          dist = sqrt((dx*dx)+(dy*dy) ); 
         if(dist<MaxRangeNode){ //save the connection if the distance is less than 1 m
            //cout<<"Found a node near the new one, is the path free?"<<endl;
            if (isCollisionFreePath(map, dx, dy, dist, new_nodex, new_nodey)){ 
               //cout<<"Create connection between "<<list_size-1<<"and "<< i<<"---------------------------"<< endl;
               num_connections++;
               AdjacencyMatrix[i][list_size-1]=dist;
               AdjacencyMatrix[list_size-1][i]=dist;
            }
         }
      }
}


bool _PRM::isCollisionFreePath( matrix_map map, double dx, double dy, double len, double xS, double yS)
{
 double step=StepCollisionFreePath;
  // The angle of the line
  double ang = atan2(dy, dx);
 // cout<<"Found a node near the new one, is the path free?"<<endl;
  // The direction cosines
  double kx = cos(ang);
  double ky = sin(ang);
   bool iscollisionfree=false;
  bool done = false;
  double x=0;
  double y=0;
  double pos = step; 
  while (!done) {
          
     x = xS + pos * kx;
     y = yS + pos * ky;
    // cout<<"Point on a path (is free?): "<<x <<" "<<y<<endl;
     pos += step;
    if(Collision_checking(x,y, map)!=0){
      iscollisionfree=false;
      done=true;
    }
    else if (pos > len) {
         
         iscollisionfree=true;
         done = true; 
    }
  }
   
  return iscollisionfree;
}

bool _PRM::CheckIfNodeIsNew(double nodex,double nodey){
   bool done=false;
   int i=0;
   bool IsNewNode=false;
   while(!done){
            //Does it already exist in the list of nodes of roadmap?
            if((nodes_list[i].x==nodex)&&(nodes_list[i].y==nodey)){
               done=true;
            }
            else if(i==list_size){
               IsNewNode=true;
               done=true;
            }
           i++;
     }
   return IsNewNode;
}


void _PRM::buildRoadMap(matrix_map map)
{  
   list_size=nodes_list.size();
   bool IsNewNode= false;
   NODE  rand_node;
   while( list_size!=num_nodes){
      
      while(!IsNewNode){
          rand_node=GenerateRandNode();
          //cout<<"Generate a new Rand Node: x "<<rand_node.x<<" y "<<rand_node.y<<endl;
         IsNewNode=CheckIfNodeIsNew(rand_node.x,rand_node.y);
      }
      IsNewNode=false;
     
     if(Collision_checking(rand_node.x, rand_node.y, map)==0){ //Each new node is first checked for collisions before being added
       nodes_list.push_back(rand_node); 
       int last_index=nodes_list.size() - 1;
      // cout<<"Adding node: "<<nodes_list[last_index].x<<" "<< nodes_list[last_index].y<<endl;
       //Now try to create connections between the new node and the others in the roadmap
       CreateConnections( map);
       
       
     }
    
      list_size=nodes_list.size();
   
   
  }
  cout<<"LIST SIZE: " <<list_size<<" num connection "<<num_connections<<endl;                     
}


