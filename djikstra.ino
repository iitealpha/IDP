//CODE DOES NOT WORK, DONT BOTHER RUNNING

int random_path[] = {0,2,2,5,6,1}; // An old path
int current_graph = 10; //Current graph value
int goal_graph = 2; //Goal graph value

int new_path(current_graph, goal_graph){
	//Hard coding paths between nodes
	if (current_graph==1){
		if (goal_graph==4){
			return {1,8,9,4,4};
		}else if (goal_graph==5){
			return {1,8,12,13,14,5,5};
		}else if (goal_graph==6){
			return {1,8,12,13,18,6,6};
		}else if (goal_graph==7){
			return {1,8,12,19,16,17,7,7};
		}else{
			return {0,0,0};
		}}
	if (current_graph==3){
		if (goal_graph==4){
			return {3,11,10,9,4,4};
		}else if (goal_graph==5){
			return {3,11,15,14,5,5};
		}else if (goal_graph==6){
			return {3,11,15,14,13,18,6,6};
		}else if (goal_graph==7){
			return {3,11,15,20,17,7,7};
		}else{
			return {0,0,0};
		}}
	if (current_graph==4){
		if (goal_graph==1){
			return {4,9,8,1,1};
		}else if (goal_graph==3{
			return {4,9,10,11,3,3};
		}else{
			return {0,0,0};
		}}
	if (current_graph==5){
		if (goal_graph==1){
			return {5,14,13,12,8,1,1};
		}else if (goal_graph==3){
			return {5,14,15,11,3,3};
		}else{
			return {0,0,0};
		}}
	if (current_graph==6){
		if (goal_graph==1){
			return {6,18,13,12,8,1,1};
		}else if (goal_graph==3){
			return {6,18,13,14,15,11,3,3};
		}else{
			return {0,0,0};
		}}
	if (current_graph==7){
		if (goal_graph==1){
			return {7,17,16,19,12,8,1,1};
		}else if (goal_graph==3){
			return {7,17,20,15,11,3,3};
		}else{
			return {0,0,0};
		}}
	
	
	
}
void loop{
	random_path=new_path(7,3);}





#include<iostream>
#include<climits>     
using namespace std;

// this method returns a minimum distance for the 
// vertex which is not included in Tset.
int minimumDist(int dist[], bool Tset[]) 
{
	int min=INT_MAX,index;
              
	for(int i=0;i<6;i++) 
	{
		if(Tset[i]==false && dist[i]<=min)      
		{
			min=dist[i];
			index=i;
		}
	}
	return index;
}

void Dijkstra(int graph[6][6],int src) // adjacency matrix used is 6x6
{
	int dist[6]; // integer array to calculate minimum distance for each node.                            
	bool Tset[6];// boolean array to mark visted/unvisted for each node.
	
	// set the nodes with infinity distance
	// except for the initial node and mark
	// them unvisited.  
	for(int i = 0; i<6; i++)
	{
		dist[i] = INT_MAX;
		Tset[i] = false;	
	}
	
	dist[src] = 0;   // Source vertex distance is set to zero.             
	
	for(int i = 0; i<6; i++)                           
	{
		int m=minimumDist(dist,Tset); // vertex not yet included.
		Tset[m]=true;// m with minimum distance included in Tset.
		for(int i = 0; i<6; i++)                  
		{
			// Updating the minimum distance for the particular node.
			if(!Tset[i] && graph[m][i] && dist[m]!=INT_MAX && dist[m]+graph[m][i]<dist[i])
				dist[i]=dist[m]+graph[m][i];
		}
	}
	cout<<"Vertex\t\tDistance from source"<<endl;
	for(int i = 0; i<6; i++)                      
	{ //Printing
		char str=65+i; // Ascii values for pritning A,B,C..
		cout<<str<<"\t\t\t"<<dist[i]<<endl;
	}
}

int main()
{
	int graph[6][6]={
		{0, 10, 20, 0, 0, 0},
		{10, 0, 0, 50, 10, 0},
		{20, 0, 0, 20, 33, 0},
		{0, 50, 20, 0, 20, 2},
		{0, 10, 33, 20, 0, 1},
		{0, 0, 0, 2, 1, 0}};
	Dijkstra(graph,0);
	return 0;	                        
}
  
