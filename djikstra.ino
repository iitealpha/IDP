void setup(){
  Serial.begin(9600);
}

int random_path[] = {0,0,0,0,0,0,0,0}; // An old path
int current_graph_number = 7; //Current graph value
int goal_graph = 3; //Goal graph value

int paths_matrix[17][8]=
{
  {0,0,0,0,0,0,0,0},
  {1,8,9,4,4,0,0,0},
  {1,8,12,13,14,5,5,0},
  {1,8,12,13,18,6,6,0},
  {1,8,12,19,16,17,7,7},
  {3,11,10,9,4,4,0,0},
  {3,11,15,14,5,5,0,0},
  {3,11,15,14,13,18,6,6},
  {3,11,15,20,17,7,7,0},
  {4,9,8,1,1,0,0,0},
  {4,9,10,11,3,3,0,0},
  {5,14,13,12,8,1,1,0},
  {5,14,15,11,3,3,0,0},
  {6,18,13,12,8,1,1,0},
  {6,18,13,14,15,11,3,3},
  {7,17,16,19,12,8,1,1},
  {7,17,20,15,11,3,3,0}
};

void new_path_define(int x){
for (int i = 0; i < 8; ++i) {
  random_path[i]=paths_matrix[x][i];
}
}

void new_path(){
	//Hard coding paths between nodes
	if (current_graph_number==1){
		if (goal_graph==4){
			new_path_define(1);
		}else if (goal_graph==5){
			new_path_define(2);
		}else if (goal_graph==6){
			new_path_define(3);
		}else if (goal_graph==7){
			new_path_define(4);
		}else{
			new_path_define(0);
		}}
	if (current_graph_number==3){
		if (goal_graph==4){
			new_path_define(5);
		}else if (goal_graph==5){
			new_path_define(6);
		}else if (goal_graph==6){
			new_path_define(7);
		}else if (goal_graph==7){
			new_path_define(8);
		}else{
			new_path_define(0);
		}}
	if (current_graph_number==4){
		if (goal_graph==1){
			new_path_define(9);
		}else if (goal_graph==3){
			new_path_define(10);
		}else{
			new_path_define(0);
		}}
	if (current_graph_number==5){
		if (goal_graph==1){
			new_path_define(11);
		}else if (goal_graph==3){
			new_path_define(12);
		}else{
			new_path_define(0);
		}}
	if (current_graph_number==6){
		if (goal_graph==1){
			new_path_define(13);
		}else if (goal_graph==3){
			new_path_define(14);
		}else{
			new_path_define(0);
		}}
	if (current_graph_number==7){
		if (goal_graph==1){
			new_path_define(15);
		}else if (goal_graph==3){
			new_path_define(16);
		}else{
			new_path_define(0);
		}}
	
	
	
}
void loop(){
	new_path();
	Serial.println(random_path[3]);
  
}
