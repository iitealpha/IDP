// map_names = [1,2,3,4,...20]
const int map_with_names[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20}; // Graph that has names of the graphs, actually pretty useles.
const int number_of_connections[20] = {1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,3,2,2}; // This one keeps number of connections each graph has. 
// This graph shows numbers of connected graphs in a clockwise +1 way
// ORDER IS CRUCIAL
const int map_of_connections[20][3] = {{8,0,0},{10,0,0},{11,0,0},{9,0,0},{14,0,0},{18,0,0},{17,0,0},{12,9,1},{8,4,10},{11,2,9},{3,10,15},{19,13,8},{12,18,14},{15,5,13},{11,14,20},{17,18,19},{20,7,16},{13,6,16},{16,12,0},{15,17,0}};
const int first_direction[20] = {1,1,1,3,1,2,1,1,4,2,3,1,4,2,3,2,2,3,2,3}; //Direction (in compass) of the first listed connection. 
// Compass directions: 1 - North, 2 - East, 3 - South, 4 - West.

int current_graph = 2; //In defolt situatino starts from graph 2
int current_compass = 1; // In defolt situation starts from going to the North
int current_scenario = 1; // Starts from straight line
bool this_is_the_end = false; // Becomes true when we reach final destination and need to reverse or go backwards
int random_path[] = {2,10,11,15,20,17,7}; // This one is just random, can have any length, just connect the graphs

int coordinate_from_compas(int current_graph_g,int current_compass_g){
  int direction_that_we_will_be_looking_for = 1 + (current_compass_g +1 )%4; // Shift current_compass by 2
  //Serial.println(direction_that_we_will_be_looking_for);
  //Serial.println(first_direction[current_graph_g-1]);
  return(direction_that_we_will_be_looking_for - first_direction[current_graph_g-1] + 4)%4;
}

int coordinate_from_final_destination(int current_graph_g, int destination_graph_g) {
  int x = 100;
  for (int i = 0; i < 3; ++i) {
    if (map_of_connections[current_graph_g-1][i] == destination_graph_g){x = i;}
  }
  return x;
}

int mode_t_junction(int start, int finish) {
  int x = 200;
  //Serial.println(start);
  //Serial.println(finish);
  if ((start == 2) && (finish == 0)) {x = 2;}
  if ((start == 0) && (finish == 2)) {x = 3;}
  if ((start == 1) && (finish == 2)) {x = 4;}
  if ((start == 1) && (finish == 0)) {x = 5;}
  if ((start == 2) && (finish == 1)) {x = 6;}
  if ((start == 0) && (finish == 1)) {x = 7;}
  return x;
}

int mode_of_movement(int current_graph_g, int next_graph_g, int current_compass_g) {
  int x = 300;
  if (number_of_connections[current_graph_g-1] == 1){
    x = 13; // Because it goes to the final destination
  } else if(number_of_connections[current_graph_g-1] == 2) {
    x = 1;
  } else {
    x = mode_t_junction(coordinate_from_compas(current_graph_g, current_compass_g), coordinate_from_final_destination(current_graph_g, next_graph_g));
  }
  return x;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // open the serial port at 9600 bps:

  int answers[sizeof(random_path) / sizeof(random_path[0]) - 1];
  for (int i = 1; i < (sizeof(random_path) / sizeof(random_path[0])); ++i) {
    Serial.println(random_path[i]);
    answers[i-1] = mode_of_movement(random_path[i], random_path[i+1], current_compass);
    current_compass = 1 + (first_direction[random_path[i]-1] + coordinate_from_final_destination(random_path[i], random_path[i+1])+3) % 4;
    
    Serial.println(answers[i-1]);
  }
  answers[sizeof(random_path) / sizeof(random_path[0]) - 2] = 13; 
}

void loop() {
 
}
