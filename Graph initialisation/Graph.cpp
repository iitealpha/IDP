#include "Arduino.h"
#include "Graph.h"

Graph::Graph(int name_given, int number_of_connections_given, int connected_graphs_given[], int corresponding_directions_given[]) {
  //_current_compass = 1; // Because it starts from going to the North
  //_backwards = false; //because it starts moving forward
  // I should have somehow written to set current compass and backwards variables 
  // first time I write object from this class
  // Code will not work if you create any objects of this class after moving. 
  // First declare map then move
  _int_name = name_given;
  _number_of_connections = number_of_connections_given;      
  for (int i = 0; i < 3; ++i) {
    _array_graphs[i] = connected_graphs_given[i];
    _array_directions[i] = corresponding_directions_given[i];
  }
}

void Graph::assign_weights(int weights_given[]) {
  // We assign this separately bc this is optional, BUT DO THIS VOID IF YOU WANT AUTOMATIC PATHS FINDER
  for (int i = 0; i < 3; ++i) {
    this->array_weights[i] = weights_given[i];
  }
}

int Graph::number_from_compas(int current_compas) { // shows origin location from current compas at current graph
  // 0 - unassigned, 1 - North, 2 - East, 3 - South, 4 - West
  int found_number = 4; // having 4 at the end means you have done something wrong
  // If current compass is 2 (East), it is the opposite of going to 2 + 2 = 4 (West), and we are looking to location of point with origin 1 + (n+1)%4
  int search_value = 1 + (current_compas + 1) % 4;
  for (int i = 0; i < 3; i++){
    if (this->_array_directions[i] == search_value) {
      found_number = i;
    }
  }
  return(found_number);
}

int Graph::number_from_destination(int final_destination) {
  int x = 100; // if it does not change, something is wrong 
  for(int i = 0; i < _number_of_connections; i++){
    if (this->_array_graphs[i] == final_destination) {
      x = i;
    }
  }
  return x;
}

int Graph::get_mode_of_motion(int final_destination, int current_compass, ){
  int x = 100; // if it does not change, something is wrong 
  if (_number_of_connections == 1) {
    x = 8; // go back
    this->_backwards = true; // Because the way it was moving changed
  } else if (_number_of_connections == 2) {
    x = 1; // go straight
  } else {

    int origin = number_from_compas(current_compass);
    int destination = number_from_destination(final_destination);
    
    if (this->_backwards == true) { //car was moving backwards
      if (origin == 0 && destination == 2) {x = 9;}
      if (origin == 0 && destination == 1) {x = 10;}
      if (origin == 1 && destination == 0) {x = 11;}
      if (origin == 2 && destination == 0) {x = 12;}
      this->_backwards = false; // Because the way it was moving changed
    } else { //car was moving normally
      if (origin == 1 && destination == 2) {x = 2;}
      if (origin == 2 && destination == 1) {x = 3;}
      if (origin == 0 && destination == 1) {x = 4;}
      if (origin == 0 && destination == 2) {x = 5;}
      if (origin == 1 && destination == 0) {x = 6;}
      if (origin == 2 && destination == 0) {x = 7;}
    }
  }
  return x;
}


