class Graph {
  
  private:
    int int_name;
    int number_of_connections;
    int array_graphs[3];
    int array_directions[3]; //0 - unassigned, 1 - North, 2 - East, 3 - South, 4 - West
    //"left" (3), "forward" (0), "right" (1), "backwards (2)", equals to direction 2 - direction 1
    // long array_weights[number_of_connections];
    // unsigned long lastDebounceTime = 0; Google what unsigned is

  public:

    Graph(int name_given, int number_of_connections_given, int connected_graphs_given[], int corresponding_directions_given[]) {
      this->int_name = name_given;
      this->number_of_connections = number_of_connections_given;      
      this->array_graphs = connected_graphs_given;
      this->array_directions = corresponding_directions_given;
    }

    void assign_weights(int weights_given[]) {
      // We assign this separately bc this is optional, BUT DO THIS VOID IF YOU WANT AUTOMATIC PATHS FINDER
      this->array_weights = weights_given;
    }

    int number_from_compas(int current_compas) { // shows origin location from current compas at current graph
      // 0 - unassigned, 1 - North, 2 - East, 3 - South, 4 - West
      int found_number = 4; // having 4 at the end means you have done something wrong
      // If current compass is 2 (East), it is the opposite of going to 2 + 2 = 4 (West), and we are looking to location of point with origin 1 + (n+1)%4
      int search_value = 1 + (current_compas + 1) % 4
      for(int i = 0; i < number_of_connections; i++){
        if (this->array_directions[i] == search_value) {
          found_number = i;
        }
      }
      return(found_number);
    }

    int number_from_destination(int final_destination) {
      int x = 100; // if it does not change, something is wrong 
      for(int i = 0; i < number_of_connections; i++){
        if (this->array_graphs[i] == final_destination) {
          x = i;
        }
      }
      return x;
    }

    int get_mode_of_motion(int final_destination, int current_compas, bool back){ //Returns "left" (3), "forward" (0), "right" (1), "backwards (2)", equals to direction 2 - direction 1
      int x = 100; // if it does not change, something is wrong 
      if (number_of_connections == 1) {
        x = 8; // go back
      } else if (number_of_connections == 2) {
        x = 1; // go straight
      } else {

        int origin = number_from_compas(current_compas);
        int destination = number_from_destination(final_destination);
        
        if (back) { //car was moving backwards
          if (origin == 0 && destination == 2) {x = 9;}
          if (origin == 0 && destination == 1) {x = 10;}
          if (origin == 1 && destination == 0) {x = 11;}
          if (origin == 2 && destination == 0) {x = 12;}
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

};
