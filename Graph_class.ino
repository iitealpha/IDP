class Graph {
  
  private:
    int name;
    int number_of_connections;
    int array_graphs[number_of_connections];
    int array_directions[numver_of_connections]; //0 - unassigned, 1 - North, 2 - East, 3 - South, 4 - West
    long array_weights[number_of_connections];
    // unsigned long lastDebounceTime = 0; Google what unsigned 

  public:

    Graph(int name_given, int number_of_connections_given) {
      name = name_given;
      number_of_connections = number_of_connections_given;
    }

    /**void init() {
      pinMode(pin, INPUT);
      update();
    }**/ //This above is for alternative initialisation of graph

    void assign_other_points(int connected_graphs_given[], int corresponding_directions_given[]) { 
      // We need to assign other points connected to this graph after we have
      // initialised all of the points, that is why we are doing this after the initialisation 
      // We do not need to know the size of these arrays as it always corresponds to 
      array_graphs = connected_graphs_given;
      array_directions = corresponding_directions_given;
    }

    void assign_weights(int weights_given[]) {
      // We assign this separately bc this is optional, BUT DO THIS VOID IF YOU WANT AUTOMATIC PATHS FINDER
      array_weights = weights_given;
    }

    int get_compass(int final_destination) {
      // This function just gives the idea to which side of the compass it needs to go
      // 0 - unassigned, 1 - North, 2 - East, 3 - South, 4 - West
      found_compass_direction = 0;
      for(int i = 0; i< number_of_connections; i++){
        if (array_graphs[i] == final_destination) {
          found_array_location = array_directions[i];
        }
      }
      /**if (found_compass_direction == 0) {
        // Write code for warning
      }**/
      return(found_compass_direction);
    }

    
    
    int get_direction(int final_destination, int current_direction) { //Returns "left" (0), "forward" (1), "right" (2), "backwards (3)"
      // Current direction is b
      update();
      return state;
    }

    bool isPressed() {
      return (getState() == HIGH);
    }

};