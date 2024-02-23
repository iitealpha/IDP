#ifndef Graph_h
#define Graph_h

#include "Arduino.h"

class Graph{

  private:
    // static int _current_compass; // This is the static variable that represents current motion of the 
    // static bool _backwards; // Becomes true when moving backwards
    int _int_name;
    int _number_of_connections;
    int _array_graphs[3];
    int _array_directions[3]; //0 - unassigned, 1 - North, 2 - East, 3 - South, 4 - West
    int array_weights[3];

  public:

    Graph(int name_given, int number_of_connections_given, int connected_graphs_given[], int corresponding_directions_given[]);
    void assign_weights(int weights_given[]);
    int number_from_compas(int current_compas);
    int number_from_destination(int final_destination);
    int get_mode_of_motion(int final_destination, int current_compass);

};

#endif