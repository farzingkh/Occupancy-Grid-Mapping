#include "../include/occgridmapping.h"
#include "../include/visualization.h"

int main()
{
    // open sensor recordings
    FILE *mes_file = fopen("../data/measurement.txt", "r");
    FILE *pos_file = fopen("../data/poses.txt", "r");
    // initialize a map
    Map map(30000, 15000, 100, 100);
    // initialize the sensor and pass the recordings
    Sensor sensor(170, 5000, 0.01, mes_file);
    // initialize a robot
    Robot robot(30000 / 5, 15000 / 3, &sensor, &map, pos_file);
    // generate the occupancy grid map
    Mapping M(&robot);
    M.occ_grid_mapping();
    // print the map
    // M.print_map();
    // plot the map
    Visualization::mapShow(&map);
    // close
    fclose(mes_file);
    fclose(pos_file);
}