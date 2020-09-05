#include "../include/visualization.h"
#include <vector>

void Visualization::mapShow(Map *map)
{
    double mapWidth = map->get_map_width();
    double mapHeight = map->get_map_height();
    double gridWidth = map->get_grid_width();
    double gridHeight = map->get_grid_height();

    //Graph Format
    plt::title("Map");
    plt::xlim(0, (int)(mapWidth / gridWidth));
    plt::ylim(0, (int)(mapHeight / gridHeight));

    // Draw every grid of the map:
    for (double x = 0; x < mapWidth / gridWidth; x++) {
        std::cout << "Remaining Rows= " << mapWidth / gridWidth - x << std::endl;
        for (double y = 0; y < mapHeight / gridHeight; y++) {
            double l_odds = map->get_occupancy(x,y);
            
            if (l_odds == 0) { //Green unkown state
                plt::plot<std::vector<double>,std::vector<double>>({ x }, { y }, "g.");
            }
            else if (l_odds > 0) { //Black occupied state
                plt::plot<std::vector<double>,std::vector<double>>({ x }, { y }, "k.");
            }
            else { //Red free state
                plt::plot<std::vector<double>,std::vector<double>>({ x }, { y }, "r.");
            }
        }
    }

    //Save the image and close the plot
    //plt::savefig("./Images/Map.png");
    plt::show();
    plt::clf();
}