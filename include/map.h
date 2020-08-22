#ifndef MAP_H
#define MAP_H

#include <vector>

class Map
{
public:
    Map(double map_width, double map_height, double grid_width, double grid_height);
    // occupancy for grids
    void set_occupancy(double x, double y, double log_odds);
    double get_occupancy(double x, double y);
    // landmarks
    void set_landmarks(double x, double y);
    std::vector<double> get_landmarks();

    std::vector<double>& operator[](int row);

    double get_map_width();
    double get_map_height();
    double get_grid_width();
    double get_grid_height();

private:
    // landmark data
    std::vector<double> landmarks__;
    // map data
    std::vector<std::vector<double>> data__;
    // map size
    double map_width__;
    double map_height__;
    double grid_width__;
    double grid_height__;
};

#endif /* MAP_H */