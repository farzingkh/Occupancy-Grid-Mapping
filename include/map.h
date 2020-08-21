#ifndef MAP_H
#define MAP_H

#include <vector>

class Map
{
public:
    Map(double size_x, double size_y, double grid_width, double, double grid_height);
    // occupancy for grids
    void set_occupancy(double x, double y, double log_odds);
    std::vector<double> get_occupancy(double x, double y);
    // landmarks
    void set_landmarks(double x, double y);
    std::vector<double> get_landmarks();

    double get_size_x();
    double get_size_y();
    double get_grid_width();
    double get_grid_height();

private:
    // landmark data
    std::vector<double> landmarks__;
    // map data
    std::vector<std::vector<double>> data__;
    // map size
    double size_x__;
    double size_y__;
    double grid_width__;
    double grid_height__;
};

#endif /* MAP_H */