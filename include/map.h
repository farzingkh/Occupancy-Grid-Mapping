#ifndef MAP_H
#define MAP_H

#include <vector>

class Map
{
public:
    Map(double size_x, double size_y, double grid_width, double, double grid_height);
    void set_occupancy(double x, double y, double log_odds);
    std::vector<double> get_occupancy(double x, double y);
    double get_size_x();
    double get_size_y();
    double get_grid_width();
    double get_grid_height();

private:
    std::vector<std::vector<double>> data__;
    // world size
    double size_x__;
    double size_y__;
    double grid_width__;
    double grid_height__;
    // log odd values
    double l_o = 0;
    double l_free = -0.4;
    double l_occ = 0.4;

};

#endif /* MAP_H */