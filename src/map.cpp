#include "../include/map.h"

Map(double size_x, double size_y, double grid_width, double, double grid_height) : size_x__(size)x), size_y__(size_y), grid_width__(grid_width), grid_height__(grid_height)
{
    data__ = std::vector<std::vector<double>> v(size_y__, std::vector<double> row(size_x__);
}

void Map::set_landmarks(double x, double y)
{
    landmarks__.push_back(x);
    landmarks__.push_back(y);
    // check invariant condition
    assert(landmarks__.size() % 2 == 0);
    std::cout << "[Landmark] x:" << x << " y:" << y << std::endl;
}

std::vector<double> Map::get_landmarks()
{
    return landmarks__;
}

void Map::set_occupancy(double x, double y, double log_odds)
{
    data__[x][y] = log_odds;
}

void Map::get_occupancy(double x, double y)
{
    return data__[x][y];
}

double Map::get_size_x()
{
    return size_x__;
}

double Map::get_size_y()
{
    return size_y__;
}

double Map::get_grid_width()
{
    return grid_width__;
}

double Map::get_grid_height()
{
    return grid_height__;
}
