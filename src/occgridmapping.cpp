#include "../include/occgridmapping.h"
#include <math.h>
#include <iostream>

Mapping::Mapping(Robot *r) : robot__(r)
{
    std::cout << "Initializing the occupancy grid mapping..." << std::endl;
}

void Mapping::occ_grid_mapping()
{
    std::cout << "Building the map..." << std::endl;

    double grid_width = (robot__->get_map())->get_grid_width();
    double grid_height = (robot__->get_map())->get_grid_height();

    double robot_width = robot__->get_width();
    double robot_height = robot__->get_height();

    double map_height = (robot__->get_map())->get_map_height();
    double map_width = (robot__->get_map())->get_map_width();

    int rows = map_height / grid_height;
    int cols = map_width / grid_width;

    Map *map = robot__->get_map();

    double sensor_max_range = (robot__->get_sensor())->get_max_range();
    // get robot pose
    State robot_states = robot__->get_pose();
    // get sensor data
    std::vector<double> sensor_data = (robot__->get_sensor())->get_readings(robot_states.x, robot_states.y, nullptr, true);

    while (robot_states.time_stamp != -1)
    {

        for (int y = 0; y < rows; ++y)
        {
            for (int x = 0; x < cols; ++x)
            {
                //std::cout << "Processing the cell at " << x << "," << y << std::endl;
                // compute centre of mass of each cell
                double com_x = x * grid_width + (grid_width / 2) - robot_width;
                double com_y = -(y * grid_height + grid_height / 2) + robot_height;
                // check if cell is in robot perception feild
                double r = sqrt(pow(robot_states.x - com_x, 2) + pow(robot_states.y - com_y, 2));
                // assuming if r is within the sensor range it's inside
                if (r <= robot__->get_sensor()->get_max_range())
                {
                    (*map)[y][x] = (*map)[y][x] + (robot__->get_sensor())->inverse_model(robot_states.x, robot_states.y, robot_states.orientation, com_x, com_y, l_o__, l_free__, l_occ__, sensor_data);
                }
            }
        }
        // read the next recording
        robot_states = robot__->get_pose();
        std::vector<double> sensor_data = (robot__->get_sensor())->get_readings(robot_states.x, robot_states.y);
    }
}

void Mapping::MAP_occ_grid_mapping()
{
    // implement maximum a priory occupancy grid mapping
    // take the occ grid map as input and maximize the priory
}

void Mapping::set_lo(double lo) { l_o__ = lo; }

void Mapping::set_lfree(double l_free) { l_free__ = l_free; }

void Mapping::set_locc(double l_occ) { l_occ__ = l_occ; }

void Mapping::print_map()
{
    Map *map = robot__->get_map();

    double cols = ((robot__->get_map())->get_map_width() / (robot__->get_map())->get_grid_width());
    double rows = ((robot__->get_map())->get_map_height() / (robot__->get_map())->get_grid_height());

    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            std::cout << (*map)[y][x] << " ";
        }
    }
    std::cout << "\n";
}