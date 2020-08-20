#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <iostream>

class Map;

class Sensor
{
public:
    Sensor(double min_z, double max_z, double noise, FILE *file = nullptr);
    // set sensor noise
    void set_noise(double noise);
    // get sensor readings
    std::vector<double> get_readings(FILE *file = nullptr);
    // sensor likelihood field model
    double forward_model(std::vector<double> readings, Map map);
    // get probability of map given sensor inverse model
    double inverse_model(double robot_x, double robot_y, double robot_theta, double grid_x, double grid_y);
    // get sensor characteristics
    double get_min_range();
    double get_max_range();
    double get_sensor_noise();

private:
    // sensor recording file
    FILE *file;
    // sensor characteristics
    double min_z__;
    double max_z__;
    double sense_noise__;
};

#endif /* SENSOR_H */