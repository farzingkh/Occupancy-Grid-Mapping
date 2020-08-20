#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <exception>
#include "map.h"
#include "sensor.h"

class Robot
{
public:
    // Initialize robot in a random position in the world
    Robot(Sensor *sensor, Map *map = nullptr, FILE *pose = nullptr);
    // de'tor
    ~Robot();
    // Set robot's states
    void set_states(double new_x, double new_y, double new_orient);
    // Set robots state transition noise
    void set_noise(double forward_noise, double turn_noise, double sensor_noise);
    // Sense the environment with sensors
    std::vector<double> sense(bool noise = true);
    // Move the robot
    void move(double turn, double forward);
    // get x
    double get_x();
    // get y
    double get_y();
    // Get pose readings
    std::string get_pose(FILE *file = nullptr);
    // Get sensor readings
    std::string get_sensor_readings(FILE *file = nullptr);

private:
    // robot states
    double x__;
    double y__;
    double orientation__;
    // noise
    static double forward_noise__;
    static double turn_noise__;
    // map
    Map *map__;
    // recorded robot pose data
    FILE *pose__;
    // robot sensor
    Sensor *sensor__;
};

#endif /* ROBOT_H */