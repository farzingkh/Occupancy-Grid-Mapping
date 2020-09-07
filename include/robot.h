#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <exception>
#include "sensor.h"

struct State
{
    double time_stamp;
    double x;
    double y;
    double orientation;
};

class Robot
{
public:
    // Initialize robot in a random position in the world
    Robot(double width, double height, Sensor *sensor, Map *map = nullptr, FILE *pose = nullptr);
    // Set robot's states
    void set_states(double new_x, double new_y, double new_orient);
    // Set robots state transition noise
    void set_noise(double forward_noise, double turn_noise, double sensor_noise);
    // Sense the environment with sensors
    std::vector<double> sense(bool noise = true);
    // Move the robot
    void move(double turn, double forward);
    // Get pose readings
    State get_pose();
    // Get sensor readings in form of a string
    std::string get_sensor_readings();
    // get map
    Map *get_map();
    // get sensor object
    Sensor *get_sensor();
    // get robot dimensions
    double get_width();
    double get_height();


private:
    // robot size
    double robot_width__;
    double robot_height__;
    // robot states
    State states__;
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