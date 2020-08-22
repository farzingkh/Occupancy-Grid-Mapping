#ifndef OCCGRIDMAPPING_H
#define OCCGRIDMAPPING_H

#include "robot.h"

class Mapping
{
public:
    Mapping(Robot *r);

    void occ_grid_mapping();
    void MAP_occ_grid_mapping();

    void set_lo(double lo);
    void set_lfree(double l_free);
    void set_locc(double l_occ);

    void print_map();

private:
    Robot *robot__;
    // log odd values
    double l_o__ = 0;
    double l_free__ = -0.4;
    double l_occ__ = 0.4;
};

#endif /* OCCGRIDMAPPING_H */