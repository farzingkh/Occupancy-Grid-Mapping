#ifndef OCCGRIGMAPPING_H
#define OCCGRIDMAPPING_H

class Mapping
{
public:
    Mapping(Robot *r);
    void occ_grid_mapping();
    void MAP_occ_grid_mapping();
private:
    Sensor* sensor__;
};

#endif /* OCCGRIDMAPPING_H */