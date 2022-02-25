#ifndef INCLUDE_NL_KALMAN_BEACONS_H
#define INCLUDE_NL_KALMAN_BEACONS_H
#include <vector>

class Display;

struct Beacon_Data
{
    double x, y;
    int id;
    Beacon_Data()
    {
        x = 0.0;
        y = 0.0;
        id = -1;
    }
    Beacon_Data(double x_pos, double y_pos)
    {
        x = x_pos;
        y = y_pos;
        id = -1;
    }
    Beacon_Data(double x_pos, double y_pos, double bacon_id)
    {
        x = x_pos;
        y = y_pos;
        id = bacon_id;
    }
};

class Beacon_Map
{
public:
    Beacon_Map();

    void add_beacon(double x, double y);
    const Beacon_Data get_beacon_with_id(int id);
    const std::vector<Beacon_Data> get_beacons_with_range(double x, double y, double range);
    const std::vector<Beacon_Data> get_beacons();
    const void render(Display &disp);

private:
    std::vector<Beacon_Data> m_beacon_map;
};

#endif