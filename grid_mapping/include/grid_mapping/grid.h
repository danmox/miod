#ifndef GRID_MAPPING_GRID_H_
#define GRID_MAPPING_GRID_H_

#include "grid_mapping/grid_base.h"

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <iostream>
#include <string.h>


namespace grid_mapping {


template <class T>
class Grid : public GridBase
{
  public:
    std::vector<T> data;
    std::string frame_id;

    Grid(Point, double, int, int, bool = true);
    Grid(const nav_msgs::OccupancyGrid::ConstPtr&);

    virtual void update(const Grid*);
    virtual T updateCell(T, T);

    void expandMap(const Point, const Point);
    void insertMap(const nav_msgs::OccupancyGrid::ConstPtr&);

    nav_msgs::OccupancyGrid createROSMsg() const;

    template <class H>
    friend std::ostream& operator<<(std::ostream& out, const Grid<H>& grid);
};


} // namespace grid_mapping


#endif
