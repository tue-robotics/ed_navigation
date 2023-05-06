#include "ed/navigation/occupancy_grid_publisher.h"

#include <nav_msgs/OccupancyGrid.h>

#include <geolib/ros/msg_conversions.h>
#include <geolib/Shape.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/measurement.h>

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include <tue/config/reader.h>

namespace ed
{

namespace navigation
{

// ----------------------------------------------------------------------------------------------------

void OccupancyGridPublisher::configure(ros::NodeHandle& nh, tue::Configuration config)
{
    double resolution = 0.;
    config.value("frame_id", frame_id_);
    config.value("resolution", resolution);

    config.value("min_z", min_z_);
    config.value("max_z", max_z_);

    min_map_size_x_ = 20;
    min_map_size_y_ = 20;
    config.value("min_map_size_x", min_map_size_x_, tue::config::OPTIONAL);
    config.value("min_map_size_y", min_map_size_y_, tue::config::OPTIONAL);

    ROS_DEBUG_STREAM("[ED NAVIGATION] Using:" << std::endl
                     << "min_z: " << min_z_ << std::endl
                     << "max_z: " << max_z_ << std::endl
                     << "min_map_size_x: " << min_map_size_x_ << std::endl
                     << "min_map_size_y: " << min_map_size_y_ << std::endl
                     << "frame_id: " << frame_id_ << std::endl
                     << "resolution: " << resolution);

    map_.setResolution(resolution);

    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 0, false);

    configured_ = true;
}

// ----------------------------------------------------------------------------------------------------

void OccupancyGridPublisher::publish(const ed::WorldModel& world)
{
    std::vector<ed::EntityConstPtr> entities_to_be_projected;
    if (getMapData(world, entities_to_be_projected))
    {
        for(std::vector<ed::EntityConstPtr>::const_iterator it = entities_to_be_projected.begin(); it != entities_to_be_projected.end(); ++it)
            updateMap(*it, map_);

        publishMapMsg(map_);
    }
    else
    {
        ROS_DEBUG_STREAM("Error getting map data:" << std::endl
                         << "width: " << map_.width_in_cells() << std::endl
                         << "height: " << map_.height_in_cells() << std::endl
                         << "resolution: " << map_.resolution() << std::endl
                         << "min_z: " << min_z_ << std::endl
                         << "max_z: " << max_z_);
    }
}

// ----------------------------------------------------------------------------------------------------

bool OccupancyGridPublisher::getMapData(const ed::WorldModel& world, std::vector<ed::EntityConstPtr>& entities_to_be_projected)
{
    // default size
    geo::Vector3 min(-min_map_size_x_/2,-min_map_size_y_/2,0);
    geo::Vector3 max(min_map_size_x_/2,min_map_size_y_/2,0);

    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        geo::ShapeConstPtr shape = e->collision();

        if (!e->has_pose() || !shape || e->existenceProbability() < 0.95 || e->hasFlag("self"))
            continue;

        //! Push back the entity
        entities_to_be_projected.push_back(e);

        //! Update the map bounds
        const std::vector<geo::Vector3>& vertices = shape->getMesh().getPoints();
        for(std::vector<geo::Vector3>::const_iterator it2 = vertices.begin(); it2 != vertices.end(); ++it2)
        {
            geo::Vector3 p1w = e->pose() * (*it2);

            // Filter the ground
            if (p1w.getZ() > min_z_)
            {
                min.x = std::min(p1w.x, min.x);
                max.x = std::max(p1w.x, max.x);

                min.y = std::min(p1w.y, min.y);
                max.y = std::max(p1w.y, max.y);
            }
        }
    }

    // Bounds fix
    min.x-=1.0;
    min.y-=1.0;

    max.x+=1.0;
    max.y+=1.0;

    //! Set the origin, width and height
    map_.setOrigin(min);

    map_.setSizeAndClear((max.x - min.x) / map_.resolution(), (max.y - min.y) / map_.resolution());

    return (map_.width_in_cells() > 0 && map_.height_in_cells() > 0 && map_.width_in_cells() * map_.height_in_cells() < 100000000);
}

// ----------------------------------------------------------------------------------------------------

void OccupancyGridPublisher::updateMap(const ed::EntityConstPtr& e, Map& map)
{
    int value = 100;

    geo::ShapeConstPtr shape = e->collision();
    if (shape)  // Do shape
    {
        const std::vector<geo::Triangle>& triangles = shape->getMesh().getTriangles();

        for(std::vector<geo::Triangle>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {

            geo::Vector3 p1w = e->pose() * it->p1();
            geo::Vector3 p2w = e->pose() * it->p2();
            geo::Vector3 p3w = e->pose() * it->p3();

            // Filter if all points are above or all points are below
            if ( (p1w.getZ() < min_z_ && p2w.getZ() < min_z_ && p3w.getZ() < min_z_) || (p1w.getZ() > max_z_ && p2w.getZ() > max_z_ && p3w.getZ() > max_z_) )
                continue;

            cv::Point2i p1, p2, p3;

            // Check if all points are on the map
            if (map.worldToMap(p1w.x, p1w.y, p1.x, p1.y) && map.worldToMap(p2w.x, p2w.y, p2.x, p2.y) && map.worldToMap(p3w.x, p3w.y, p3.x, p3.y) ) {
                cv::line(map.image, p1, p2, value);
                cv::line(map.image, p1, p3, value);
                cv::line(map.image, p2, p3, value);
            }

        }
    }
}

// ----------------------------------------------------------------------------------------------------

void OccupancyGridPublisher::publishMapMsg(const Map& map)
{
    nav_msgs::OccupancyGrid map_msg;
    geo::convert(map.origin(), map_msg.info.origin.position);

    map_msg.info.resolution = map.resolution();
    map_msg.info.width = map.width_in_cells();
    map_msg.info.height = map.height_in_cells();

    map_msg.data.resize(map.width_in_cells() * map.height_in_cells());

    unsigned int i = 0;
    for(int my = 0; my < map.height_in_cells(); ++my)
    {
        for(int mx = 0; mx < map.width_in_cells(); ++mx)
        {
            unsigned char c = map.image.at<unsigned char>(my, mx);
            if (c > 0)
                map_msg.data[i] = c;
            else
                map_msg.data[i] = -1;
            ++i;
        }
    }

    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = frame_id_;
    map_pub_.publish(map_msg);
}

// ----------------------------------------------------------------------------------------------------

} // namespace navigation

} // namespace ed

