#include "occupancy_grid_publisher.h"

//#include <ros/node_handle.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geolib/ros/msg_conversions.h>
#include <geolib/Shape.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/measurement.h>

#include <tue/config/reader.h>

// ----------------------------------------------------------------------------------------------------

void OccupancyGridPublisher::configure(ros::NodeHandle& nh, const double& res, const double& min_z, const double& max_z,
                                       const std::string& frame_id, double unknown_obstacle_inflation)
{
    convex_hull_enabled_ = true;

    map_.setResolution(res);

    frame_id_ = frame_id;

    min_z_ = min_z;
    max_z_ = max_z;

    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 0, false);

    unknown_obstacle_inflation_ = unknown_obstacle_inflation;

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
//        std::cout << "Error getting map data:" << std::endl;
//        std::cout << "width: " << width_ << std::endl;
//        std::cout << "height: " << height_ << std::endl;
//        std::cout << "resolution: " << res_ << std::endl;
//        std::cout << "min_z: " << min_z_ << std::endl;
//        std::cout << "max_z: " << max_z_ << std::endl;
    }
}

// ----------------------------------------------------------------------------------------------------

bool OccupancyGridPublisher::getMapData(const ed::WorldModel& world, std::vector<ed::EntityConstPtr>& entities_to_be_projected)
{
    // default size
    geo::Vector3 min(-10,-10,0);
    geo::Vector3 max(10,10,0);

    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (e->id().str() == "rack") //HACK
            continue;

        if (!e->has_pose() || e->existenceProbability() < 0.95 || e->hasFlag("self"))
            continue;

        //! Push back the entity
        entities_to_be_projected.push_back(e);

        //! Update the map bounds
        geo::ShapeConstPtr shape = e->shape();
        if (shape)  // Do shape
        {
            const std::vector<geo::Vector3>& vertices = shape->getMesh().getPoints();
            for(std::vector<geo::Vector3>::const_iterator it = vertices.begin(); it != vertices.end(); ++it) {

                geo::Vector3 p1w = e->pose() * (*it);

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
//        else if (convex_hull_enabled_)
//        {
//            if (e->convexHull().z_max + e->pose().t.z > min_z_)  // Filter the ground
//            {

//                const std::vector<geo::Vec2f>& chull_points = e->convexHull().points;
//                for(std::vector<geo::Vec2f>::const_iterator it = chull_points.begin(); it != chull_points.end(); ++it)
//                {
//                    float x = it->x + e->pose().t.x;
//                    float y = it->y + e->pose().t.y;

//                    min.x = std::min<double>(x - unknown_obstacle_inflation_, min.x);
//                    max.x = std::max<double>(x + unknown_obstacle_inflation_, max.x);

//                    min.y = std::min<double>(y - unknown_obstacle_inflation_, min.y);
//                    max.y = std::max<double>(y + unknown_obstacle_inflation_, max.y);

//                    is_empty = false;
//                }
//            }
//        }
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

    geo::ShapeConstPtr shape = e->shape();
    if (shape)  // Do shape
    {
        const std::vector<geo::Triangle>& triangles = shape->getMesh().getTriangles();

        for(std::vector<geo::Triangle>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {

            geo::Vector3 p1w = e->pose() * it->p1_;
            geo::Vector3 p2w = e->pose() * it->p2_;
            geo::Vector3 p3w = e->pose() * it->p3_;

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
//    else if (convex_hull_enabled_) // Do convex hull
//    {
//        if (e->convexHull().z_max + e->pose().t.z > min_z_ && e->convexHull().z_min + e->pose().t.z < max_z_)
//        {
//            const std::vector<geo::Vec2f>& chull_points = e->convexHull().points;
//
//            for (unsigned int i = 0; i < chull_points.size(); ++i)
//            {
//                int j = (i + 1) % chull_points.size();
//
//                geo::Vector3 p1w(chull_points[i].x + e->pose().t.x, chull_points[i].y + e->pose().t.y, 0);
//                geo::Vector3 p2w(chull_points[j].x + e->pose().t.x, chull_points[j].y + e->pose().t.y, 0);
//
//                // Check if all points are on the map
//                cv::Point2i p1, p2;
//                if (map.worldToMap(p1w.x, p1w.y, p1.x, p1.y) && map.worldToMap(p2w.x, p2w.y, p2.x, p2.y) )
//                    cv::line(map.image, p1, p2, value, unknown_obstacle_inflation_ / map.resolution() + 1);
//            }
//        }
//    }
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
