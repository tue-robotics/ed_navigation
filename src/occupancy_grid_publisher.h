#ifndef occupancy_grid_publisher_h_
#define occupancy_grid_publisher_h_

#include <ros/ros.h>
#include <geolib/datatypes.h>
#include <ed/plugin.h>
#include <ed/types.h>

#include <opencv2/opencv.hpp>

class OccupancyGridPublisher
{

public:

    OccupancyGridPublisher() : width_(0), height_(0), res_(0), configured_(false) {}

    void configure(ros::NodeHandle& nh, const double &res, const double& min_z, const double& max_z, const std::string &frame_id);

    void publish(const ed::WorldModel& world);

    bool configured() { return configured_; }

private:

    bool getMapData(const ed::WorldModel& world, std::vector<ed::EntityConstPtr>& entities_to_be_projected);

    void updateMap(const ed::EntityConstPtr& e, cv::Mat& map);

    void publishMapMsg (const cv::Mat& map);

    bool worldToMap (double wx, double wy, int& mx, int& my) const
    {
        if (wx < origin_.x || wy < origin_.y)
            return false;

        mx = (wx - origin_.x) / res_ ;
        my = (wy - origin_.y) / res_ ;

        if (mx < width_ && my < height_)
            return true;

        return false;
    }

    void mapToWorld (unsigned int mx, unsigned int my, double& wx, double& wy) const
    {
        wx = origin_.x + (mx + 0.5) * res_;
        wy = origin_.y + (my + 0.5) * res_;
    }

    ros::Publisher map_pub_;

    //! Configurarable parameters  --------------

    bool configured_;

    geo::Vector3 origin_;
    int width_, height_;
    std::string frame_id_;

    double res_, min_z_, max_z_;
};

#endif
