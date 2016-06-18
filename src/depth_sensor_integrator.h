#ifndef _DEPTH_SENSOR_INTEGRATOR_H_
#define _DEPTH_SENSOR_INTEGRATOR_H_

#include "map.h"

#include <ed/kinect/image_buffer.h>
#include <tue/config/configuration.h>

#include <ros/publisher.h>

class DepthSensorIntegrator
{

public:

    DepthSensorIntegrator();

    ~DepthSensorIntegrator();

    void initialize(tue::Configuration config, const std::string& map_frame);

    bool updateMap(Map& map);

    bool isInitialized() const { return !map_frame_.empty(); }

private:

    ImageBuffer image_buffer_;


    // Params

    std::string map_frame_;

    double slope_threshold_;
    double min_distance_;
    double max_distance_;

    int num_samples_;

    int slope_window_size_;

    ros::Publisher pointcloud2_publisher_;

};

#endif
