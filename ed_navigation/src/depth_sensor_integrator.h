#ifndef _DEPTH_SENSOR_INTEGRATOR_H_
#define _DEPTH_SENSOR_INTEGRATOR_H_

#include "map.h"

#include <ed/kinect/image_buffer.h>
#include <tue/config/configuration.h>

#include <ros/publisher.h>

/**
 * @brief A depth image is converted to a pointcloud based on the normal. This is used for detection of obstacles
 */
class DepthSensorIntegrator
{

public:
    /**
     * @brief constructor
     */
    DepthSensorIntegrator();

    /**
     * @brief destructor
     */
    ~DepthSensorIntegrator();

    /**
     * @brief initialize
     * @param config tue::Configuration
     */
    void initialize(tue::Configuration config);

    /**
     * @brief New depth image is converted pointcloud
     * @return If update is executed correctly
     */
    bool update();

    /**
     * @brief isInitialized
     * @return If the instance is initialized
     */
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
