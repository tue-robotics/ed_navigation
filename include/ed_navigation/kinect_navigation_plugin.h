#ifndef ed_kinect_navigation_plugin_h_
#define ed_kinect_navigation_plugin_h_

#include <ed/plugin.h>
#include <ed/world_model.h>

#include "../../src/depth_sensor_integrator.h"

/**
 * @brief The KinectNavigationPlugin class
 * ED plugin for de depth_sensor_integrator. Which uses the depth image to detect objects
 */
class KinectNavigationPlugin : public ed::Plugin
{

public:

    /**
     * @brief constructor
     */
    KinectNavigationPlugin() {}

    /**
     * @brief destructor
     */
    virtual ~KinectNavigationPlugin() {}

    /**
     * @brief configure
     * @param config
     * @
     * parametergroup: depth_sensor_integration
     * parameters:
     *      frame_id: /map
     *      topic: /amigo/top_kinect/rgbd
     *      num_samples: 640
     *      slope_threshold: 1
     *      slope_window_size: 30
     *      min_distance: 0.4
     *      max_distance: 2.0
     */
    void configure(tue::Configuration config);

    /**
     * @brief initialize
     */
    void initialize();

    /**
     * @brief process
     * @param world
     * @param req
     */
    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

    // --------------------

private:

    /**
     * @brief depth_sensor_integrator_
     */
    DepthSensorIntegrator depth_sensor_integrator_;

};

#endif
