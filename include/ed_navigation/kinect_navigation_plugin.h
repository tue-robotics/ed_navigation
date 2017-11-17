#ifndef ed_kinect_navigation_plugin_h_
#define ed_kinect_navigation_plugin_h_

#include <ed/plugin.h>
#include <ed/world_model.h>

#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include <cb_planner_msgs_srvs/LocalPlannerActionFeedback.h>

#include "../../src/depth_sensor_integrator.h"

/**
 * @brief The KinectNavigationPlugin class
 * ED plugin for de depth_sensor_integrator. Which uses the depth image to detect objects.
 * Based on the computed normals, points are computed that are probably measured due to an unmodeled object. These are published as a PointCloud.
 * THe plugin can be run in triggered mode. This way the pointcloud is only computed when the local_planner is active or at a backup.
 * To choose this mode the parameters should contains the trigger group. Otherwise all incoming images are computed
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
     * parametergroup: trigger (optional)
     * paramaters:
     *      local_planner_feedback_topic: /amigo/local_planner/action_server/feedback (required)
     *      trigger_duration: continue for x seconds after last trigger (optional)(default 1.0)
     *      backup_frequency: minimal frequency, when not triggered (optional)(default 1.0)
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
     * @param init
     * parameter:
     *
     */
    void initialize(ed::InitData& init);

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

    /**
     * @brief cb_queue_
     */
    ros::CallbackQueue cb_queue_;

    /**
     * @brief local_planner_subs_
     */
    ros::Subscriber local_planner_subs_;

    /**
     * @brief local_plannerCallback
     * @param msg
     */
    void local_plannerCallback(const cb_planner_msgs_srvs::LocalPlannerActionFeedbackConstPtr& msg);

    bool run_triggered_ = false;
    bool triggered_ = false;

    ros::Time last_trigger_time_;
    ros::Time last_update_time_;

    float backup_frequency_ = 1.0;
    float trigger_duration_ = 1.0;
};

#endif
