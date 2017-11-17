#include "ed_navigation/kinect_navigation_plugin.h"

#include <ed/entity.h>
#include <ed/error_context.h>

#include <ros/node_handle.h>

#include <tue/config/reader.h>

#include <iomanip>

// ----------------------------------------------------------------------------------------------------

void KinectNavigationPlugin::configure(tue::Configuration config)
{
    // configure the depth_sensor integration
    if (config.readGroup("depth_sensor_integration"))
    {
        depth_sensor_integrator_.initialize(config);
        config.endGroup();
    }
}

// ----------------------------------------------------------------------------------------------------

void KinectNavigationPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;
    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);

    //check if local_planner_topic is provided for triggered run
    if (config.readGroup("trigger", tue::OPTIONAL))
    {
        std::string local_planner_feedback_topic;
        if (config.value("local_planner_feedback_topic", local_planner_feedback_topic, tue::REQUIRED))
        {
            run_triggered_ = true;
            local_planner_subs_ = nh.subscribe<cb_planner_msgs_srvs::LocalPlannerActionFeedback>(local_planner_feedback_topic, 1, &KinectNavigationPlugin::local_plannerCallback, this);

            config.value("trigger_duration", trigger_duration_, tue::OPTIONAL);
            config.value("backup_frequency", backup_frequency_, tue::OPTIONAL);

            last_trigger_time_, last_trigger_time_ = ros::Time::now();
            ROS_INFO_STREAM("Plugin [" << this->name() << "] running in triggered mode with trigger duration: '" << trigger_duration_ << "' and backup_frequency: '" << backup_frequency_ << "'.");
            config.endGroup();
        }
    }
    else
    {
        ROS_INFO_STREAM("Plugin [" << this->name() << "] not running in triggered mode");
        //not running triggered, so triggered is always true
        triggered_ = true;
    }
}

// ----------------------------------------------------------------------------------------------------

void KinectNavigationPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    if (run_triggered_)
    {
        cb_queue_.callAvailable();

        ros::Time time_now = ros::Time::now();
        ros::Rate rate = ros::Rate(backup_frequency_);

        if (last_trigger_time_ + ros::Duration(trigger_duration_) >= time_now || last_update_time_ + ros::Duration(rate) <= time_now)
        {
            ROS_DEBUG_STREAM("Plugin [" << this->name() << "] Time since last update: " << time_now-last_update_time_);
            triggered_ = true;
        }
    }

    // Publish the occupancy grid
    if (depth_sensor_integrator_.isInitialized())
        if (triggered_)
        {
            depth_sensor_integrator_.update();
            if (run_triggered_)
            {
                last_update_time_ = ros::Time::now();
                triggered_ = false;
            }
        }
}

// ----------------------------------------------------------------------------------------------------

void KinectNavigationPlugin::local_plannerCallback(const cb_planner_msgs_srvs::LocalPlannerActionFeedbackConstPtr& msg)
{
    triggered_ = true;
    last_trigger_time_ = ros::Time::now();
}

// ----------------------------------------------------------------------------------------------------


ED_REGISTER_PLUGIN(KinectNavigationPlugin)
