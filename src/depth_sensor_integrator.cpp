#include "depth_sensor_integrator.h"

#include "timer.h"

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <geolib/ros/tf_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <ros/node_handle.h>

// ----------------------------------------------------------------------------------------------------

// Decomposes 'pose' into a (X, Y, YAW) and (Z, ROLL, PITCH) component
void decomposePose(const geo::Pose3D& pose, geo::Pose3D& pose_xya, geo::Pose3D& pose_zrp)
{
    tf::Matrix3x3 m;
    geo::convert(pose.R, m);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose_xya.R.setRPY(0, 0, yaw);
    pose_xya.t = geo::Vec3(pose.t.x, pose.t.y, 0);

    pose_zrp = pose_xya.inverse() * pose;
}

// ----------------------------------------------------------------------------------------------------

DepthSensorIntegrator::DepthSensorIntegrator()
{
}

// ----------------------------------------------------------------------------------------------------

DepthSensorIntegrator::~DepthSensorIntegrator()
{
}

// ----------------------------------------------------------------------------------------------------

void DepthSensorIntegrator::initialize(tue::Configuration config)
{
    std::string rgbd_topic;
    if (config.value("topic", rgbd_topic))
        image_buffer_.initialize(rgbd_topic);

    config.value("num_samples", num_samples_);
    config.value("slope_threshold", slope_threshold_);
    config.value("slope_window_size", slope_window_size_);
    config.value("min_distance", min_distance_);
    config.value("max_distance", max_distance_);
    config.value("frame_id", map_frame_);

    ros::NodeHandle nh("~");
    pointcloud2_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("navigation/cloud", 20);
}

// ----------------------------------------------------------------------------------------------------

bool DepthSensorIntegrator::update()
{
    if (!isInitialized())
        return false;

    bool visualize = false;

    rgbd::ImageConstPtr image;

    geo::Pose3D sensor_pose;
    if (!image_buffer_.nextImage(map_frame_, image, sensor_pose))
        return false;

    // - - - - - - - - - - - - - - - - - - - - - - -

    geo::Pose3D sensor_pose_xya;
    geo::Pose3D sensor_pose_zrp;
    decomposePose(sensor_pose, sensor_pose_xya, sensor_pose_zrp);


    // - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<geo::Vector3> measurements;

    Timer timer;

    cv::Mat depth = image->getDepthImage();

    int width = depth.cols;
    int height = depth.rows;

    rgbd::View view(*image, width);
    const geo::DepthCamera& rasterizer = view.getRasterizer();

    cv::Mat obstacle_map, bla;
    if (visualize)
    {
        obstacle_map = cv::Mat(500, 500, CV_32FC1, 0.0);
        bla = depth.clone();
    }

    cv::Mat buffer(height, 1, CV_64FC4, 0.0);
    std::vector<geo::Vector3> p_floors(height);

    double x_step = 1;
    if (num_samples_ > 0 && num_samples_ < depth.cols)
        x_step = (double)depth.cols / num_samples_;

    for(double x = 0; x < width; x += x_step)
    {
        geo::Vector3 p_floor_closest(1e6, 1e6, 0);

        buffer.at<cv::Vec4d>(0, 0) = cv::Vec4d(0, 0, 0, 0);

        for(int y = 1; y < height; ++y)
        {
            float d = depth.at<float>(y, x);
            if (d == 0 || d != d)
            {
                buffer.at<cv::Vec4d>(y, 0) = buffer.at<cv::Vec4d>(y - 1, 0);
                continue;
            }

            geo::Vector3 p_sensor = rasterizer.project2Dto3D(x, y) * d;
            p_floors[y] = sensor_pose_zrp * p_sensor;
            const geo::Vector3& p_floor = p_floors[y];

            if (p_floor.z > 0.2)
            {
                if (p_floor.y < p_floor_closest.y)
                    p_floor_closest = p_floor;

                if (visualize)
                {
                    cv::Point2i p(p_floor.x * 50 + obstacle_map.rows / 2,
                                  obstacle_map.cols / 2 - p_floor.y * 50);
                    if (p.x >= 0 && p.x < obstacle_map.cols && p.y >= 0 && p.y < obstacle_map.rows)
                        obstacle_map.at<float>(p) = 0.5;
                }
            }

            //            x_sum_ += x;
            //            y_sum_ += y;
            //            xy_sum_ += x * y;
            //            x2_sum_ += x * x;

            buffer.at<cv::Vec4d>(y, 0) = buffer.at<cv::Vec4d>(y - 1, 0)
                        + cv::Vec4d(p_floor.y, p_floor.z, p_floor.y * p_floor.z, p_floor.y * p_floor.y);
        }

        for(int y = slope_window_size_; y < height - slope_window_size_; ++y)
        {
            float d = depth.at<float>(y, x);
            if (d == 0 || d != d)
                continue;

            const geo::Vector3& p_floor = p_floors[y];

            if (p_floor.z < 0.2)
            {
//                const cv::Vec4d& bm = buffer.at<cv::Vec4d>(y, 0);

//                double c1, s1;
//                cv::Vec4d b1 = bm - buffer.at<cv::Vec4d>(y - slope_window_size_, 0);
//                s1 = (slope_window_size_ * b1[2] - b1[0] * b1[1]) / (slope_window_size_ * b1[3] - b1[0] * b1[0]);
                //            c1 = b1[1] / slope_window_size_ - s1 * (b1[0] / slope_window_size_);

                double c2, s2;
                cv::Vec4d b2 = buffer.at<cv::Vec4d>(y + slope_window_size_ / 2, 0)
                        - buffer.at<cv::Vec4d>(y - slope_window_size_ / 2, 0);
                s2 = (slope_window_size_ * b2[2] - b2[0] * b2[1]) / (slope_window_size_ * b2[3] - b2[0] * b2[0]);
                //            c2 = b2[1] / slope_window_size_ - s1 * (b2[0] / slope_window_size_);

//                if (std::abs(s1) > 1 && std::abs(s2) < 1)
                if (std::abs(s2) > slope_threshold_)
                {
                    if (p_floor.y < p_floor_closest.y)
                        p_floor_closest = p_floor;

                    if (visualize)
                    {
                        bla.at<float>(y, x) = 10;
                        cv::Point2i p(p_floor.x * 50 + obstacle_map.rows / 2,
                                      obstacle_map.cols / 2 - p_floor.y * 50);
                        if (p.x >= 0 && p.x < obstacle_map.cols && p.y >= 0 && p.y < obstacle_map.rows)
                            obstacle_map.at<float>(p) = 1;
                    }
                }
            }
        }

        if (p_floor_closest.y < max_distance_ && p_floor_closest.y > min_distance_)
        {
            geo::Vec3 p_map = sensor_pose_xya * p_floor_closest;
            if (p_map.z > 1.7)
                continue;

            measurements.push_back(p_map);

//            cv::Point p_cv;
//            if (map.worldToMap(p_map.x, p_map.y, p_cv.x, p_cv.y))
//            {
//                map.image.at<unsigned char>(p_cv) = 100;
//            }
        }
    }

    sensor_msgs::PointCloud2 pointcloud2_msg;
    pointcloud2_msg.header.frame_id = "/map";
    pointcloud2_msg.header.stamp = ros::Time::now();

    pointcloud2_msg.height = 1;

    pointcloud2_msg.fields.resize(3);

    pointcloud2_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    pointcloud2_msg.fields[0].name = "x";
    pointcloud2_msg.fields[0].offset = 0;
    pointcloud2_msg.fields[0].count = 1;

    pointcloud2_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    pointcloud2_msg.fields[1].name = "y";
    pointcloud2_msg.fields[1].offset = 4;
    pointcloud2_msg.fields[1].count = 1;

    pointcloud2_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    pointcloud2_msg.fields[2].name = "z";
    pointcloud2_msg.fields[2].offset = 8;
    pointcloud2_msg.fields[2].count = 1;

    pointcloud2_msg.point_step = 12;

    pointcloud2_msg.data.resize(pointcloud2_msg.point_step * measurements.size());
    pointcloud2_msg.row_step = pointcloud2_msg.point_step * measurements.size();
    pointcloud2_msg.width = measurements.size();

    unsigned int i = 0;
    for (std::vector<geo::Vector3>::const_iterator it = measurements.begin(); it != measurements.end(); ++it)
    {
        float x = it->x;
        float y = it->y;
        float z = 0.1;

        // Copy x y z values to byte array
        memcpy(pointcloud2_msg.data.data() + i * pointcloud2_msg.point_step    , reinterpret_cast<uint8_t*>(&x), 4);
        memcpy(pointcloud2_msg.data.data() + i * pointcloud2_msg.point_step + 4, reinterpret_cast<uint8_t*>(&y), 4);
        memcpy(pointcloud2_msg.data.data() + i * pointcloud2_msg.point_step + 8, reinterpret_cast<uint8_t*>(&z), 4);
        ++i;
    }

    pointcloud2_publisher_.publish(pointcloud2_msg);

    timer.stop();
//    std::cout << "DepthSensorIntegrator: " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

    if (visualize)
    {
        cv::imshow("bla", bla / 10);
        cv::imshow("obstacle map", obstacle_map);
        cv::waitKey(3);
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

