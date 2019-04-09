#include "ed_navigation/navigation_plugin.h"

#include <ed/entity.h>
#include <ed/error_context.h>
#include <ed/convex_hull.h>
#include <ed/convex_hull_calc.h>

#include <tue/config/reader.h>

#include <geolib/datatypes.h>
#include <geolib/Shape.h>
#include <geolib/CompositeShape.h>

#include <ros/console.h>

#include <iomanip>

// ----------------------------------------------------------------------------------------------------

/**
 * @brief constructConstraint Construct a costraint which matches the border of the convexhull
 * @param chull with the points of the volume. Points of the volume, must follow the border of the volume.
 * Last point is connected to first point. The minimum number of points is 3.
 * @param constraint string with the contraint
 * @param offset offset to the constraint
 */
void constructConstraint(const ed::ConvexHull& chull, std::stringstream& constraint, const double offset = 0.0)
{
    if (chull.points.size() < 3)
    {
        std::cout << "Error: Convex hull has to consist of at least three points !!" << std::endl;
        return;
    }

    // To make sure we don't get e powers in the string
    constraint << std::fixed << std::setprecision(6);

    constraint << "(";

    double dx,dy,xi,yi,xs,ys,length;
    for (unsigned int i = 0; i < chull.points.size(); ++i)
    {
        if (i > 0)
            constraint << " and ";

        xi = chull.points[i].x;
        yi = chull.points[i].y;

        dx = ((i + 1 < chull.points.size()) ? chull.points[i+1].x : chull.points[0].x) - xi;
        dy = ((i + 1 < chull.points.size()) ? chull.points[i+1].y : chull.points[0].y) - yi;
        length = sqrt(dx * dx + dy * dy);

        xs = xi + (dy/length)*offset;
        ys = yi - (dx/length)*offset;

        constraint << "-(x-" << xs << ")*" << dy << "+(y-" << ys << ")*" << dx << " > 0";
    }

    constraint << ")";
}

// ----------------------------------------------------------------------------------------------------

//
/**
 * @brief constructShapeConstraint Construct a constraint based on the mesh of shape
 * @param shape shape of the volume in entity frame
 * @param entity_pose pose of the entity
 * @param offset offset to the constraint
 * @return constraint string
 */
std::string constructShapeConstraint(const geo::ShapeConstPtr& shape, const geo::Pose3D& entity_pose, const double offset = 0.0)
{
    std::stringstream shape_constraint;

    geo::Shape shape_tr;
    shape_tr.setMesh(shape->getMesh().getTransformed(entity_pose)); // get shape in map frame.
    std::vector<geo::Vector3> points = shape_tr.getMesh().getPoints();

    // Calculate cluster convex hull
    float z_min = 1e9;
    float z_max = -1e9;

    // Calculate z_min and z_max of cluster
    std::vector<geo::Vec2f> points_2d;
    for(std::vector<geo::Vector3>::const_iterator it = points.begin(); it != points.end(); ++it)
    {
        points_2d.push_back(geo::Vec2f(it->x, it->y));

        z_min = std::min<float>(z_min, it->z);
        z_max = std::max<float>(z_max, it->z);
    }

    ed::ConvexHull chull;
    ed::convex_hull::createAbsolute(points_2d, z_min, z_max, chull);

    constructConstraint(chull, shape_constraint, offset);
    return shape_constraint.str();
}

// ----------------------------------------------------------------------------------------------------

/**
 * @brief constructCompositeShapeConstraint Construct a constraint based on each sub shape of a CompositeShape
 * @param composite CompositeShape of the volume in entity frame
 * @param entity_pose pose of the entity
 * @param offset offset to the constraint. N.B.: this is applied to the children of the composite shape separately.
 * As a result, the shape constraint may be disjoint.
 * @return constraint string
 */
std::string constructCompositeShapeConstraint(geo::CompositeShapeConstPtr& composite, const geo::Pose3D& entity_pose,
                                              const double offset = 0.0)
{
    std::stringstream shape_constraint;

    bool first_sub_shape = true;

    const std::vector<std::pair<geo::ShapePtr, geo::Transform> >& sub_shapes = composite->getShapes();

    for (std::vector<std::pair<geo::ShapePtr, geo::Transform> >::const_iterator it = sub_shapes.begin();
         it != sub_shapes.end(); ++it)
    {
        geo::ShapeConstPtr ShapeC = boost::const_pointer_cast<geo::Shape>(it->first);
        std::string sub_shape_constraint = constructShapeConstraint(ShapeC, entity_pose * it->second.inverse(), offset);
        if (!sub_shape_constraint.empty())
        {
            if (first_sub_shape)
                first_sub_shape = false;
            else
                shape_constraint << " or ";

           shape_constraint << "(" << sub_shape_constraint << ")";
        }
    }

    return shape_constraint.str();
}

// ----------------------------------------------------------------------------------------------------

void NavigationPlugin::configure(tue::Configuration config)
{
    ros::NodeHandle nh("~/navigation");

    ros::AdvertiseServiceOptions opt_srv_get_goal_constraint =
            ros::AdvertiseServiceOptions::create<ed_navigation::GetGoalConstraint>(
                "get_constraint", boost::bind(&NavigationPlugin::srvGetGoalConstraint, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_get_goal_constraint_ = nh.advertiseService(opt_srv_get_goal_constraint);

    if (config.readGroup("constraint_service", tue::config::REQUIRED))
    {
        config.value("default_offset", default_offset_);
        if (!config.value("room_offset", room_offset_, tue::config::OPTIONAL))
            room_offset_ = 0.0;
        ROS_DEBUG_STREAM("[ED NAVIGATION] Default offset: " << default_offset_ << ", Room offset: " << room_offset_);
        config.endGroup();
    }

    // Configure the occupancy grid publisher
    if (config.readGroup("occupancy_grid_publisher"))
    {
        double res, min_z, max_z, unknown_obstacle_inflation;
        std::string frame_id;
        config.value("frame_id", frame_id);
        config.value("resolution", res);

        config.value("min_z", min_z);
        config.value("max_z", max_z);

        if (!config.value("unknown_obstacle_inflation", unknown_obstacle_inflation, tue::config::OPTIONAL))
            unknown_obstacle_inflation = 0.0;

        ROS_DEBUG_STREAM("[ED NAVIGATION] Using min_z: " << min_z << ",  max_z: " << max_z);
        occupancy_grid_publisher_.configure(nh, res, min_z, max_z, frame_id, unknown_obstacle_inflation);

        config.endGroup();
    }
}

// ----------------------------------------------------------------------------------------------------

void NavigationPlugin::initialize()
{

}

// ----------------------------------------------------------------------------------------------------

void NavigationPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    // Check for services
    world_ = &world;
    cb_queue_.callAvailable();

    // Publish the occupancy grid
    if (occupancy_grid_publisher_.configured())
        occupancy_grid_publisher_.publish(world);
}

// ----------------------------------------------------------------------------------------------------

/**
 * @brief NavigationPlugin::srvGetGoalConstraint Create a constraint to navigate to a specific volume.
 * @param req service request
 * @param res service result
 * @return bool
 */
bool NavigationPlugin::srvGetGoalConstraint(const ed_navigation::GetGoalConstraint::Request& req, ed_navigation::GetGoalConstraint::Response& res)
{
    if (req.entity_ids.size() != req.area_names.size())
    {
        res.error_msg = "Number of entity ids and number of volume names are not the same";
        return true;
    }

    std::stringstream constraint;
    constraint.precision(6); // To make sure we don't get e powers in the string
    constraint << std::fixed;

    bool first = true;

    for (unsigned int i = 0; i < req.entity_ids.size(); ++i)
    {
        ed::EntityConstPtr e = world_->getEntity(req.entity_ids[i]);

        double offset = 0.0;

        if (!e)
        {
            res.error_msg = "No such entity: '" + req.entity_ids[i] + "'.";
            continue;
        }

        if (!e->has_pose())
        {
            res.error_msg = "Entity '" + req.entity_ids[i] + "' has no pose.";
            continue;
        }

        std::stringstream entity_constraint;

        if(req.area_names[i] == "near")
        {
            if (!e->shape())
            {
                res.error_msg = "Navigating to area 'near' of entity '" + req.entity_ids[i] + "' isn't possible, because it doesn't have a shape.";
                continue;
            }
            std::vector<geo::Vec2f> points;

            for (std::vector<geo::Vec2f>::const_iterator it = e->convexHull().points.begin(); it != e->convexHull().points.end(); ++it)
                points.push_back(geo::Vec2f(it->x + e->pose().t.x, it->y + e->pose().t.y));
            ed::ConvexHull chull; // In MAP frame
            ed::convex_hull::createAbsolute(points, 0.0, 0.1, chull);
            offset = default_offset_;

            constructConstraint(chull, entity_constraint, offset);
        }
        else
        {
            std::map<std::string, geo::ShapeConstPtr>::const_iterator it = e->volumes().find(req.area_names[i]);
            if (it == e->volumes().end())
            {
                res.error_msg = "Entity '" + e->id().str() + "': volume '" + req.area_names[i] + "' does not exist";
                continue;
            }

            if (req.area_names[i] == "in")
                offset = room_offset_;

            std::string shape_constraint;
            geo::CompositeShapeConstPtr composite = boost::dynamic_pointer_cast<const geo::CompositeShape>(it->second);
            if (composite)
                shape_constraint = constructCompositeShapeConstraint(composite, e->pose(), offset);
            else
                shape_constraint = constructShapeConstraint(it->second, e->pose(), offset);

            entity_constraint << "(" << shape_constraint << ")";
        }

        // add to constraint here
        if (first)
            first = false;
        else
            constraint << " and ";

        constraint << "(" << entity_constraint.str() << ")";
    }


    res.position_constraint_map_frame = constraint.str();

    return true;
}

ED_REGISTER_PLUGIN(NavigationPlugin)



