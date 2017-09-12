#include <baxter_mover_utils/baxter_mover.hpp>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.h>
#include <moveit/depth_image_octomap_updater/depth_image_octomap_updater.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_world.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

/*
using namespace baxter_mover;
double x = 0.65, y = 0.2, z = 0.1;

void planning_scene_cb(const moveit_msgs::PlanningSceneConstPtr& monitored_planning_scene){
    const collision_detection::World::ObjectConstPtr object = monitored_planning_scene->world.octomap.octomap;
    //const collision_detection::World::ObjectConstPtr object = new collision_detection::World::Object(monitored_planning_scene->world.octomap);
    octomap::AbstractOcTree* oc_tree = octomap_msgs::binaryMsgToMap(monitored_planning_scene->world.octomap.octomap);

    ROS_WARN_STREAM("The tree type is: " << oc_tree->getTreeType());
    oc_tree->getMetricMax(x, y, z);
    ROS_WARN_STREAM("The output of getMetricMax is: X = " << x << ", Y = " << y << ", and Z = " << z);
    ROS_INFO("***********************************************************");
//    moveit_msgs::
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_octomap_node");
    ros::NodeHandle node;

    ros::Subscriber psm_sub = node.subscribe<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1, planning_scene_cb);


    ros::Rate my_rate(1);
    while (ros::ok()) {
       ros::spinOnce();
       my_rate.sleep();
    }
    return 0;
}*/



class tf_publisher{
public:
    void init(){
        _stamped_transform_publisher = _nh.advertise<geometry_msgs::TransformStamped>("/object_position_base_frame", 1000);
        _object_update_sub = _nh.subscribe<tf2_msgs::TFMessage>("/tf", 10, &tf_publisher::tf_callback, this);

        ros::AsyncSpinner my_spinner(4);
        my_spinner.start();
    }

    void publish_point_frame(std::string child_frame_id){
        // publish transform
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(point.point.x,
                                         point.point.y,
                                         point.point.z));

        tf::Quaternion q(0.,
                         0.,
                         0.,
                         1.0);

        // Handle different coordinate systems (Arena vs. rviz)
        transform.setRotation(q);
        ros::Time timestamp(ros::Time::now());
        tf_pub.sendTransform(tf::StampedTransform(transform, timestamp, "camera_rgb_optical_frame", child_frame_id));
    }

    void tf_callback(const tf2_msgs::TFMessageConstPtr& tf_msgs){
        point.header.frame_id = "camera_rgb_optical_frame";/*
        point.point.x = rand() / (RAND_MAX + 1.0);
        point.point.y = rand() / (RAND_MAX + 1.0);
        point.point.z = rand() / (RAND_MAX + 1.0);*/

        point.point.x = 0.3;
        point.point.y = 0.2;
        point.point.z = 1.0;

        //ROS_WARN_STREAM("The choosen point in camera frame is: X = " << point.point.x << ", Y = " << point.point.y << ", and Z = " << point.point.z);

        publish_point_frame("/object_base_frame");
    }

    void tf_base_conversion(std::string child_frame_id){
        tf::StampedTransform transform;
        tf::TransformListener listener;
        std::string parent_frame = "/base";
        try{
            listener.lookupTransform(parent_frame, child_frame_id, ros::Time(0), transform);
            tf::transformStampedTFToMsg(transform, msg);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    void publish(){
        _stamped_transform_publisher.publish(msg);
    }

private:
    ros::NodeHandle _nh;
    ros::Publisher _stamped_transform_publisher;
    ros::Subscriber _object_update_sub;
    geometry_msgs::PointStamped point;
    tf::TransformBroadcaster tf_pub;
    tf::StampedTransform stamped_transform;
    geometry_msgs::TransformStamped msg;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_octomap_node");
    ros::NodeHandle node;

    ros::Publisher stamped_transform_publisher = node.advertise<geometry_msgs::TransformStamped>("/object_position_frame", 1000);
    tf::StampedTransform transform;
    tf::TransformListener listener;
    geometry_msgs::TransformStamped msg;
    std::string parent_frame = "/base";
    std::string child_frame_id = "/object_base_frame";

    tf_publisher my_tf_publisher;
    my_tf_publisher.init();

    ros::Rate my_rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        try{
            listener.lookupTransform(parent_frame, child_frame_id, ros::Time(0), transform);
            tf::transformStampedTFToMsg(transform, msg);
            stamped_transform_publisher.publish(msg);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        my_rate.sleep();
    }
    return 0;
}
