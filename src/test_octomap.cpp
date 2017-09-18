#include <baxter_mover_utils/baxter_mover.hpp>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.h>
#include <moveit/depth_image_octomap_updater/depth_image_octomap_updater.h>

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>

#include <octomap_msgs/conversions.h>

#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_world.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <moveit/occupancy_map_monitor/occupancy_map.h>
#include <octomap_msgs/OctomapWithPose.h>

using namespace baxter_mover;
double x, y, z;

void planning_scene_cb(const moveit_msgs::PlanningSceneConstPtr& psm){
    //moveit_msgs::CollisionObject test;

    geometry_msgs::Point my_point;

    octomap::point3d my_octomap_point = octomap::pointMsgToOctomap(my_point);

    //octomap::OcTree::
    ROS_INFO_STREAM("OCTOMAP TESTER : Trying to print collision objects stuff, with size of: " << psm->world.collision_objects.size());
    for(size_t i = 0; i < psm->world.collision_objects.size(); i++)
        ROS_WARN_STREAM("OCTOMAP TESTER : For object: " << i << " the id is: " << psm->world.collision_objects[i].id);

    //occupancy_map_monitor::OccMapTreePtr my_tree;


    ROS_WARN_STREAM("OCTOMAP TESTER : Octomap id is : " << psm->world.octomap.octomap.id);

    if(strcmp(psm->world.octomap.octomap.id.c_str(), "OcTree") == 0){
        ROS_WARN_STREAM("OCTOMAP TESTER : Octomap frame id is : " << psm->world.octomap.header.frame_id);
        ROS_WARN_STREAM("OCTOMAP TESTER : Octomap resolution is : " << psm->world.octomap.octomap.resolution);
        ROS_WARN_STREAM("OCTOMAP TESTER : Octomap size is : " << psm->world.octomap.octomap.data.size());

        ROS_WARN_STREAM("OCTOMAP TESTER : Octomap binary is : " << psm->world.octomap.octomap.binary);
        //psm->world.octomap.octomap.binary = true;


        //octomap_msgs::Octomap msg;
        //octomap_msgs::msgToMap()

        //if(octomap_msgs::binaryMapToMsg(psm->world.octomap.octomap, msg)){
        octomap::AbstractOcTree* oc_tree = octomap_msgs::msgToMap(psm->world.octomap.octomap);




        if(oc_tree == NULL){
            ROS_ERROR("OCTOMAP TESTER : The tree is null !!!");
            return;
        }

        std::vector<int8_t> data_vector;
        octomap_msgs::fullMapToMsgData(*oc_tree, data_vector);
//        for(size_t i = 0; i < data_vector.size(); i++)
//            ROS_INFO_STREAM("OCTOMAP TESTER : Element number: " << i << " is: " << data_vector[i]);

        ROS_INFO("OCTOMAP TESTER : Trying to read octree " );
        octomap::OcTree* my_octree;
        octomap_msgs::readTree(my_octree, psm->world.octomap.octomap);
        ROS_INFO("OCTOMAP TESTER : Octree read" );
        my_octree->writeData(std::cout);
        ROS_INFO("OCTOMAP TESTER : Octree wrote" );
        oc_tree->getMetricMax(x, y, z);
        ROS_WARN_STREAM("OCTOMAP TESTER : The output of getMetricMax is: X = " << x << ", Y = " << y << ", and Z = " << z);
        ROS_WARN_STREAM("OCTOMAP TESTER : The tree type is: " << oc_tree->getTreeType());
        //oc_tree->writeData(std::cout);
        ROS_INFO("***********************************************************");
        oc_tree->clear();
    }
    //}
    //    my_tree.reset(new occupancy_map_monitor::OccMapTree(monitored_planning_scene->world.octomap.octomap));
    //    my_tree.reset(new occupancy_map_monitor::OccMapTree(monitored_planning_scene->world.octomap.octomap.resolution));
    //    octomap::AbstractOcTree;
    //    const collision_detection::World::ObjectConstPtr object(new );
    //    collision_detection::Contact my_contact;

    //    const collision_detection::World::ObjectConstPtr object = new collision_detection::World::Object(monitored_planning_scene->world.octomap);

    //    octomap_msgs::OctomapWithPose my_octomap = monitored_planning_scene->world.octomap;

    //
    //
    //
    //    if (!object)
    //    {
    //      ROS_ERROR("No valid Object passed in, cannot refine Normals!");
    //      return ;
    //    }
    //    if (!object->shapes_.empty())
    //    {
    //        const shapes::ShapeConstPtr& shape = object->shapes_[0];
    //        boost::shared_ptr<const shapes::OcTree> shape_octree = boost::dynamic_pointer_cast<const shapes::OcTree>(shape);
    //        if (shape_octree)
    //        {
    //            octomap::OcTreeBaseImpl<octomap::OcTreeNode, octomap::AbstractOccupancyOcTree>::tree_iterator it = shape_octree->octree->begin_tree();
    //        }
    //    }
    //    else{
    //        ROS_ERROR("The collision object variable is empty !!!");
    //        return;
    //    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_octomap_node");
    ros::NodeHandle node;

    ros::Subscriber psm_sub = node.subscribe<moveit_msgs::PlanningScene>("/dream_babbling/controller_node/move_group/monitored_planning_scene", 1, planning_scene_cb);


    ros::Rate my_rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        my_rate.sleep();
    }
    return 0;
}

