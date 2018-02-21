#include <baxter_mover_utils/baxter_mover.hpp>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>

using namespace baxter_mover;

int main(int argc, char **argv)
    {
        ros::init(argc, argv, "test_baxter_utils");
        ros::NodeHandle node;
        BAXTER_Mover::Ptr _my_test;
        _my_test.reset(new BAXTER_Mover(node));

        moveit::planning_interface::MoveGroup::Plan plan;

        //ros::AsyncSpinner my_spinner(1);
        //my_spinner.start();

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "base";
        target_pose.pose.position.x = 0.65;
        target_pose.pose.position.y = -0.4;
        target_pose.pose.position.z = 0.3;
        target_pose.pose.orientation.w = 0;
        target_pose.pose.orientation.x = 0;
        target_pose.pose.orientation.y = 1;
        target_pose.pose.orientation.z = 0;


        //_my_test->both_arm_group->
        ROS_ERROR_STREAM("TEST : The end effectors of the group are: " << _my_test->group->getEndEffector());
        ROS_ERROR_STREAM("TEST : The end effectors of the secondary group are: " << _my_test->secondary_group->getEndEffector());

        _my_test->group->setPoseTarget(target_pose);
        if(_my_test->group->plan(plan))
            _my_test->group->execute(plan);

        ros::waitForShutdown();
        return 0;
    }
