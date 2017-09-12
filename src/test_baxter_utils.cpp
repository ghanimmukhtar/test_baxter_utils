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

    moveit::planning_interface::MoveGroup::Plan first_plan, second_plan;

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


    double x = 0.65;
    double y = -0.4;
    double z = 0.2;


    //_my_test->both_arm_group->
    ROS_ERROR_STREAM("TEST : The end effectors of the group are: " << _my_test->group->getEndEffector());
    ROS_ERROR_STREAM("TEST : The end effectors of the secondary group are: " << _my_test->secondary_group->getEndEffector());

    std::vector<geometry_msgs::Pose> goal;
    goal.push_back(target_pose.pose);

    std::vector<geometry_msgs::Pose> waypoints;


    bool finish = false;
    int arm_choice = 0;
    std::string gripper;
    while(!finish && ros::ok()){
//        if(arm_choice % 2 == 0)
//            gripper = "left_gripper";
//        else
//            gripper = "right_gripper";

//        _my_test->both_arm_group->setEndEffectorLink(gripper);
        ROS_ERROR_STREAM("TEST : The end effectors of the combined group are: " << _my_test->both_arm_group->getEndEffectorLink());

        robot_state::RobotState start_state_second_trajectory = *_my_test->group->getCurrentState();
        //ROS_WARN_STREAM("TEST : The robot state is: ");
        //_my_test->both_arm_group->getCurrentState()->printStateInfo();
        //if(arm_choice % 2 == 0)
        //_my_test->both_arm_group->setPoseTarget(target_pose, "left_hand_eef");
        _my_test->both_arm_group->setPositionTarget(x, y, z, "l_gripper_l_finger_tip");
        // else
        _my_test->both_arm_group->setPositionTarget(x, y, z, "r_gripper_l_finger_tip");
        if(_my_test->both_arm_group->plan(first_plan)){
            ROS_WARN("TEST : Trying to plan the straight line part");
//            moveit::core::jointTrajPointToRobotState(first_plan.trajectory_.joint_trajectory,
//                                                     first_plan.trajectory_.joint_trajectory.points.size() - 1,
//                                                     start_state_second_trajectory);

//            waypoints.push_back(target_pose.pose);
//            target_pose.pose.position.z = 0.1;
//            waypoints.push_back(target_pose.pose);
//            moveit_msgs::RobotTrajectory robot_trajectory;
//            _my_test->both_arm_group->setStartState(start_state_second_trajectory);
//            double fraction = _my_test->both_arm_group->computeCartesianPath(waypoints, 0.01, 0.0, robot_trajectory);
//            ROS_WARN_STREAM("CONTROLLER : fraction solved of desired path in this trial is: " <<
//                            fraction);

//            if(fraction == 1){
//                second_plan.trajectory_ = robot_trajectory;
//                _my_test->both_arm_group->execute(first_plan);
//                _my_test->both_arm_group->execute(second_plan);

//            }
        }
        else
            ROS_WARN("TEST : Didn't even try to plan the straight line part !!!");
        arm_choice++;
        ROS_INFO_STREAM("*******************************: " << arm_choice << " :********************************");
    }
    //ros::waitForShutdown();
    return 0;
}
