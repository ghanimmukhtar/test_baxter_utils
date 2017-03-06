#include <baxter_mover_utils/baxter_mover.hpp>

using namespace baxter_mover;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_baxter_utils");
    ros::NodeHandle node;
    BAXTER_Mover::Ptr _my_test;
    _my_test.reset(new BAXTER_Mover(node));

    //ros::AsyncSpinner my_spinner(1);
    //my_spinner.start();

    ros::waitForShutdown();
    return 0;
}
