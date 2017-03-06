#include <baxter_mover_utils/baxter_mover.hpp>

using namespace baxter_mover;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "service_user");
    ros::NodeHandle node;

    ros::ServiceClient mover = node.serviceClient<baxter_mover_utils::move_baxter_arm>("move_baxter_arm", 1);

    //ros::AsyncSpinner my_spinner(1);
    //my_spinner.start();

    baxter_mover_utils::move_baxter_arm::Request request;
    baxter_mover_utils::move_baxter_arm::Response response;
    request.arm = "right";
    request.type = "position";
    request.goal = {0.0, -0.9, 0.1};

    mover.call(request, response);



    return 0;
}
