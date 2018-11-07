#include "carve_nodes.h"
#include<yarp/os/Network.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/os/LogStream.h> // for yError()

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
    Carve::RegisterNodes(factory);
}

using namespace yarp::os;
using namespace yarp::os;
using namespace yarp::dev;

NodeStatus Carve::IsInvPoseComputed(TreeNode& self)
{
    SleepMS(50);

//    std::cout << "[IsInvPoseComputed] Current pose: " << self.blackboard()->get<std::string>("InvPose") << std::endl;



//    Bottle cmd;
//    Bottle response;

//    cmd.addString("getInvPose");
//    Carve::inv_pose_port_out.write(cmd,response);

    std::string inv_pose = self.blackboard()->get<std::string>("InvPose");


    std::cout << "[IsInvPoseComputed] Current pose: " << inv_pose << std::endl;

//    if(inv_pose.empty())
//    {    std::cout << "[IsInvPoseComputed]  inv pose empty " << inv_pose << std::endl;
//        return NodeStatus::FAILURE;
//    }
    return !inv_pose.empty() ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}


NodeStatus Carve::IsInvPoseValid(TreeNode& self)
{
    SleepMS(50);



    return NodeStatus::SUCCESS;
}

NodeStatus Carve::IsRobotAtInvPose(TreeNode& self)
{
    SleepMS(50);
    bool is_at_inv_pose = self.blackboard()->get<bool>("RAtInvPose");

    std::cout << "[IsRobotAtInvPose]  is_at_inv_pose: " << is_at_inv_pose << std::endl;



    return is_at_inv_pose ?  NodeStatus::SUCCESS : NodeStatus::FAILURE;
}


NodeStatus Carve::LocateObj(TreeNode& self)
{

    std::cout << "[LocateObj] : Locating Object" << std::endl;


    SleepMS(2000);
    self.blackboard()->set("IsObjLocated", true);
    return NodeStatus::SUCCESS;
}

NodeStatus Carve::IsObjLocated(TreeNode& self)
{
    SleepMS(50);


    bool is_obj_located = self.blackboard()->get<bool>("IsObjLocated");
    std::cout << "[IsObjLocated] : " <<  is_obj_located << std::endl;


    return is_obj_located ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}


NodeStatus Carve::ComputeInvPose(TreeNode& self)
{
    SleepMS(200);

    std::cout << "[ComputeInvPose] computing inv pose: " <<  self.blackboard()->get<std::string>("InvPose") << std::endl;

    std::string inv_pose_computed = "-1;3;0.5";
//    Bottle cmd;
//    Bottle response;

//    cmd.addString("setInvPose");
//    cmd.addString(inv_pose_computed);
//    Carve::inv_pose_port_out.write(cmd,response);

    std::cout << "[ComputeInvPose] new pose: " << inv_pose_computed << std::endl;
    self.blackboard()->set("InvPose", inv_pose_computed);
    return NodeStatus::SUCCESS;
}



NodeStatus Carve::SetInvPoseInvalid(TreeNode& self)
{
    SleepMS(200);

    std::cout << "[SetInvPoseInvalid] computing inv pose: "  << std::endl;

    std::string inv_pose_invalid = "-1";
    Bottle cmd;
    Bottle response;

    cmd.addString("setInvPose");
    cmd.addString(inv_pose_invalid);
    Carve::inv_pose_port_out.write(cmd,response);

    return NodeStatus::SUCCESS;
}



NodeStatus Carve::Goto::tick()
{


    Bottle cmd;
    Bottle response;
    int N = 10;

    cmd.addString("getInvPose");
    Carve::inv_pose_port_out.write(cmd,response);
    std::string goal = response.get(0).asString();

    for (int i = 0; i< N; i++)
    {
        SleepMS(500);
        if (_halt_requested)
        {
         return BT::NodeStatus::IDLE;
        }

        std::cout << "[Goto] going to target " << i << "of 10"  << std::endl;


    }


        blackboard()->set("RAtInvPose", true);
//    cmd.clear();
//    response.clear();

//    cmd.addString("getInvPose");
//    Carve::inv_pose_port_out.write(cmd,response);
    return _halt_requested ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
}

void Carve::Goto::halt()
{
    std::cout << "[Goto] halt" << std::endl;

    _halt_requested.store(true);
}



NodeStatus Carve::Grasp::tick()
{

    std::cout << "[Grasp] Grasping Object" << std::endl;

    Bottle cmd;
    Bottle response;

    cmd.addString("getObjPose");
    Carve::inv_pose_port_out.write(cmd,response);
    std::string grasp_goal = response.get(0).asString();
    return _halt_requested ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
}

void Carve::Grasp::halt()
{
    std::cout << "[Grasp] halt" << std::endl;

    _halt_requested.store(true);
}



void Carve::RegisterNodes(BehaviorTreeFactory& factory)
{
    factory.registerSimpleCondition("IsInvPoseComputed", IsInvPoseComputed);
    factory.registerSimpleCondition("IsInvPoseValid", IsInvPoseValid);
    factory.registerSimpleAction("ComputeInvPose", ComputeInvPose);

    factory.registerSimpleCondition("IsRobotAtInvPose", IsRobotAtInvPose);
    factory.registerSimpleCondition("LocateObj", LocateObj);
    factory.registerSimpleAction("IsObjLocated", IsObjLocated);
    factory.registerSimpleAction("SetInvPoseInvalid", SetInvPoseInvalid);




    factory.registerNodeType<Goto>("Goto");
    factory.registerNodeType<Grasp>("Grasp");




    Carve::inv_pose_port_out.open("/inv_pose_request:o");
    //Carve::goto_port_out.open("goto_request:o");

    yarp.connect("/inv_pose_request:o", "/inv_pose_request:i");
    //yarp.connect("goto_request:i", "goto_request:o");

    Property        navTestCfg;
    INavigation2D* iNav = 0;
    Map2DLocation pos;
    Map2DLocation curr_goal;
    PolyDriver ddNavClient;

    //Gets the current navigation status
    NavigationStatusEnum status;
    //opens a navigation2DClient device to control the robot
    navTestCfg.put("device", "navigation2DClient");
    navTestCfg.put("local", "/robotGotoExample");
    navTestCfg.put("navigation_server", "/robotGoto");
    navTestCfg.put("map_locations_server", "/mapServer");
    navTestCfg.put("localization_server", "/localizationServer");
    bool ok = ddNavClient.open(navTestCfg);
    if (!ok)
    {
        yError() << "Unable to open navigation2DClient device driver";
//            return -1;
    }

    //gets a INavigation2D interface from the navigation2DClient
//        INavigation2D* iNav = 0;
    ok = ddNavClient.view(iNav);
    if (!ok)
    {
        yError() << "Unable to open INavigation2D interface";
//            return -1;
    }


}
