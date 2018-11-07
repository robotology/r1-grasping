#include "behavior_tree_core/bt_factory.h"
#include<yarp/os/Network.h>
#include<yarp/os/RpcClient.h>

using namespace BT;




inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Custom type
struct Pose2D
{
    double x, y, theta;
};


namespace Carve
{
yarp::os::Network yarp;
yarp::os::RpcClient inv_pose_port_out, goto_port_out;

BT::NodeStatus IsInvPoseComputed(TreeNode& self);

BT::NodeStatus IsInvPoseValid(TreeNode& self);

BT::NodeStatus ComputeInvPose(TreeNode& self);

BT::NodeStatus IsRobotAtInvPose(TreeNode& self);

BT::NodeStatus IsObjLocated(TreeNode& self);

BT::NodeStatus LocateObj(TreeNode& self);

BT::NodeStatus SetInvPoseInvalid(TreeNode& self);

class Goto : public BT::ActionNode
{
  public:
    // If your TreeNode requires a NodeParameter, you must define a constructor
    // with this signature.
    Goto(const std::string& name, const BT::NodeParameters& params)
      : ActionNode(name, params)
    {
    }

    // It is mandatory to define this static method.
    // If you don't, BehaviorTreeFactory::registerNodeType will not compile.
    //
    static const BT::NodeParameters& requiredNodeParameters()
    {
        static BT::NodeParameters params = {{"goal", "0;0;0"}};
        return params;
    }

    BT::NodeStatus tick() override;

    virtual void halt() override;

  private:
    std::atomic_bool _halt_requested;
};

class Grasp : public BT::ActionNode
{
  public:
    // If your TreeNode requires a NodeParameter, you must define a constructor
    // with this signature.
    Grasp(const std::string& name, const BT::NodeParameters& params)
      : ActionNode(name, params)
    {
    }

    // It is mandatory to define this static method.
    // If you don't, BehaviorTreeFactory::registerNodeType will not compile.
    //
    static const BT::NodeParameters& requiredNodeParameters()
    {
        static BT::NodeParameters params = {{"obj_pose", "0;0;0"}};
        return params;
    }

    BT::NodeStatus tick() override;

    virtual void halt() override;

  private:
    std::atomic_bool _halt_requested;
};


void RegisterNodes(BT::BehaviorTreeFactory& factory);
}
