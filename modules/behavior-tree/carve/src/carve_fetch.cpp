#include "behavior_tree_core/xml_parsing.h"
#include "behavior_tree_logger/bt_cout_logger.h"
#include "Blackboard/blackboard_local.h"

#include "carve_nodes.h"


using namespace BT;

// clang-format off
const std::string xml_text = R"(
<root>
    <BehaviorTree ID="MainTree">
        <Fallback name="root">
            <Sequence name="fetch">
                <Fallback name="invpose">
                <IsInvPoseComputed/>
                <ComputeInvPose/>
                </Fallback>
                <Fallback name="goto">
                <IsRobotAtInvPose/>
                <Goto goal="InvPose" />
                </Fallback>
                <Fallback name="locate">
                <IsObjLocated/>
                <LocateObj/>
                </Fallback>
                 <Grasp obj_pose="${ObjPose}" />
            </Sequence>
            <SetInvPoseInvalid/>
        </Fallback>
    </BehaviorTree>
</root>
 )";
//<root main_tree_to_execute = "MainTree" >
//    <BehaviorTree ID="MainTree">
//           <Sequence name="root">
//           <SetBlackboard key="InvPose" value="" />
//               <Fallback name="invpose">
//                   <IsInvPoseComputed/>
//                   <ComputeInvPose/>
//               </Fallback>
//           <Goto goal="${InvPose}" />
//           </Sequence>
//       </BehaviorTree>
//</root>
// clang-format on

// Write into the blackboard key: [GoalPose]
// Use this function to create a SimpleActionNode that can access the blackboard


int main()
{
    using namespace BT;

    BehaviorTreeFactory factory;
//    factory.registerSimpleCondition("IsInvPoseComputed",Carve::IsInvPoseComputed);
//    factory.registerSimpleAction("ComputeInvPose", Carve::ComputeInvPose);
//    factory.registerNodeType<Carve::Goto>("Goto");



    // create a Blackboard from BlackboardLocal (simple, not persistent, local storage)    
    auto blackboard = Blackboard::create<BlackboardLocal>();
    std::string inv_pose = "";

    blackboard->set("RAtInvPose", false);
    blackboard->set("InvPose", inv_pose);
    blackboard->set("InvPoseComputed", false);
    blackboard->set("IsObjGrasped", false);
    blackboard->set("IsObjLocated", false);


    Carve::RegisterNodes(factory);


    // Important: when the object tree goes out of scope, all the TreeNodes are destroyed
//    auto tree = buildTreeFromText(factory, xml_text, blackboard);

      auto tree = buildTreeFromText(factory, xml_text);
      tree.root_node->setBlackboard(blackboard);
      assignBlackboardToEntireTree( tree.root_node, blackboard );

    NodeStatus status = NodeStatus::RUNNING;
    while (true)
    {
       // std::cout << "Ticking the root node" << std::endl;
        status = tree.root_node->executeTick();
       // std::cout << "root returned "<< status << std::endl;


        SleepMS(1000);   // optional sleep to avoid "busy loops"
    }
    return 0;
}
