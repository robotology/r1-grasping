
struct Bottle{}
(
    yarp.name = "yarp::os::Bottle"
    yarp.includefile="yarp/os/Bottle.h"
)

service GraspApproach_IDL {
  bool ApproachObjectName(1:string object, 2:string objectOPC, 3:double distance);
  bool ApproachObjectPosition(1:double x, 2:double y, 3:double z, 4:double dist, 5:string frame);
  Bottle PlanApproachObjectName(1:string object, 2:string objectOPC);
  Bottle PlanApproachObjectPosition(1:double x, 2:double y, 3:double z, 4:string frame);
  bool ContinuousApproachObjectName(1:string object, 2:string objectOPC, 3:double step)
}
