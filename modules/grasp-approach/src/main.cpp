/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Jason Chevrie <jason.chevrie@iit.it>
 */

#include <cstdlib>
#include <string>
#include <atomic>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <yarp/dev/IMap2D.h>
#include <yarp/dev/INavigation2D.h>

#include <src/GraspApproach_IDL.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/****************************************************************/
class GraspApproach : public GraspApproach_IDL, public RFModule
{
    RpcServer rpcPort;

    RpcClient mobileReachingPort;
    RpcClient vision3DPort;
    RpcClient OPCdetectionPort;
    BufferedPort<Bottle> detectionPort;
    RpcClient graspPort;
    RpcClient actionPort;

    PolyDriver        navDriver;
    INavigation2D*    inav;
    PolyDriver        mapDriver;
    IMap2D*           imap;

    int verbosity;

    /****************************************************************/
    bool ApproachObjectName(const string &object, const string &objectOPC, double dist)
    {
        // Get object position
        Vector objectPosition(3);
        if(!RetrieveObjectPosition(object, objectPosition))
        {
            yError() << "Cannot find object position";
            return false;
        }

        // Refine object grasp pose
        Vector objectOrientation(4);
        if(!RefineObjectGraspPose(objectPosition, objectOrientation, "local", objectOPC))
        {
            yError() << "Cannot refine object grasp pose";
            return false;
        }

        Vector objectPose = cat(objectPosition, objectOrientation);

        Map2DLocation optimalBasePose;
        if(!PlanMobileReaching(objectPose, optimalBasePose, "map"))
        {
              yError() << "Cannot plan a base pose to grasp object" << object;
              return false;
        }

        if(!NavigateNearObject(optimalBasePose, objectPosition, dist))
        {
            yError() << "Cannot navigate to approach object";
            return false;
        }

        if(!LookAtObject(objectPosition,"map"))
        {
            yError() << "Cannot look at the object";
            return false;
        }

        return true;
    }

    /****************************************************************/
    bool ApproachObjectPosition(double x, double y, double z, double dist, const string &frame)
    {
        // Get object position
        Vector objectPosition(3);
        objectPosition[0] = x;
        objectPosition[1] = y;
        objectPosition[2] = z;

        // Refine object grasp pose
        Vector objectOrientation(4);
        if(!RefineObjectGraspPose(objectPosition, objectOrientation, frame))
        {
            yError() << "Cannot refine object grasp pose";
            return false;
        }

        Vector objectPose = cat(objectPosition, objectOrientation);

        Map2DLocation optimalBasePose;
        if(!PlanMobileReaching(objectPose, optimalBasePose, "map"))
        {
              yError() << "Cannot plan a base pose to grasp object at" << objectPosition.toString();
              return false;
        }

        if(!NavigateNearObject(optimalBasePose, objectPosition, dist))
        {
            yError() << "Cannot navigate to approach object";
            return false;
        }

        if(!LookAtObject(objectPosition,"map"))
        {
            yError() << "Cannot look at the object";
            return false;
        }

        return true;
    }

    /****************************************************************/
    Bottle PlanApproachObjectName(const string &object, const string &objectOPC)
    {
        Bottle reply;
        reply.addVocab(Vocab::encode("many"));

        // Get object position
        Vector objectPosition(3);
        if(!RetrieveObjectPosition(object, objectPosition))
        {
            yError() << "Cannot find object position";
            reply.addString("Cannot find object position");
            return reply;
        }

        // Refine object grasp pose
        Vector objectOrientation(4);
        if(!RefineObjectGraspPose(objectPosition, objectOrientation, "local", objectOPC))
        {
            yError() << "Cannot refine object grasp pose";
            reply.addString("Cannot refine object grasp pose");
            return reply;
        }

        Vector objectPose = cat(objectPosition, objectOrientation);

        Map2DLocation optimalBasePose;
        if(!PlanMobileReaching(objectPose, optimalBasePose, "map"))
        {
              yError() << "Cannot plan a base pose to grasp object" << object;
              reply.addString("Cannot plan a base pose to grasp object");
              return reply;
        }

        reply.addString("Success");
        reply.addString("Grasp pose in map:");
        reply.addString(objectPose.toString());
        reply.addString("Base pose:");
        reply.addString(optimalBasePose.toString());

        return reply;
    }

    /****************************************************************/
    Bottle PlanApproachObjectPosition(double x, double y, double z, const string &frame)
    {
        Bottle reply;
        reply.addVocab(Vocab::encode("many"));

        // Get object position
        Vector objectPosition(3);
        objectPosition[0] = x;
        objectPosition[1] = y;
        objectPosition[2] = z;

        // Refine object grasp pose
        Vector objectOrientation(4);
        if(!RefineObjectGraspPose(objectPosition, objectOrientation, frame))
        {
            yError() << "Cannot refine object grasp pose";
            reply.addString("Cannot refine object grasp pose");
            return reply;
        }

        Vector objectPose = cat(objectPosition, objectOrientation);

        Map2DLocation optimalBasePose;
        if(!PlanMobileReaching(objectPose, optimalBasePose, "map"))
        {
              yError() << "Cannot plan a base pose to grasp object at" << objectPosition.toString();
              reply.addString("Cannot plan a base pose to grasp object at " + objectPosition.toString());
              return reply;
        }

        reply.addString("Success");
        reply.addString("Grasp pose in map:");
        reply.addString(objectPose.toString());
        reply.addString("Base pose in map:");
        reply.addString(optimalBasePose.toString());

        return reply;
    }


    /****************************************************************/
    bool ContinuousApproachObjectName(const string &object, const string &objectOPC, double step)
    {
        bool goalReached = false;
        while(!goalReached)
        {
            // Get object position
            Vector objectPosition(3);
            if(!RetrieveObjectPosition(object, objectPosition))
            {
                yError() << "Cannot find object position";
                return false;
            }

            // Refine object grasp pose
            Vector objectOrientation(4);
            if(!RefineObjectGraspPose(objectPosition, objectOrientation, "local", objectOPC))
            {
                yError() << "Cannot refine object grasp pose";
                return false;
            }

            Vector objectPose = cat(objectPosition, objectOrientation);

            Map2DLocation optimalBasePose;
            if(!PlanMobileReaching(objectPose, optimalBasePose, "map"))
            {
                  yError() << "Cannot plan a base pose to grasp object" << object;
                  return false;
            }

            Map2DLocation loc;
            inav->getCurrentPosition(loc);

            double dist = sqrt(pow(loc.x-objectPosition[0],2)+pow(loc.y-objectPosition[1],2)) - step;

            if(!NavigateNearObject(optimalBasePose, objectPosition, dist, &goalReached))
            {
                yError() << "Cannot navigate to approach object";
                return false;
            }

            if(!LookAtObject(objectPosition,"map"))
            {
                yError() << "Cannot look at the object";
                return false;
            }
        }

        return true;
    }

    /****************************************************************/
    bool RetrieveObjectPosition(const string &object, Vector &objectPosition)
    {
        if(RetrieveObjectPositionAFAR(object, objectPosition))
        {
            if(verbosity>1)
            {
                yDebug() << "Using afar detector to retrieve object 3D position:" << objectPosition.toString();
            }
            return true;
        }
        else if(RetrieveObjectPositionOPC(object, objectPosition))
        {
            if(verbosity>1)
            {
                yDebug() << "Using OPC detector to retrieve object 3D position:" << objectPosition.toString();
            }
            return true;
        }

        yError() << "Could not find object position";
        return false;
    }

    /****************************************************************/
    bool RetrieveObjectPositionAFAR(const string &object, Vector &objectPosition)
    {
        objectPosition.resize(3,0.0);

        // Retrieve object bounding
        Vector bbox(4,0.0);
        if(detectionPort.getInputCount() < 1 )
        {
            return false;
        }

        if (Bottle *input = detectionPort.read())
        {
            if(verbosity>1)
            {
                yDebug() << "Received from detection:" << input->size() << input->toString();
            }

            if(Bottle *content = input->get(0).asList())
            {

                if(content->size() < 7)
                {
                    yError() << "No object detected in the scene";
                    return false;
                }

                if(content->get(5).asString() == object)
                {
                    bbox[0] = content->get(0).asInt();
                    bbox[1] = content->get(1).asInt();
                    bbox[2] = content->get(2).asInt();
                    bbox[3] = content->get(3).asInt();
                }
                else
                {
                    yError() << "Object \""+object+"\" not detected in the scene";
                    return false;
                }
            }
            else
            {
                yError() << "Empty reply from detection";
                return false;
            }

            if(verbosity>1)
            {
                yDebug() << "Retrieved bounding box of object \""+object+"\":" << bbox.toString();
            }
        }
        else
        {
            yError() << "Cannot read detection port";
            return false;
        }

        // Retrieve object 3D position
        if(vision3DPort.getOutputCount() < 1)
        {
            //objectPosition[0] = 11.1;
            //objectPosition[1] = 2.65;
            //objectPosition[2] = 0.8;
            yError() << "Missing connection to vision3d-gateway module";
            return false;
        }
        else
        {
            Bottle cmd;
            cmd.addString("Points");
            cmd.addInt(bbox(0) + (abs(bbox[2]-bbox[0]) + 1) / 2);
            cmd.addInt(bbox(1) + (abs(bbox[3]-bbox[1]) + 1) / 2);

            Bottle reply;
            vision3DPort.write(cmd, reply);

            objectPosition[0] = reply.get(0).asDouble();
            objectPosition[1] = reply.get(1).asDouble();
            objectPosition[2] = reply.get(2).asDouble();
        }

        return true;
    }

    /****************************************************************/
    bool RetrieveObjectPositionOPC(const string &object, Vector &objectPosition)
    {
        objectPosition.resize(3,0.0);

        if(OPCdetectionPort.getOutputCount() < 1)
        {
            yError() << "Missing connection to detection module";
            return false;
        }

        if(verbosity>1)
        {
            yDebug() << "Using OPC to retrieve object position";
        }

        Bottle cmd;
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &content = cmd.addList();
        Bottle &cond_1=content.addList();
        cond_1.addString("entity");
        cond_1.addString("==");
        cond_1.addString("object");
        content.addString("&&");
        Bottle &cond_2=content.addList();
        cond_2.addString("name");
        cond_2.addString("==");
        cond_2.addString(object);
        Bottle reply;
        OPCdetectionPort.write(cmd,reply);

        if (reply.size()<2)
            return false;

        if (reply.get(0).asVocab() != Vocab::encode("ack"))
            return false;

        if (Bottle *idField = reply.get(1).asList())
        {
            if (Bottle *idValues = idField->get(1).asList())
            {
                //  if there are more objects under the same name, pick the first one
                int id = idValues->get(0).asInt();

                //  get the actual bounding box
                //  command message format:  [get] (("id" <num>) (propSet ("prop0" "prop1" ...)))
                cmd.clear();
                cmd.addVocab(Vocab::encode("get"));
                Bottle &content = cmd.addList();
                Bottle &list_bid = content.addList();
                list_bid.addString("id");
                list_bid.addInt(id);
                Bottle &list_propSet = content.addList();
                list_propSet.addString("propSet");
                Bottle &list_items = list_propSet.addList();
                list_items.addString("position_3d");
                reply.clear();
                OPCdetectionPort.write(cmd,reply);

                if (reply.size()<2)
                    return false;

                if (reply.get(0).asVocab() != Vocab::encode("ack"))
                    return false;

                if (Bottle *propField = reply.get(1).asList())
                {
                    if (Bottle *position_3d = propField->find("position_3d").asList())
                    {
                        if (position_3d->size()>=3)
                        {
                            objectPosition[0] = position_3d->get(0).asDouble();
                            objectPosition[1] = position_3d->get(1).asDouble();
                            objectPosition[2] = position_3d->get(2).asDouble();
                        }
                        else return false;
                    }
                    else return false;
                }
                else return false;
            }
            else return false;
        }
        else return false;

        return true;
    }

    /****************************************************************/
    bool RefineObjectGraspPose(Vector &objectPosition, Vector &objectOrientation, const string &frame, const string &OPC_name="")
    {
        // Use grasp planner with cardinal points if it is connected
        // Otherwise, if area shape is known, orientation is parallel to the edge closest to the object
        // Otherwise orientation is parallel to robot/object axis
        // After refinement position and orientation are expressed in map frame

        if(objectPosition.size() != 3)
        {
            yError() << "Invalid vector size in RefineObjectGraspPose";
            return false;
        }

        objectOrientation.resize(4,0.0);

        if(graspPort.getOutputCount()>0)
        {
            if(verbosity > 1)
            {
                yDebug() << "Using graspProcessor to find best object orientation";
            }

            if(frame=="map")
            {
                objectPosition = ConvertToLocalFrame(objectPosition);
            }
            else if(frame!="local")
            {
                yError() << "Invalid frame argument, should be \"map\" or \"local\"";
                return false;
            }

            if(verbosity>0)
            {
                yInfo() << "Asking grasp planner for grasping pose of object at" << objectPosition.toString();
            }

            Bottle cmd;
            cmd.addString("mobile_grasp_pose");
            cmd.addString(OPC_name);
            cmd.addList().read(objectPosition);
            cmd.addString("right");

            Bottle reply;
            graspPort.write(cmd, reply);

            if(reply.size() < 1)
            {
                yError() << "Empty reply from grasp planner";
                return false;
            }

            if(reply.get(0).asVocab() != Vocab::encode("ack"))
            {
                yError() << "Grasping pose could not be computed by grasp planner";
                return false;
            }

            if(Bottle *pose_b = reply.get(1).asList())
            {
                if(pose_b->size() != 7)
                {
                    yError() << "Invalid reply from grasp planner:" << reply.toString();
                    return false;
                }

                Vector pose(7);
                for(int i=0; i<7; i++)
                {
                    pose[i] = pose_b->get(i).asDouble();
                }

                pose = ConvertToMapFrame(pose);
                objectPosition[0] = pose[0];
                objectPosition[1] = pose[1];
                objectPosition[2] = pose[2];
                objectOrientation[0] = pose[3];
                objectOrientation[1] = pose[4];
                objectOrientation[2] = pose[5];
                objectOrientation[3] = pose[6];
            }
            else
            {
                yError() << "Invalid reply from grasp planner:" << reply.toString();
                return false;
            }
        }
        else
        {
            if(verbosity > 1)
            {
                yDebug() << "Missing connection to graspProcessor, using simple method to find object orientation";
            }

            if(frame=="local")
            {
                objectPosition = ConvertToMapFrame(objectPosition);
            }
            else if(frame!="map")
            {
                yError() << "Invalid frame argument, should be \"map\" or \"local\"";
                return false;
            }

            Map2DLocation loc;
            inav->getCurrentPosition(loc);

            Vector robotObjectAxis(2);
            robotObjectAxis[0] = objectPosition[0]-loc.x;
            robotObjectAxis[1] = objectPosition[1]-loc.y;
            robotObjectAxis *= 1.0/norm(robotObjectAxis);
            robotObjectAxis.push_back(0.0);

            vector<string> areaNames;
            imap->getAreasList(areaNames);

            if(verbosity>1)
            {
                yDebug() << "Area list:";
                for(int i=0 ; i<areaNames.size() ; i++)
                {
                    yDebug() << "\t" << areaNames[i];
                }
            }

            Map2DArea area;
            bool in_known_area=false;
            for(size_t i=0 ; i<areaNames.size() ; i++)
            {
                imap->getArea(areaNames[i], area);
                if(area.checkLocationInsideArea(loc))
                {
                    if(verbosity>1)
                    {
                        yDebug() << "Currently in" << areaNames[i];
                    }
                    in_known_area=true;
                    break;
                }
            }

            if(in_known_area)
            {
                if(verbosity>0)
                {
                    yInfo() << "Setting grasping orientation according to room boundaries";
                }

                Vector normal(2);
                double minDist=numeric_limits<double>::max();

                Vector objectPosition2D = objectPosition.subVector(0,1);
                Vector p0(2);
                Vector p1(2);
                p1[0]=area.points[area.points.size()-1].x;
                p1[1]=area.points[area.points.size()-1].y;

                if(verbosity>1)
                {
                    yDebug() << "Area points list:";
                }

                for(size_t i=0 ; i<area.points.size() ; i++)
                {
                    if (verbosity>1)
                        yDebug() << "\t" << area.points[i].x << area.points[i].y;

                    p0=p1;
                    p1[0]=area.points[i].x;
                    p1[1]=area.points[i].y;

                    Vector x=p0-p1;
                    x*=1.0/norm(x);
                    Vector x0=objectPosition2D-p0;
                    Vector x1=objectPosition2D-p1;

                    double dist=0;
                    if( dot(x0,x)<=0 )
                    {
                        dist=norm(x0);
                    }
                    else if( dot(x1,x)>=0 )
                    {
                        dist=norm(x1);
                    }
                    else
                    {
                        dist=fabs(x0[1]*x[0]-x0[0]*x[1]);
                    }

                    if(dist<minDist)
                    {
                        minDist=dist;
                        normal[0]=x[1];
                        normal[1]=-x[0];
                        if(minDist==0)
                            break;
                    }
                }

                normal.push_back(0);
                if(dot(normal, robotObjectAxis) < 0)
                {
                    normal = -1.0*normal;
                }

                Matrix R(3,3);

                Vector m_z(3,0.0);
                m_z[2] = -1;

                R.setCol(0, normal);
                R.setCol(1, m_z);
                R.setCol(2, cross(normal, m_z));

                objectOrientation = dcm2axis(R);
            }
            else
            {
                if(verbosity>0)
                {
                    yInfo() << "Setting grasping orientation along robot->object axis";
                }

                Matrix R(3,3);

                Vector m_z(3,0.0);
                m_z[2] = -1;

                R.setCol(0, robotObjectAxis);
                R.setCol(1, m_z);
                R.setCol(2, cross(robotObjectAxis, m_z));
            }
        }

        if(verbosity>1)
        {
            yDebug() << "Object position refined to:" << objectPosition.toString();
            yDebug() << "Object orientation fixed to:" << objectOrientation.toString();
        }

        return true;
    }

    /****************************************************************/
    bool PlanMobileReaching(const Vector &objectPose, Map2DLocation &optimalBasePose, const string &frame)
    {
        if(objectPose.size() != 7)
        {
            yError() << "Invalid vector size in PlanMobileReaching";
            return false;
        }

        if(graspPort.getOutputCount() > 0)
        {
            if(verbosity>0)
            {
                yInfo() << "Using grasp processor to plan optimal base pose";
            }

            Bottle cmd;
            cmd.addString("ask_best_base_pose");
            if(frame=="map")
            {
                cmd.addList().read(ConvertToLocalFrame(objectPose));
            }
            else if(frame!="map")
            {
                yError() << "Invalid frame argument, should be \"map\" or \"local\"";
                return false;
            }
            else
            {
                cmd.addList().read(objectPose);
            }
            cmd.addString("right");

            if(verbosity>1)
            {
                yDebug() << "Sending to grasp processor:" << cmd.toString();
            }

            Bottle reply;
            if(!graspPort.write(cmd, reply))
            {
                yError() << "Could not communicate with grasp processor";
                return false;
            }

            if(verbosity>1)
            {
                yDebug() << "Received reply from grasp processor:" << reply.toString();
            }

            if(reply.size() < 1)
            {
                yError() << "Empty reply size from grasp processor";
                return false;
            }

            if(reply.get(0).asVocab() != Vocab::encode("ack"))
            {
                yError() << "Object is unreachable";
                return false;
            }

            if(reply.size() < 2)
            {
                yError() << "Invalid reply size from grasp processor";
                return false;
            }

            if (Bottle *joints = reply.get(1).asList())
            {
                if(joints->size() < 3)
                {
                    yError() << "Invalid joint vector dimension returned by grasp processor";
                    return false;
                }

                Vector basePose(7, 0.0);
                basePose[0]=joints->get(0).asDouble();
                basePose[1]=joints->get(1).asDouble();
                basePose[5] = 1.0;
                basePose[6] = M_PI/180.0*joints->get(2).asDouble();

                basePose = ConvertToMapFrame(basePose);
                optimalBasePose.x = basePose[0];
                optimalBasePose.y = basePose[1];
                optimalBasePose.theta = 180.0/M_PI*basePose[5]*basePose[6];
                return true;
            }
            else
            {
                yError() << "Invalid reply from grasp processor:" << reply.toString();
                return false;
            }
        }
        else if(mobileReachingPort.getOutputCount() > 0)
        {
            if(verbosity>0)
            {
                yInfo() << "Using mobile reaching controller to plan optimal base pose";
            }

            Bottle cmd;
            cmd.addVocab(Vocab::encode("ask"));
            Bottle &cmd1 = cmd.addList();
            Bottle &cmdParams = cmd1.addList();
            cmdParams.addString("parameters");
            Bottle &cmdParamsList = cmdParams.addList();
            Bottle &cmdParamsMode = cmdParamsList.addList();
            cmdParamsMode.addString("mode");
            cmdParamsMode.addString("full_pose+no_torso_no_heave");
            Bottle &cmdTarget = cmd1.addList();
            cmdTarget.addString("target");
            cmdTarget.addList().addList().read(objectPose);

            if(verbosity>1)
            {
                yDebug() << "Sending to mobile reaching controller:" << cmd.toString();
            }

            Bottle reply;
            if(!mobileReachingPort.write(cmd, reply))
            {
                yError() << "Could not communicate with mobile reaching controller";
                return false;
            }

            if(verbosity>1)
            {
                yDebug() << "Received reply from mobile reaching controller:" << reply.toString();
            }

            if(reply.size() < 1)
            {
                yError() << "Empty reply from mobile reaching controller";
                return false;
            }

            if(reply.get(0).asVocab() != Vocab::encode("ack"))
            {
                yError() << "Object is unreachable";
                return false;
            }

            if (Bottle *jointsList = reply.find("q").asList())
            {
                if(jointsList->size() < 1)
                {
                    yError() << "Empty joint list returned by mobile reaching controller";
                    return false;
                }

                if (Bottle *joints = jointsList->get(0).asList())
                {
                    if(joints->size() < 3)
                    {
                        yError() << "Invalid joint vector dimension returned by mobile reaching controller";
                        return false;
                    }
                    optimalBasePose.x = joints->get(0).asDouble();
                    optimalBasePose.y = joints->get(1).asDouble();
                    optimalBasePose.theta = joints->get(2).asDouble();
                    return true;
                }
                else
                {
                    yError() << "Invalid reply from mobile reaching controller:" << reply.toString();
                    return false;
                }
            }
            else
            {
                yError() << "Invalid reply from mobile reaching controller:" << reply.toString();
                return false;
            }
        }
        else
        {
            yError() << "Missing connection to plan a mobile grasp";
            return false;
        }

        return true;
    }

    /****************************************************************/
    bool NavigateNearObject(Map2DLocation baseGoal, const Vector &objectPosition, double dist, bool *goalReached=nullptr)
    {
        if(verbosity > 1)
        {
            yDebug() << "NavigateNearObject received base position" << baseGoal.toString() << " and object position" << objectPosition.toString();
        }

        if(objectPosition.size() < 2)
        {
            yError() << "Invalid vector size in NavigateNearObject";
            return false;
        }

        if(!SendNavigationCommand(baseGoal))
        {
            return false;
        }

        Vector goalVec = objectPosition.subVector(0,1);

        Map2DLocation loc;
        inav->getCurrentPosition(loc);
        Vector locVec(2);
        locVec[0] = loc.x;
        locVec[1] = loc.y;

        NavigationStatusEnum navStatus;
        inav->getNavigationStatus(navStatus);

        while( (norm(goalVec-locVec) > dist) &&
               navStatus!=navigation_status_goal_reached &&
               navStatus!=navigation_status_idle &&
               navStatus!=navigation_status_aborted &&
               navStatus!=navigation_status_failing &&
               navStatus!=navigation_status_error )
        {
            inav->getNavigationStatus(navStatus);
            inav->getCurrentPosition(loc);
            locVec[0] = loc.x;
            locVec[1] = loc.y;

            if(verbosity > 1)
            {
                yDebug() << "Desired stop distance=" << dist << "\tStatus" << (int)navStatus << "\t Current distance=" << norm(goalVec-locVec);
            }

            yarp::os::Time::delay(0.1);
        }

        if(verbosity > 1)
        {
            yDebug() << "Stopped at location" << loc.toString();
        }

        inav->stopNavigation();

        if(navStatus==navigation_status_aborted ||
           navStatus==navigation_status_failing ||
           navStatus==navigation_status_error )
        {
            if(verbosity > 0)
            {
                yInfo("Navigation failed during motion");
            }
            return false;
        }


        if(norm(goalVec-locVec) > dist)
        {
            if(verbosity > 0)
            {
                yInfo("Optimal base pose reached before reaching desired object/robot distance");
            }
            if(goalReached)
            {
                *goalReached = true;
            }
        }
        else
        {
            if(verbosity > 0)
            {
                yInfo("Desired object/robot distance reached");
            }
            if(goalReached)
            {
                *goalReached = false;
            }
        }

        return true;
    }

    /****************************************************************/
    bool LookAtObject(Vector objectPosition, const string &frame)
    {
        if(actionPort.getOutputCount() < 1)
        {
            yError() << "Missing connection to action gateway";
            return false;
        }

        if(frame=="map")
        {
            objectPosition = ConvertToLocalFrame(objectPosition);
        }
        else if(frame!="local")
        {
            yError() << "Invalid frame argument, should be \"map\" or \"local\"";
            return false;
        }

        Bottle cmd;
        cmd.addVocab(Vocab::encode("look"));
        Bottle &targetProp=cmd.addList();
        targetProp.addString("cartesian");
        targetProp.addDouble(objectPosition[0]);
        targetProp.addDouble(objectPosition[1]);
        targetProp.addDouble(objectPosition[2]);
        cmd.addString("wait");

        if(verbosity > 0)
        {
            yInfo() << "Sending to action-gateway:" << cmd.toString();
        }

        Bottle reply;
        actionPort.write(cmd, reply);

        if(reply.size() < 1)
        {
            yError() << "Empty reply from action gateway";
            return false;
        }

        if(reply.get(0).asVocab() != Vocab::encode("ack"))
        {
            yError() << "Cannot look at the object";
            return false;
        }

        Time::delay(15);

        return true;
    }

    /****************************************************************/
    bool SendNavigationCommand(Map2DLocation baseGoal)
    {
        NavigationStatusEnum navStatus;
        inav->getNavigationStatus(navStatus);
        if(navStatus!=navigation_status_idle)
        {
            inav->stopNavigation();
        }

        Map2DLocation loc;
        inav->getCurrentPosition(loc);
        baseGoal.map_id = loc.map_id;

        if(!inav->gotoTargetByAbsoluteLocation(baseGoal))
        {
            yError() << "Error from navigation server";
            return false;
        }

        return true;
    }

    /****************************************************************/
    Vector ConvertToMapFrame(const Vector &xL)
    {
        Map2DLocation loc;
        inav->getCurrentPosition(loc);


        Vector O(4,0.0);
        O[2] = 1.0;
        O[3] = M_PI/180.0*loc.theta;
        Matrix Hd=axis2dcm(O);
        Vector T(3,0.0);
        T[0] = loc.x+0.044*cos(O[3]);
        T[1] = loc.y+0.044*sin(O[3]);

        Vector xM;

        if(xL.size() == 3)
        {
            xM = T + Hd.submatrix(0,2, 0,2)*xL;
        }
        else if(xL.size() == 4)
        {
            xM = dcm2axis(Hd*axis2dcm(xL));
        }
        else if(xL.size() == 7)
        {
            xM.resize(7);
            xM.setSubvector(0, T + Hd.submatrix(0,2, 0,2)*xL.subVector(0,2));
            xM.setSubvector(3, dcm2axis(Hd*axis2dcm(xL.subVector(3,6))));
        }
        else
        {
            yError() << "Invalid input dimension for map frame conversion";
        }

        return xM;
    }

    /****************************************************************/
    Vector ConvertToLocalFrame(const Vector &xM)
    {
        Map2DLocation loc;
        inav->getCurrentPosition(loc);

        Vector O(4,0.0);
        O[2] = 1.0;
        O[3] = M_PI/180.0*loc.theta;
        Matrix Hd = axis2dcm(O);
        Hd[0][3] = loc.x+0.044*cos(O[3]);
        Hd[1][3] = loc.y+0.044*sin(O[3]);

        Hd = SE3inv(Hd);

        Vector T = Hd.subcol(0,3, 3);

        Vector xL;

        if(xM.size() == 3)
        {
            xL = T + Hd.submatrix(0,2, 0,2)*xM;
        }
        else if(xM.size() == 4)
        {
            xL = dcm2axis(Hd*axis2dcm(xM));
        }
        else if(xM.size() == 7)
        {
            xL.resize(7);
            xL.setSubvector(0, T + Hd.submatrix(0,2, 0,2)*xM.subVector(0,2));
            xL.setSubvector(3, dcm2axis(Hd*axis2dcm(xM.subVector(3,6))));
        }
        else
        {
            yError() << "Invalid input dimension for local frame conversion";
        }

        return xL;
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        this->setName(rf.check("name", Value("grasp-approach")).asString().c_str());
        verbosity=rf.check("verbosity",Value(0)).asInt();
        string map_server=rf.check("map-server",Value("/mapServer")).asString();
        string loc_server=rf.check("loc-server",Value("/localizationServer")).asString();
        string nav_server=rf.check("nav-server",Value("/navigationServer")).asString();

        mobileReachingPort.open("/"+this->getName()+"/mobile-reaching/rpc:o");
        vision3DPort.open("/"+this->getName()+"/vision3d-gateway/rpc:o");
        OPCdetectionPort.open("/"+this->getName()+"/OPCdetection/rpc:o");
        detectionPort.open("/"+this->getName()+"/detection:i");
        graspPort.open("/"+this->getName()+"/grasp-processor/rpc:o");
        actionPort.open("/"+this->getName()+"/action-gateway/rpc:o");

        // Map
        Property option;
        option.put("device","map2DClient");
        option.put("remote",map_server);
        option.put("local","/grasp-approach/map");
        if (!mapDriver.open(option))
        {
            yError("Unable to connect to %s",(map_server+"/rpc").c_str());
            close();
            return false;
        }
        if (!mapDriver.view(imap))
        {
            yError() << "Unable to open IMap2D interface";
            return false;
        }

        // Navigation
        option.clear();
        option.put("device","navigation2DClient");
        option.put("local","/grasp-approach/navigation");
        option.put("navigation_server",nav_server);
        option.put("map_locations_server",map_server);
        option.put("localization_server",loc_server);
        if (!navDriver.open(option))
        {
            yError("Unable to connect to %s",(nav_server+"/rpc").c_str());
            close();
            return false;
        }
        if (!navDriver.view(inav))
        {
            yError() << "Unable to open INavigation2D interface";
            return false;
        }
        inav->stopNavigation();
        yarp::os::Time::delay(0.1);

        rpcPort.open("/"+this->getName()+"/rpc:i");
        this->yarp().attachAsServer(rpcPort);

        return true;
    }

    /****************************************************************/
    double getPeriod() override
    {
        return 1.0;
    }

    /****************************************************************/
    bool updateModule() override
    {
        return true;
    }

    /****************************************************************/
    bool respond(const Bottle &command, Bottle &reply) override
    {
        reply.clear();

        return true;
    }

    /****************************************************************/
    bool interruptModule() override
    {
        rpcPort.interrupt();
        mobileReachingPort.interrupt();
        vision3DPort.interrupt();
        OPCdetectionPort.interrupt();
        detectionPort.interrupt();
        graspPort.interrupt();
        actionPort.interrupt();

        return true;
    }

    /****************************************************************/
    bool close() override
    {
        rpcPort.close();
        mobileReachingPort.close();
        vision3DPort.close();
        OPCdetectionPort.close();
        detectionPort.close();
        graspPort.close();
        actionPort.close();

        return true;
    }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setDefaultContext("grasp-detector");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    GraspApproach module;
    return module.runModule(rf);
}
