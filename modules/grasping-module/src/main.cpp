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

#include <iostream>           // for std::cout
#include <chrono>             // for seconds
#include <thread>             //for this_thread::sleep_for
#include <cstdlib>
#include <atomic>
#include <string>

//YARP imports
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/dev/IVisualParams.h>
#include <yarp/dev/GenericVocabs.h>

#include <src/GraspingModule_IDL.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

/****************************************************************/
class GraspingModule : public RFModule, public GraspingModule_IDL
{
    string objectName;

    RpcServer rpcPort;

    RpcClient objectPositionFetchPort;
    RpcClient pointCloudFetchPort;
    RpcClient superQuadricFetchPort;
    RpcClient graspingPoseGeneratorPort;
    RpcClient graspingPoseRefinerPort;
    RpcClient graspingPoseSelectionPort;
    RpcClient actionGatewayPort;

    std::atomic<bool> halt_requested;

    /****************************************************************/
    bool getObjectPosition(const string &objectName, Vector &position)
    {
        //connects to an object recognition database: sends the object name and retrieves object location

        position.resize(3, 0.0);

        if(objectPositionFetchPort.getOutputCount()<1)
        {
            yError() << "getObjectPosition: no connection to object position reader module";
            return false;
        }

        Bottle cmd;
        cmd.fromString("ask ((name == "+objectName+"))");
yDebug() << "sending " << cmd.toString();
        Bottle reply;
        objectPositionFetchPort.write(cmd, reply);
yDebug() << "get reply " << reply.toString();
        if(reply.size() != 2)
        {
            yError() << "getObjectPosition: Retrieved invalid answer to object id query from the object position reader module: " << reply.toString();
            return false;
        }

        if(reply.get(0).asVocab() != Vocab::encode("ack"))
        {
            yError() << "getObjectPosition: object" << objectName << "does not exist in the database";
            return false;
        }

        if(!reply.check("id"))
        {
            yError() << "getObjectPosition: Retrieved invalid answer to object id query from the object position reader module: missing id:" << reply.toString();
            return false;
        }

        Bottle *objectIDlist = reply.find("id").asList();
yDebug() << "object ID list " <<objectIDlist->toString();
        if(objectIDlist->size() < 1)
        {
            yError() << "getObjectPosition: Retrieved invalid answer to object id query from the object position reader module: empty id list:" << reply.toString();
            return false;
        }

        int objectID = objectIDlist->get(0).asInt();
yDebug() << "object ID " << objectID;
        cmd.clear();
yDebug() << "sending: " << "get ((id "+objectIDlist->get(0).toString()+") (propSet (position_3d)))";
        cmd.fromString("get ((id "+objectIDlist->get(0).toString()+") (propSet (position_3d)))");
yDebug() << "sending " << cmd.toString();
        objectPositionFetchPort.write(cmd, reply);
yDebug() << "get reply " << reply.toString();
        if(reply.size() != 2)
        {
            yError() << "getObjectPosition: Retrieved invalid answer to object position query from object position reader module: " << reply.toString();
            return false;
        }

        if(reply.get(0).asVocab() != Vocab::encode("ack"))
        {
            yError() << "getObjectPosition: Retrieved invalid answer to object position query from object position reader module: object position not found: " << reply.toString();
            return false;
        }

        if(!reply.get(1).check("position_3d"))
        {
            yError() << "getObjectPosition: Retrieved invalid answer to object position query from object position reader module: object position not found: " << reply.toString();
            return false;
        }

        Bottle *position_3d = reply.get(1).find("position_3d").asList();
yDebug() << "position 3D " << position_3d->toString();

        if(position_3d->size() < 3)
        {
            yError() << "getObjectPosition: Retrieved invalid dimension of object position retrived from object position reader module: " << position_3d->toString();
            return false;
        }

        for(int i=0 ; i<3 ; i++) position[i] = position_3d->get(i).asDouble();

        return true;
    }

    /****************************************************************/
    bool getObjectPointCloud(const Vector &position3D, PointCloud<DataXYZRGBA> &pointCloud)
    {
        //connects to some vision module: sends the object position and retrieves a point cloud of the object

        pointCloud.clear();

        if(position3D.size() != 3)
        {
            yError() << "getObjectPointCloud: Invalid dimension of object position input vector";
            return false;
        }

        if(pointCloudFetchPort.getOutputCount()<1)
        {
            yError() << "getObjectPointCloud: no connection to point cloud reader module";
            return false;
        }

        Bottle cmd;
        cmd.addString("get_point_cloud_from_3D_position");
        cmd.addDouble(position3D(0));
        cmd.addDouble(position3D(1));
        cmd.addDouble(position3D(2));

        Bottle reply;
        pointCloudFetchPort.write(cmd, reply);

        if(!pointCloud.fromBottle(*(reply.get(0).asList())))
        {
            yError() << "getObjectPointCloud: Retrieved invalid point cloud: " << reply.toString();
            return false;
        }

        if(pointCloud.size()<1)
        {
            yError() << "getObjectPointCloud: Retrieved empty point cloud: " << reply.toString();
            return false;
        }

        yInfo() << "getObjectPointCloud: retrieved point cloud with" << pointCloud.size() << "points";

        return true;
    }

    /****************************************************************/
    bool getObjectSuperquadric(const PointCloud<DataXYZRGBA> &pointCloud, Vector &superQuadricParameters)
    {
        //connects to a superquadric fitting module: sends a point cloud and retrieves a superquadric

        if(superQuadricFetchPort.getOutputCount()<1)
        {
            yError() << "getObjectSuperquadric: no connection to a superquadric fitting module";
            return false;
        }

        Bottle reply;
        superQuadricFetchPort.write(pointCloud, reply);

        Vector superquadricTmp;
        reply.write(superquadricTmp);

        if (superquadricTmp.size() != 9)
        {
            yError() << "getObjectSuperquadric: Retrieved invalid superquadric: " << superquadricTmp.toString();
            return false;
        }

        superQuadricParameters.resize(10);
        superQuadricParameters[0] = superquadricTmp[0];
        superQuadricParameters[1] = superquadricTmp[1];
        superQuadricParameters[2] = superquadricTmp[2];
        superQuadricParameters[3] = 0.0;
        superQuadricParameters[4] = 0.0;
        superQuadricParameters[5] = 1.0;
        superQuadricParameters[6] = superquadricTmp[3];
        superQuadricParameters[7] = superquadricTmp[4];
        superQuadricParameters[8] = superquadricTmp[5];
        superQuadricParameters[9] = superquadricTmp[6];

        yInfo() << "getObjectSuperquadric: retrieved superquadric: " << superQuadricParameters.toString();

        return true;
    }

    /****************************************************************/
    bool getGraspingPoseCandidates(const Vector &superQuadricParameters, vector<Vector> &poseCandidates)
    {
        //connects to a grasp planner module: sends a superquadric (and additionnal constraints?) and retrieves a set of grasping pose candidates

        poseCandidates.clear();

        if(superQuadricParameters.size() != 10)
        {
            yError() << "getGraspingPoseCandidates: wrong number of superquadric parameters";
            return false;
        }

        if(graspingPoseGeneratorPort.getOutputCount()<1)
        {
            yError() << "getGraspingPoseCandidates: no connection to a grasping pose generator module";
            return false;
        }

        Bottle command;
        command.addString("get_raw_grasp_poses");
        for(int i=0 ; i<10 ; i++) command.addDouble(superQuadricParameters[i]);

        Bottle reply;
        graspingPoseGeneratorPort.write(command, reply);

        if(reply.size()<1)
        {
            yError() << "getGraspingPoseCandidates: empty reply from grasp pose generator module";
            return false;
        }

        if(reply.get(0).asVocab()==Vocab::encode("nack"))
        {
            yError() << "getGraspingPoseCandidates: invalid reply from grasp pose generator module";
            return false;
        }

        vector<Vector> poseCandidatesTmp;
        for(int i=1 ; i+6<reply.size() ; i+=7)
        {
            poseCandidatesTmp.push_back(Vector(7, 0.0));
            for(int j=0 ; j<7 ; j++) poseCandidatesTmp.back()[j] = reply.get(i+j).asDouble();
        }

        if(graspingPoseRefinerPort.getOutputCount()<1)
        {
            yError() << "getGraspingPoseCandidates: no connection to a grasping pose refining module";
            return false;
        }

        for(int i=0 ; i<poseCandidatesTmp.size() ; i++)
        {
            command.clear();
            command.addString("refine_grasp_pose");
            for(int j=0 ; j<10 ; j++) command.addDouble(superQuadricParameters[j]);
            for(int j=0 ; j<7 ; j++) command.addDouble(poseCandidatesTmp[i][j]);

            reply.clear();
            graspingPoseRefinerPort.write(command, reply);

            if(reply.size()<7) continue;

            if(reply.get(0).asVocab()!=Vocab::encode("ok")) continue;

            poseCandidates.push_back(Vector(7, 0.0));
            for(int j=0 ; j<7 ; j++) poseCandidates.back()[j] = reply.get(j).asDouble();
        }

        yInfo() <<  "getGraspingPoseCandidates: keep" << poseCandidates.size() << "/" << poseCandidatesTmp.size() << "feasible grasping pose candidates";

        return true;
    }

    /****************************************************************/
    bool getFinalGraspingPose(const Vector &superQuadricParameters, const vector<Vector> &poseCandidates, Vector &finalGraspingPose)
    {
        //connects to robot kinematic module: sends a set of grasping pose candidates and retrieves the best grasping pose

        finalGraspingPose.resize(7, 0.0);

        if(superQuadricParameters.size() != 10)
        {
            yError() << "getFinalGraspingPose: wrong number of superquadric parameters";
            return false;
        }

        if(poseCandidates.size()<1)
        {
            yError() << "getFinalGraspingPose: no pose candidate to select";
            return false;
        }

        if(graspingPoseSelectionPort.getOutputCount()<1)
        {
            yError() << "getFinalGraspingPose: no connection to a grasping pose selection module";
            return false;
        }

        Bottle command;
        command.addString("select_best_grasp_pose");
        for(int i=0 ; i<10 ; i++) command.addDouble(superQuadricParameters[i]);

        for(int i=0 ; i<poseCandidates.size() ; i++)
        {
            for(int j=0 ; j<7 ; j++) command.addDouble(poseCandidates[i][j]);
        }

        Bottle reply;
        graspingPoseSelectionPort.write(command, reply);

        if(reply.size()<1)
        {
            yError() << "getFinalGraspingPose: empty reply from grasp pose selection module";
            return false;
        }

        if(reply.get(0).asVocab()==Vocab::encode("nack") || reply.size()<8)
        {
            yError() << "getFinalGraspingPose: invalid reply from grasp pose selection module";
            return false;
        }

        if(reply.get(0).asVocab()!=Vocab::encode("ok"))
        {
            yError() << "getFinalGraspingPose: no valid grasping pose selected by grasp pose selection module";
            return false;
        }

        for(int i=0 ; i<7 ; i++) finalGraspingPose[i] = reply.get(i+1).asDouble();

        yInfo() <<  "getFinalGraspingPose: selected final pose" << finalGraspingPose.toString();

        return true;
    }

    /****************************************************************/
    bool performGrasp(const Vector &finalGraspingPose, bool useRightHand)
    {
        //connects to a robot kinematic module: sends a grasping pose and retrieves a boolean once the object is grasped

        if(finalGraspingPose.size() != 7)
        {
            yError() << "performGrasp: invalid dimension of pose vector";
            return false;
        }

        if(actionGatewayPort.getOutputCount() < 1)
        {
            yError() << "performGrasp: no connection to action gateway module";
            return false;
        }

        //  communication with actionRenderingEngine/cmd:io
        //  grasp ("cartesian" x y z gx gy gz theta) ("approach" (-0.05 0 +-0.05 0.0)) "left"/"right"

        Bottle command;

        command.addString("grasp");
        Bottle &ptr = command.addList();
        ptr.addString("cartesian");
        ptr.addDouble(finalGraspingPose(0));
        ptr.addDouble(finalGraspingPose(1));
        ptr.addDouble(finalGraspingPose(2));
        ptr.addDouble(finalGraspingPose(3));
        ptr.addDouble(finalGraspingPose(4));
        ptr.addDouble(finalGraspingPose(5));
        ptr.addDouble(finalGraspingPose(6));

        Bottle &ptr1 = command.addList();
        ptr1.addString("approach");
        Bottle &ptr2 = ptr1.addList();
        ptr2.addDouble(-0.05);
        ptr2.addDouble(0.0);
        if(useRightHand)
        {
            ptr2.addDouble(-0.05);
            command.addString("right");
        }
        else
        {
            ptr2.addDouble(0.05);
            command.addString("left");
        }
        ptr2.addDouble(0.0);

        yInfo() << "performGrasp: sending command to action module:" << command.toString();

        Bottle reply;
        actionGatewayPort.write(command, reply);

        if(reply.get(0).asVocab()==Vocab::encode("ack")) return true;
        else return false;
    }

    /****************************************************************/
    bool serviceGraspObject(const string &objectName)
    {
        // perform the full grasping of an object with a given name

        yInfo() << this->getName() << ": receive instruction to grasp object:" << objectName;

        if(halt_requested)
        {
            yInfo()<<"serviceGraspObjectAtPosition: halt requested before end of process";
            return false;
        }

        Vector position3D;
        if(!this->getObjectPosition(objectName, position3D))
        {
            yError()<<"serviceGraspObject: getObjectPosition failed";
            return false;
        }

        if(halt_requested)
        {
            yInfo()<<"serviceGraspObject: halt requested before end of process";
            return false;
        }

        if(!this->serviceGraspObjectAtPosition(position3D[0], position3D[1], position3D[2]))
        {
            yError()<<"serviceGraspObject: serviceGraspObjectAtPosition failed";
            return false;
        }

        return true;
    }

    /****************************************************************/
    bool serviceGraspObjectAtPosition(double x, double y, double z)
    {
        // perform the full grasping of an object at a given 3D position

        yInfo() << this->getName() << ": receive instruction to grasp object at" << x << y << z;

        Vector position3D(3);
        position3D[0]=x;
        position3D[1]=y;
        position3D[2]=z;

        if(halt_requested)
        {
            yInfo()<<"serviceGraspObjectAtPosition: halt requested before end of process";
            return false;
        }

        PointCloud<DataXYZRGBA> pointCloud;
        if(!this->getObjectPointCloud(position3D, pointCloud))
        {
            yError()<<"serviceGraspObjectAtPosition: getObjectPointCloud failed";
            return false;
        }

        if(halt_requested)
        {
            yInfo()<<"serviceGraspObjectAtPosition: halt requested before end of process";
            return false;
        }

        Vector superQuadricParameters;
        if(!this->getObjectSuperquadric(pointCloud, superQuadricParameters))
        {
            yError()<<"serviceGraspObjectAtPosition: getObjectSuperquadric failed";
            return false;
        }

        if(halt_requested)
        {
            yInfo()<<"serviceGraspObjectAtPosition: halt requested before end of process";
            return false;
        }

        vector<Vector> poseCandidates;
        if(!this->getGraspingPoseCandidates(superQuadricParameters, poseCandidates))
        {
            yError()<<"serviceGraspObjectAtPosition: getGraspingPoseCandidates failed";
            return false;
        }

        if(halt_requested)
        {
            yInfo()<<"serviceGraspObjectAtPosition: halt requested before end of process";
            return false;
        }

        Vector finalGraspingPose;
        if(!this->getFinalGraspingPose(superQuadricParameters, poseCandidates, finalGraspingPose))
        {
            yError()<<"serviceGraspObjectAtPosition: getFinalGraspingPose failed";
            return false;
        }

        if(halt_requested)
        {
            yInfo()<<"serviceGraspObjectAtPosition: halt requested before end of process";
            return false;
        }

        if(!this->performGrasp(finalGraspingPose, true))
        {
            yError()<<"serviceGraspObjectAtPosition: performGrasp failed";
            return false;
        }

        return true;
    }

    bool start()
    {
        halt_requested = false;
        return true;
    }

    bool halt()
    {
        halt_requested = true;
        return true;
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        std::string moduleName = rf.check("name",Value("grasping-module"),"module name (string)").asString().c_str();
        this->setName(moduleName.c_str());

        static_cast<GraspingModule_IDL*>(this)->yarp().attachAsServer(rpcPort);
        std::string rpcPortName= "/"+this->getName()+"/rpc";
        if (!rpcPort.open(rpcPortName))
        {
           yError() << this->getName() << ": Unable to open port " << rpcPortName;
           return false;
        }

        std::string objectPositionFetchPortName= "/"+this->getName()+"/objectPositionFetch:rpc:o";
        if (!objectPositionFetchPort.open(objectPositionFetchPortName))
        {
           yError() << this->getName() << ": Unable to open port " << objectPositionFetchPortName;
           return false;
        }

        std::string pointCloudFetchPortName= "/"+this->getName()+"/pointCloudFetch:rpc:o";
        if (!pointCloudFetchPort.open(pointCloudFetchPortName))
        {
           yError() << this->getName() << ": Unable to open port " << pointCloudFetchPortName;
           return false;
        }

        std::string superQuadricFetchPortName= "/"+this->getName()+"/superQuadricFetch:rpc:o";
        if (!superQuadricFetchPort.open(superQuadricFetchPortName))
        {
           yError() << this->getName() << ": Unable to open port " << superQuadricFetchPortName;
           return false;
        }

        std::string graspingPoseGeneratorPortName= "/"+this->getName()+"/graspingPoseGenerator:rpc:o";
        if (!graspingPoseGeneratorPort.open(graspingPoseGeneratorPortName))
        {
           yError() << this->getName() << ": Unable to open port " << graspingPoseGeneratorPortName;
           return false;
        }

        std::string graspingPoseRefinerPortName= "/"+this->getName()+"/graspingPoseRefiner:rpc:o";
        if (!graspingPoseRefinerPort.open(graspingPoseRefinerPortName))
        {
           yError() << this->getName() << ": Unable to open port " << graspingPoseRefinerPortName;
           return false;
        }

        std::string graspingPoseSelectionPortName= "/"+this->getName()+"/graspingPoseSelection:rpc:o";
        if (!graspingPoseSelectionPort.open(graspingPoseSelectionPortName))
        {
           yError() << this->getName() << ": Unable to open port " << graspingPoseSelectionPortName;
           return false;
        }

        std::string actionGatewayPortName= "/"+this->getName()+"/actionGateway:rpc:o";
        if (!actionGatewayPort.open(actionGatewayPortName))
        {
           yError() << this->getName() << ": Unable to open port " << actionGatewayPortName;
           return false;
        }

        objectName = rf.check("objectName",Value("Bottle"),"Grasped object name (string)").asString().c_str();

        halt_requested = false;

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

    }

    /****************************************************************/
    bool interruptModule() override
    {
        rpcPort.interrupt();
        objectPositionFetchPort.interrupt();
        pointCloudFetchPort.interrupt();
        superQuadricFetchPort.interrupt();
        graspingPoseGeneratorPort.interrupt();
        graspingPoseRefinerPort.interrupt();
        graspingPoseSelectionPort.interrupt();
        actionGatewayPort.interrupt();

        return true;
    }

    /****************************************************************/
    bool close() override
    {
        rpcPort.close();
        objectPositionFetchPort.close();
        pointCloudFetchPort.close();
        superQuadricFetchPort.close();
        graspingPoseGeneratorPort.close();
        graspingPoseRefinerPort.close();
        graspingPoseSelectionPort.close();
        actionGatewayPort.close();

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
    rf.setDefaultContext("grasping-module");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    GraspingModule module;
    return module.runModule(rf);
}
