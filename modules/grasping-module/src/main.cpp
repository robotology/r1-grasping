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
    RpcServer rpcPort;

    RpcClient pointCloudFetchPort;
    RpcClient superQuadricFetchPort;
    RpcClient graspingPoseGeneratorPort;
    RpcClient graspingPoseRefinerPort;
    RpcClient actionGatewayPort;

    /****************************************************************/
    bool getObjectPosition(const string &objectName, Vector &position) const
    {
        //connects to an object recognition database: sends the object name and retrieves object location
    }

    /****************************************************************/
    bool getObjectPointCloud(const Vector &position3D, PointCloud<DataXYZRGBA> &pointCloud) const
    {
        //connects to some vision module: sends the object position and retrieves a point cloud of the object

        pointCloud.clear();

        if(position3D.size() != 3)
        {
            yError() << "getObjectPointCloud: Invalid dimension of object position input vector";
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
    bool getFinalGraspingPose(const vector<Vector> &poseCandidates, Vector &finalGraspingPose) const
    {
        //connects to robot kinematic module: sends a set of grasping pose candidates and retrieves the best grasping pose
    }

    /****************************************************************/
    bool performGrasp(const Vector &finalGraspingPose, bool useRightHand) const
    {
        //connects to a robot kinematic module: sends a grasping pose and retrieves a boolean once the object is grasped

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

        yInfo() << "goToGraspingPose: sending command to action module:" << command.toString();

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

        Vector position3D;
        if(!this->getObjectPosition(objectName, position3D))
        {
            yError()<<"serviceGraspObject: getObjectPosition failed";
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

        PointCloud<DataXYZRGBA> pointCloud;
        if(!this->getObjectPointCloud(position3D, pointCloud))
        {
            yError()<<"serviceGraspObjectAtPosition: getObjectPointCloud failed";
            return false;
        }

        Vector superQuadricParameters;
        if(!this->getObjectSuperquadric(pointCloud, superQuadricParameters))
        {
            yError()<<"serviceGraspObjectAtPosition: getObjectSuperquadric failed";
            return false;
        }

        vector<Vector> poseCandidates;
        if(!this->getGraspingPoseCandidates(superQuadricParameters, poseCandidates))
        {
            yError()<<"serviceGraspObjectAtPosition: getGraspingPoseCandidates failed";
            return false;
        }

        Vector finalGraspingPose;
        if(!this->getFinalGraspingPose(poseCandidates, finalGraspingPose))
        {
            yError()<<"serviceGraspObjectAtPosition: getFinalGraspingPose failed";
            return false;
        }

        if(!this->performGrasp(finalGraspingPose, true))
        {
            yError()<<"serviceGraspObjectAtPosition: performGrasp failed";
            return false;
        }

        return true;
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        std::string moduleName = rf.check("name",Value("grasping-module"),"module name (string)").asString().c_str();
        this->setName(moduleName.c_str());

        this->yarp().attachAsServer(rpcPort);
        std::string rpcPortName= "/"+this->getName()+"/rpc";
        if (!rpcPort.open(rpcPortName))
        {
           yError() << this->getName() << ": Unable to open port " << rpcPortName;
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

        std::string actionGatewayPortName= "/"+this->getName()+"/actionGateway:rpc:o";
        if (!actionGatewayPort.open(actionGatewayPortName))
        {
           yError() << this->getName() << ": Unable to open port " << actionGatewayPortName;
           return false;
        }

        return true;
    }

    /****************************************************************/
    double getPeriod() override
    {
        return 0.0;
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
        pointCloudFetchPort.interrupt();
        superQuadricFetchPort.interrupt();
        actionGatewayPort.interrupt();

        return true;
    }

    /****************************************************************/
    bool close() override
    {
        rpcPort.close();
        pointCloudFetchPort.close();
        superQuadricFetchPort.close();
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
