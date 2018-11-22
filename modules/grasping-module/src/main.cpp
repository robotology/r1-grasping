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
    RpcClient actionGatewayPort;

    /****************************************************************/
    bool getObjectPosition(Vector &position) const
    {
        //connects to an object recognition database: sends the object name and retrieves object location
    }

    /****************************************************************/
    bool getObjectPointCloud(const Vector &position3D, PointCloud<DataXYZRGBA> &pointCloud) const
    {
        //connects to some vision module: sends the object position and retrieves a point cloud of the object

        if(position3D.size() == 3)
        {
            Bottle cmd;
            cmd.addString("get_point_cloud_from_3D_position");
            cmd.addDouble(position3D(0));
            cmd.addDouble(position3D(1));
            cmd.addDouble(position3D(2));

            Bottle reply;
            pointCloudFetchPort.write(cmd, reply);

            yInfo() << "getObjectPointCloud: reply size: " << reply.size();

            if(!pointCloud.fromBottle(reply))
            {
                yError() << "getObjectPointCloud: Retrieved invalid point cloud: " << reply.toString();
                return false;
            }
        }
        else
        {
            yError() << "getObjectPointCloud: Invalid dimension of object position input vector";
            return false;
        }

        return true;
    }

    /****************************************************************/
    bool getObjectSuperquadric(const PointCloud<DataXYZRGBA> &pointCloud, Vector &superQuadricParameters) const
    {
        //connects to a superquadric fitting module: sends a point cloud and retrieves a superquadric
    }

    /****************************************************************/
    bool getGraspingPoseCandidates(Vector &superQuadricParameters, vector<Matrix> &poseCandidates) const
    {
        //connects to a grasp planner module: sends a superquadric (and additionnal constraints?) and retrieves a set of grasping pose candidates
    }

    /****************************************************************/
    bool getFinalGraspingPose() const
    {
        //connects to robot kinematic module: sends a set of grasping pose candidates and retrieves the best grasping pose
    }

    /****************************************************************/
    bool goToGraspingPose() const
    {
        //connects to a robot kinematic module: sends a grasping pose and retrieves a boolean once the pose is reached
    }

    /****************************************************************/
    bool graspObject() const
    {
        //connects to robot kinematic module: sends a command to grasp an object (close the hand) and retrieves a boolean once the object is grasped
    }

    /****************************************************************/
    bool serviceGraspObject(double x, double y, double z)
    {
        yInfo() << this->getName() << ": receive instruction to grasp object at " << x << " " << y << " " << z;

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
