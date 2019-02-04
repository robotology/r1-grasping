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
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <src/GraspDetector_IDL.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

/****************************************************************/
class GraspDetector : public GraspDetector_IDL, public RFModule
{
    BufferedPort<Bottle> skinEventPort; // default name /grasp-detector/skin_events:i, should be connected to /skinManager/skin_events:o
    RpcServer rpcPort;

    double detectionTimeOut;
    double minNbSensorsFingers;
    double minNbSensorsThumb;

    std::atomic<bool> fingersPressed[2];
    std::atomic<bool> thumbPressed[2];
    double lastFingersPressedEventTime[2];
    double lastThumbPressedEventTime[2];

    /****************************************************************/
    bool IsObjectGraspedLeft()
    {
        yDebug() << "IsObjectGraspedLeft" << (fingersPressed[0] && thumbPressed[0]);
        return fingersPressed[0] && thumbPressed[0];
    }

    /****************************************************************/
    bool IsObjectGraspedRight()
    {
        yDebug() << "IsObjectGraspedRight" << (fingersPressed[1] && thumbPressed[1]);
        return fingersPressed[1] && thumbPressed[1];
    }

    /****************************************************************/
    bool IsFingerPressedLeft()
    {
        yDebug() << "IsFingerPressedLeft" << fingersPressed[0];
        return fingersPressed[0];
    }

    /****************************************************************/
    bool IsFingerPressedRight()
    {
        yDebug() << "IsFingerPressedRight" << fingersPressed[1];
        return fingersPressed[1];
    }

    /****************************************************************/
    bool IsThumbPressedLeft()
    {
        yDebug() << "IsThumbPressedLeft" << thumbPressed[0];
        return thumbPressed[0];
    }

    /****************************************************************/
    bool IsThumbPressedRight()
    {
        yDebug() << "IsThumbPressedRight" << thumbPressed[1];
        return thumbPressed[1];
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        fingersPressed[0] = false;
        fingersPressed[1] = false;
        thumbPressed[0] = false;
        thumbPressed[1] = false;
        lastFingersPressedEventTime[0] = Time::now();
        lastFingersPressedEventTime[1] = lastFingersPressedEventTime[0];
        lastThumbPressedEventTime[0] = lastFingersPressedEventTime[0];
        lastThumbPressedEventTime[1] = lastFingersPressedEventTime[0];

        this->setName(rf.check("name", Value("grasp-detector")).asString().c_str());

        detectionTimeOut = rf.check("detectionTimeOut", Value(1.0)).asDouble();
        yInfo() << "Load module with detectionTimeOut =" << detectionTimeOut;

        minNbSensorsFingers = rf.check("minNbSensorsFingers", Value(7)).asInt();
        yInfo() << "Load module with minNbSensorsFingers =" << minNbSensorsFingers;

        minNbSensorsThumb = rf.check("minNbSensorsThumb", Value(4)).asInt();
        yInfo() << "Load module with minNbSensorsThumb =" << minNbSensorsThumb;

        skinEventPort.open("/"+this->getName()+"/skin_events:i");
        rpcPort.open("/"+this->getName()+"/rpc:i");

        this->yarp().attachAsServer(rpcPort);

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
        if(skinEventPort.getInputCount()<1)
        {
            yWarning() << "No connection to skin event input";
            Time::delay(0.1);
            return true;
        }

        double t = Time::now();

        for(int i : {0,1})
        {
            if(t - lastFingersPressedEventTime[i] > detectionTimeOut)
                fingersPressed[i] = false;

            if(t - lastThumbPressedEventTime[i] > detectionTimeOut)
                thumbPressed[i] = false;
        }

        Bottle *input = skinEventPort.read(false);

        if(!input)
        {
            Time::delay(0.01);
            return true;
        }

        if(input->size() != 1)
            return true;

        Bottle *list = input->get(0).asList();
        if(list->size() != 8)
        {
            yError() << "Invalid dimension of skin contact info returned by skinManager:" << list->toString();
            return true;
        }

        Bottle *pressedSensorInfo = list->get(0).asList();
        if(pressedSensorInfo->size() != 4)
        {
            yError() << "Invalid dimension of first argument of skin contact info returned by skinManager:" << list->toString();
            return true;
        }
        int pressedSensorPartIndex = pressedSensorInfo->get(3).asInt();

        int index = -1;
        if(pressedSensorPartIndex == 1)
            index = 0;
        else if(pressedSensorPartIndex == 4)
            index = 1;
        else
            return true;

        Bottle *pressedSensorIndices = list->get(6).asList();

        const int midSensorIndex = 96;
        int nbPressedFingers = 0;
        int nbPressedThumb = 0;
        for(int i=0 ; i<pressedSensorIndices->size() ; i++)
        {
            if(pressedSensorIndices->get(i).asInt() < midSensorIndex)
                nbPressedFingers++;
            else
                nbPressedThumb++;
        }

        if(nbPressedFingers > minNbSensorsFingers)
        {
            fingersPressed[index] = true;
            lastFingersPressedEventTime[index] = Time::now();
        }

        if(nbPressedThumb > minNbSensorsThumb)
        {
            thumbPressed[index] = true;
            lastThumbPressedEventTime[index] = Time::now();
        }
        
        yDebug() << (index==0?"left\t":"right\t") << fingersPressed[index] << '\t' << thumbPressed[index] << '\t' << (thumbPressed[index]&&fingersPressed[index])  << '\t' << nbPressedFingers << "/" << minNbSensorsFingers << '\t' << nbPressedThumb << "/" << minNbSensorsThumb << '\t' << Time::now()-lastFingersPressedEventTime[index] << '\t' << Time::now()-lastThumbPressedEventTime[index];

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
        skinEventPort.interrupt();
        rpcPort.interrupt();

        return true;
    }

    /****************************************************************/
    bool close() override
    {
        skinEventPort.close();
        rpcPort.close();

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

    GraspDetector detector;
    return detector.runModule(rf);
}
