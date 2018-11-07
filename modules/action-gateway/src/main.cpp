/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/****************************************************************/
class Gateway : public RFModule
{
    RpcServer cmdPort;
    RpcClient opcPort;
    RpcClient gazePort;

    string robot;
    double period;
    int ack,nack;
    bool interrupting;

    struct {
        vector<double> head;
        vector<double> torso;
        vector<double> left_arm;
        vector<double> left_hand;
        vector<double> right_arm;
        vector<double> right_hand;
        double speed_angular;
        double speed_linear;
    } home;

    vector<PolyDriver> drivers;
    IControlMode     *imod_head,*imod_torso,*imod_left_arm,*imod_left_hand,
                     *imod_right_arm,*imod_right_hand;
    IPositionControl *ipos_head,*ipos_torso,*ipos_left_arm,*ipos_left_hand,
                     *ipos_right_arm,*ipos_right_hand;

    /****************************************************************/
    bool getHomeInfo(const Bottle &opt, const string &part, vector<double> &v,
                     IPositionControl *ipos)
    {
        v.clear();
        if (Bottle *b=opt.find(part).asList())
        {
            int nAxes;
            ipos->getAxes(&nAxes);
            v.assign((size_t)nAxes,0.0);

            size_t len=std::min(v.size(),b->size());
            for (size_t i=0; i<len; i++)
            {
                v.push_back(b->get(i).asDouble());
            }
        }
        return !v.empty();
    }

    /****************************************************************/
    bool openDriver(PolyDriver &driver, const string &part,
                    IControlMode *&imod, IPositionControl *&ipos)
    {
        Property options;
        options.put("device","remote_controlboard");
        options.put("remote","/"+robot+"/"+part);
        options.put("local","/action-gateway/"+part);
        if (driver.open(options))
        {
            return (driver.view(imod) && driver.view(ipos));
        }
        else
            return false;
    }

    /****************************************************************/
    bool checkMotionDonePart(IPositionControl *ipos)
    {
        int nAxes;
        ipos->getAxes(&nAxes);
        unique_ptr<bool> done=unique_ptr<bool>(new bool[nAxes]);
        ipos->checkMotionDone(done.get());
        for (int i=0; i<nAxes; i++)
        {
            if (!done.get()[i])
            {
                return false;
            }
        }
        return true;
    }

    /****************************************************************/
    bool goHome(const string &part)
    {
        if (!part.empty() && (part!="all") && (part!="head") && (part!="gaze") &&
            (part!="torso") && (part!="left") && (part!="right"))
        {
            yError()<<"Unrecognized part requested for homing";
            return false;
        }

        yInfo()<<"Start homing"<<part;
        stopCartesian();
        if (!home.head.empty() && (part.empty() || (part=="all") || (part=="head") || (part=="gaze")))
        {
            vector<int> modes(home.head.size(),VOCAB_CM_POSITION);
            vector<double> accs(home.head.size(),numeric_limits<double>::max());
            vector<double> spds(home.head.size(),home.speed_angular);

            imod_head->setControlModes(modes.data());
            ipos_head->setRefAccelerations(accs.data());
            ipos_head->setRefSpeeds(spds.data());
            ipos_head->positionMove(home.head.data());
        }
        if (!home.torso.empty() && (part.empty() || (part=="all") || (part=="torso") || (part=="left") || (part=="right")))
        {
            vector<int> modes(home.torso.size(),VOCAB_CM_POSITION);
            vector<double> accs(home.torso.size(),numeric_limits<double>::max());
            vector<double> spds(home.torso.size(),home.speed_angular);
            spds[0]=home.speed_linear;  // heave

            imod_torso->setControlModes(modes.data());
            ipos_torso->setRefAccelerations(accs.data());
            ipos_torso->setRefSpeeds(spds.data());
            ipos_torso->positionMove(home.head.data());
        }
        if (!home.left_arm.empty() && (part.empty() || (part=="all") || (part=="left")))
        {
            vector<int> modes(home.left_arm.size(),VOCAB_CM_POSITION);
            vector<double> accs(home.left_arm.size(),numeric_limits<double>::max());
            vector<double> spds(home.left_arm.size(),home.speed_angular);
            spds[5]=home.speed_linear;  // heave

            imod_left_arm->setControlModes(modes.data());
            ipos_left_arm->setRefAccelerations(accs.data());
            ipos_left_arm->setRefSpeeds(spds.data());
            ipos_left_arm->positionMove(home.head.data());
        }
        if (!home.left_hand.empty() && (part.empty() || (part=="all") || (part=="left")))
        {
            vector<int> modes(home.left_hand.size(),VOCAB_CM_POSITION);
            vector<double> accs(home.left_hand.size(),numeric_limits<double>::max());
            vector<double> spds(home.left_hand.size(),home.speed_angular);

            imod_left_hand->setControlModes(modes.data());
            ipos_left_hand->setRefAccelerations(accs.data());
            ipos_left_hand->setRefSpeeds(spds.data());
            ipos_left_hand->positionMove(home.head.data());
        }
        if (!home.right_arm.empty() && (part.empty() || (part=="all") || (part=="right")))
        {
            vector<int> modes(home.right_arm.size(),VOCAB_CM_POSITION);
            vector<double> accs(home.right_arm.size(),numeric_limits<double>::max());
            vector<double> spds(home.right_arm.size(),home.speed_angular);
            spds[5]=home.speed_linear;  // heave

            imod_right_arm->setControlModes(modes.data());
            ipos_right_arm->setRefAccelerations(accs.data());
            ipos_right_arm->setRefSpeeds(spds.data());
            ipos_right_arm->positionMove(home.head.data());
        }
        if (!home.right_hand.empty() && (part.empty() || (part=="all") || (part=="right")))
        {
            vector<int> modes(home.right_hand.size(),VOCAB_CM_POSITION);
            vector<double> accs(home.right_hand.size(),numeric_limits<double>::max());
            vector<double> spds(home.right_hand.size(),home.speed_angular);

            imod_right_hand->setControlModes(modes.data());
            ipos_right_hand->setRefAccelerations(accs.data());
            ipos_right_hand->setRefSpeeds(spds.data());
            ipos_right_hand->positionMove(home.head.data());
        }

        while (true)
        {
            if (checkMotionDonePart(ipos_head) && checkMotionDonePart(ipos_torso) &&
                checkMotionDonePart(ipos_left_arm) && checkMotionDonePart(ipos_left_hand) &&
                checkMotionDonePart(ipos_right_arm) && checkMotionDonePart(ipos_right_hand))
            {
                break;
            }
            Time::delay(1.0);
        }

        yInfo()<<"Homing"<<part<<"concluded";
        return true;
    }

    /****************************************************************/
    bool stopCartesian()
    {
        bool ret=false;
        if (gazePort.getOutputCount()>0)
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("stop"));
            if (gazePort.write(cmd,rep))
            {
                ret=(rep.get(0).asVocab()==ack);
            }
        }

        yInfo()<<"Cartesian movements stopped";
        return ret;
    }

    /****************************************************************/
    bool getObjectPosition3D(const string &object, Vector &x)
    {
        bool ret=false;
        x.resize(3,0.0);
        if (opcPort.getOutputCount()>0)
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("ask"));
            Bottle &content=cmd.addList();
            Bottle &cond_1=content.addList();
            cond_1.addString("entity");
            cond_1.addString("==");
            cond_1.addString("object");
            content.addString("&&");
            Bottle &cond_2=content.addList();
            cond_2.addString("name");
            cond_2.addString("==");
            cond_2.addString(object);

            opcPort.write(cmd,rep);
            if (rep.size()>1)
            {
                if (rep.get(0).asVocab()==ack)
                {
                    if (Bottle *idField=rep.get(1).asList())
                    {
                        if (Bottle *idValues=idField->get(1).asList())
                        {
                            Bottle cmd,rep;
                            int id=idValues->get(0).asInt();                            
                            cmd.addVocab(Vocab::encode("get"));
                            Bottle &content=cmd.addList();
                            Bottle &list_bid=content.addList();
                            list_bid.addString("id");
                            list_bid.addInt(id);
                            Bottle &list_propSet=content.addList();
                            list_propSet.addString("propSet");
                            list_propSet.addList().addString("position_3d");
                            
                            opcPort.write(cmd,rep);
                            if (rep.get(0).asVocab()==ack)
                            {
                                if (Bottle *propField=rep.get(1).asList())
                                {
                                    if (Bottle *b=propField->find("position_3d").asList())
                                    {
                                        size_t len=std::min(x.length(),b->size());
                                        for (size_t i=0; i<len; i++)
                                            x[i]=b->get(i).asDouble();
                                        ret=true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        return ret; 
    }

    /****************************************************************/
    bool look(const int u, const int v)
    {
        Vector p(2); p[0]=u; p[1]=v;
        Bottle target;
        target.addList().read(p);

        Property prop;
        prop.put("control-frame","depth");
        prop.put("target-type","image");
        prop.put("image","depth");
        prop.put("target-location",target.get(0));
        return look(prop);
    }

    /****************************************************************/
    bool look(const Vector &x)
    {
        Bottle target;
        target.addList().read(x);

        Property prop;
        prop.put("control-frame","depth");
        prop.put("target-type","cartesian");
        prop.put("target-location",target.get(0));
        return look(prop);
    }

    /****************************************************************/
    bool look(const Property &prop)
    {
        bool ret=false;
        if (gazePort.getOutputCount()>0)
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("look"));
            cmd.addList().read(prop);
            if (gazePort.write(cmd,rep))
            {
                ret=(rep.get(0).asVocab()==ack);
            }
            yInfo()<<cmd.toString();
        }
        return ret;
    }

    /****************************************************************/
    void waitUntilFixating()
    {
        while (!interrupting && (gazePort.getOutputCount()>0))
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("get"));
            cmd.addString("done");
            if (gazePort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==ack)
                {
                    if (rep.get(1).asInt()>0)
                    {
                        break;
                    }
                }
                else
                    break;
            }
            Time::delay(std::min(10.0*period,1.0));
        }
        yInfo()<<"Attained fixation";
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        drivers=vector<PolyDriver>(6);
        bool drivers_ok=true;
        drivers_ok&=openDriver(drivers[0],"head",imod_head,ipos_head);
        drivers_ok&=openDriver(drivers[1],"torso",imod_torso,ipos_torso);
        drivers_ok&=openDriver(drivers[2],"left_arm",imod_left_arm,ipos_left_arm);
        drivers_ok&=openDriver(drivers[3],"left_hand",imod_left_hand,ipos_left_hand);
        drivers_ok&=openDriver(drivers[4],"right_arm",imod_right_arm,ipos_right_arm);
        drivers_ok&=openDriver(drivers[5],"right_hand",imod_right_hand,ipos_right_hand);
        if (!drivers_ok)
        {
            yError()<<"Unable to talk to device drivers";
            close();
            return false;
        }

        // default values
        robot="cer";
        period=0.01;
        ack=Vocab::encode("ack");
        nack=Vocab::encode("nack");
        interrupting=false;

        // retrieve values from config file
        Bottle &gGeneral=rf.findGroup("general");
        if (!gGeneral.isNull())
        {
            robot=gGeneral.check("robot",Value(robot)).asString();
            period=gGeneral.check("period",Value(period)).asDouble();
        }

        Bottle &gHome=rf.findGroup("home");
        if (!gHome.isNull())
        {
            getHomeInfo(gHome,"head",home.head,ipos_head);
            getHomeInfo(gHome,"torso",home.torso,ipos_torso);
            getHomeInfo(gHome,"left_arm",home.left_arm,ipos_left_arm);
            getHomeInfo(gHome,"left_hand",home.left_hand,ipos_left_hand);
            getHomeInfo(gHome,"right_arm",home.right_arm,ipos_right_arm);
            getHomeInfo(gHome,"right_hand",home.right_hand,ipos_right_hand);
            home.speed_angular=gHome.check("speed-angular",Value(10.0)).asDouble();
            home.speed_linear=gHome.check("speed-linear",Value(0.01)).asDouble();
        }

        cmdPort.open("/action-gateway/cmd:io");
        opcPort.open("/action-gateway/opc/rpc");
        gazePort.open("/action-gateway/gaze/rpc");

        attach(cmdPort);
        return true;
    }

    /****************************************************************/
    double getPeriod() override
    {
        return period;
    }

    /****************************************************************/
    bool updateModule() override
    {
        return true;
    }

    /****************************************************************/
    bool respond(const Bottle &command, Bottle &reply) override
    {
        bool ok=false;
        int cmd=command.get(0).asVocab();
        if (cmd==Vocab::encode("home"))
        {
            string part="all";
            if (command.size()>=2)
            {
                part=command.get(1).asString();
            }
            ok=goHome(part);
        }
        else if ((cmd==Vocab::encode("look")) && (command.size()>=2))
        {
            Value &target=command.get(1);
            if (target.isList())
            {
                if (Bottle *b=target.asList())
                {
                    if (b->size()>=3)
                    {
                        int u,v;
                        u=b->get(1).asInt();
                        v=b->get(2).asInt();
                        ok=look(u,v);
                    }
                }
            }
            else if (target.isString())
            {
                Vector x;
                if (getObjectPosition3D(target.asString(),x))
                {
                    ok=look(x);
                }
            }

            if (ok && (command.size()>=3))
            {
                if (command.get(2).asString()=="wait")
                {
                    waitUntilFixating();
                }
            }
        }

        reply.addVocab(ok?ack:nack);
        return true;
    }

    /****************************************************************/
    bool interruptModule() override
    {
        interrupting=true;
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        if (cmdPort.asPort().isOpen())
        {
            cmdPort.close();
        }
        if (opcPort.asPort().isOpen())
        {
            opcPort.close();
        }
        if (gazePort.asPort().isOpen())
        {
            gazePort.close();
        }
        for (auto &d:drivers)
        {
            if (d.isValid())
            {
                d.close();
            }
        }
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
    rf.setDefaultContext("action-gateway");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    Gateway gateway;
    return gateway.runModule(rf);
}