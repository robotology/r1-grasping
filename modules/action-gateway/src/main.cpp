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
    RpcClient reachLPort;
    RpcClient reachRPort;

    string robot;
    double period;
    int ack,nack;
    bool interrupting;
    Vector pose_latch;
    string part_latch;

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

    struct {
        string mode;
        double torso_heave;
        double lower_arm_heave;
        vector<double> left_hand;
        vector<double> right_hand;
        double speed_angular;
        double lift;
    } grasping;

    vector<PolyDriver> drivers;
    IControlMode     *imod_head,*imod_torso,*imod_left_arm,*imod_left_hand,
                     *imod_right_arm,*imod_right_hand;
    IPositionControl *ipos_head,*ipos_torso,*ipos_left_arm,*ipos_left_hand,
                     *ipos_right_arm,*ipos_right_hand;

    /****************************************************************/
    bool getVectorInfo(const Bottle &opt, const string &part, vector<double> &v,
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
            ipos_torso->positionMove(home.torso.data());
        }
        if (!home.left_arm.empty() && (part.empty() || (part=="all") || (part=="left") || (part=="left_arm")))
        {
            vector<int> modes(home.left_arm.size(),VOCAB_CM_POSITION);
            vector<double> accs(home.left_arm.size(),numeric_limits<double>::max());
            vector<double> spds(home.left_arm.size(),home.speed_angular);
            spds[5]=home.speed_linear;  // heave

            imod_left_arm->setControlModes(modes.data());
            ipos_left_arm->setRefAccelerations(accs.data());
            ipos_left_arm->setRefSpeeds(spds.data());
            ipos_left_arm->positionMove(home.left_arm.data());
        }
        if (!home.left_hand.empty() && (part.empty() || (part=="all") || (part=="left") || (part=="left_hand")))
        {
            vector<int> modes(home.left_hand.size(),VOCAB_CM_POSITION);
            vector<double> accs(home.left_hand.size(),numeric_limits<double>::max());
            vector<double> spds(home.left_hand.size(),home.speed_angular);

            imod_left_hand->setControlModes(modes.data());
            ipos_left_hand->setRefAccelerations(accs.data());
            ipos_left_hand->setRefSpeeds(spds.data());
            ipos_left_hand->positionMove(home.left_hand.data());
        }
        if (!home.right_arm.empty() && (part.empty() || (part=="all") || (part=="right") || (part=="right_arm")))
        {
            vector<int> modes(home.right_arm.size(),VOCAB_CM_POSITION);
            vector<double> accs(home.right_arm.size(),numeric_limits<double>::max());
            vector<double> spds(home.right_arm.size(),home.speed_angular);
            spds[5]=home.speed_linear;  // heave

            imod_right_arm->setControlModes(modes.data());
            ipos_right_arm->setRefAccelerations(accs.data());
            ipos_right_arm->setRefSpeeds(spds.data());
            ipos_right_arm->positionMove(home.right_arm.data());
        }
        if (!home.right_hand.empty() && (part.empty() || (part=="all") || (part=="right") || (part=="left_hand")))
        {
            vector<int> modes(home.right_hand.size(),VOCAB_CM_POSITION);
            vector<double> accs(home.right_hand.size(),numeric_limits<double>::max());
            vector<double> spds(home.right_hand.size(),home.speed_angular);

            imod_right_hand->setControlModes(modes.data());
            ipos_right_hand->setRefAccelerations(accs.data());
            ipos_right_hand->setRefSpeeds(spds.data());
            ipos_right_hand->positionMove(home.right_hand.data());
        }

        while (!interrupting)
        {
            if (checkMotionDonePart(ipos_head) && checkMotionDonePart(ipos_torso) &&
                checkMotionDonePart(ipos_left_arm) && checkMotionDonePart(ipos_left_hand) &&
                checkMotionDonePart(ipos_right_arm) && checkMotionDonePart(ipos_right_hand))
            {
                break;
            }
            Time::delay(1.0);
        }

        yInfo()<<"Homing"<<part<<"complete";
        return true;
    }

    /****************************************************************/
    bool stopCartesianHelper(RpcClient &port)
    {
        if (port.getOutputCount()>0)
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("stop"));
            if (port.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==ack)
                {
                    return true;
                }
            }
        }
        return false;
    }

    /****************************************************************/
    bool stopCartesian(const string &part="all")
    {
        bool ret=true;
        if ((part=="all") || (part=="gaze") || (part=="head"))
        {
            if (stopCartesianHelper(gazePort))
            {
                yInfo()<<"Gaze movements stopped";
            }
            else
            {
                yWarning()<<"Problems detected while stopping gaze movements";
                ret=false;
            }
        }
        if ((part=="all") || (part=="left"))
        {
            if (stopCartesianHelper(reachLPort))
            {
                yInfo()<<"Left-arm movements stopped";
            }
            else
            {
                yWarning()<<"Problems detected while stopping left-arm movements";
                ret=false;
            }
        }
        if ((part=="all") || (part=="right"))
        {
            if (stopCartesianHelper(reachRPort))
            {
                yInfo()<<"Right-arm movements stopped";
            }
            else
            {
                yWarning()<<"Problems detected while stopping right-arm movements";
                ret=false;
            }
        }
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
    bool closeHand(const string &part)
    {
        vector<double> *hand=nullptr;
        IPositionControl *ipos=nullptr;
        IControlMode *imod=nullptr;

        if (part=="left")
        {
            hand=&grasping.left_hand;
            imod=imod_left_hand;
            ipos=ipos_left_hand;
        }
        else if (part=="right")
        {
            hand=&grasping.right_hand;
            imod=imod_right_hand;
            ipos=ipos_right_hand;
        }
        else
        {
            yError()<<"Unrecognized hand";
            return false;
        }

        yInfo()<<"Start closing hand"<<part;
        if (!hand->empty())
        {
            vector<int> modes(hand->size(),VOCAB_CM_POSITION);
            vector<double> accs(hand->size(),numeric_limits<double>::max());
            vector<double> spds(hand->size(),grasping.speed_angular);

            imod->setControlModes(modes.data());
            ipos->setRefAccelerations(accs.data());
            ipos->setRefSpeeds(spds.data());
            ipos->positionMove(hand->data());
        }

        while (!interrupting)
        {
            if (checkMotionDonePart(ipos))
            {
                break;
            }
            Time::delay(1.0);
        }

        yInfo()<<"Closing hand"<<part<<"complete";
        return true;
    }

    /****************************************************************/
    bool reach(const Vector &pose, const string &part="select")
    {
        if (pose.length()<7)
        {
            yError()<<"Too few parameters given for reaching";
            return false;
        }

        string part_=part;
        if ((part_!="left") && (part_!="right"))
        {
            part_=(pose[1]>0.0?"left":"right");
        }
        RpcClient *port=&(part_=="left"?reachLPort:reachRPort);

        Bottle params;
        Bottle &params_info1=params.addList();
        params_info1.addString("parameters");
        Bottle &params_info2=params_info1.addList();

        Bottle mode=params_info2.addList();
        mode.addString("mode");
        mode.addString(grasping.mode);

        Bottle &torso_heave=params_info2.addList();
        torso_heave.addString("torso_heave");
        torso_heave.addDouble(grasping.torso_heave);

        Bottle &lower_arm_heave=params_info2.addList();
        lower_arm_heave.addString("lower_arm_heave");
        lower_arm_heave.addDouble(grasping.lower_arm_heave);

        Bottle target;
        Bottle &target_info=target.addList();
        target_info.addString("target");
        target_info.addList().read(pose);

        Bottle cmd,rep;
        cmd.addVocab(Vocab::encode("go"));
        Bottle &cmd_info=cmd.addList();
        cmd_info.append(params);
        cmd_info.append(target);
        if (port->write(cmd,rep))
        {
            yInfo()<<cmd.toString();
            if (rep.get(0).asVocab()==ack)
            {
                return waitUntilDone(*port);
            }
        }
        return false;
    }

    /****************************************************************/
    bool grasp(const Vector &pose, const Vector &approach,
               const string &part="select")
    {
        if ((pose.length()<7) || (approach.length()<4))
        {
            yError()<<"Too few parameters given for grasping";
            return false;
        }

        string part_=part;
        if ((part_!="left") && (part_!="right"))
        {
            part_=(pose[1]>0.0?"left":"right");
        }

        Vector pose_approach=pose;
        pose_approach[0]+=approach[0];
        pose_approach[1]+=approach[1];
        pose_approach[2]+=approach[2];

        if (reach(pose_approach,part_))
        {
            if (reach(pose,part_))
            {
                if (closeHand(part_))
                {
                    Vector pose_lift=pose;
                    pose_lift[2]+=grasping.lift;
                    return reach(pose_lift,part_);
                }
            }
        }
        return false;
    }

    /****************************************************************/
    bool waitUntilDone(RpcClient &port)
    {
        string part="Gaze";
        if (&port==&reachLPort)
        {
            part="Left-arm";
        }
        else if (&port==&reachRPort)
        {
            part="Right-arm";
        }

        while (!interrupting && (port.getOutputCount()>0))
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("get"));
            cmd.addString("done");
            if (port.write(cmd,rep))
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
        yInfo()<<part<<"movements complete";
        return (!interrupting);
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
            getVectorInfo(gHome,"head",home.head,ipos_head);
            getVectorInfo(gHome,"torso",home.torso,ipos_torso);
            getVectorInfo(gHome,"left_arm",home.left_arm,ipos_left_arm);
            getVectorInfo(gHome,"left_hand",home.left_hand,ipos_left_hand);
            getVectorInfo(gHome,"right_arm",home.right_arm,ipos_right_arm);
            getVectorInfo(gHome,"right_hand",home.right_hand,ipos_right_hand);
            home.speed_angular=gHome.check("speed_angular",Value(10.0)).asDouble();
            home.speed_linear=gHome.check("speed_linear",Value(0.01)).asDouble();
        }

        Bottle &gGrasping=rf.findGroup("grasping");
        if (!gGrasping.isNull())
        {
            grasping.mode=gGrasping.check("mode",Value("full_pose+no_torso_no_heave")).asString();
            grasping.torso_heave=gGrasping.check("torso_heave",Value(0.1)).asDouble();
            grasping.lower_arm_heave=gGrasping.check("lower_arm_heave",Value(0.05)).asDouble();
            getVectorInfo(gGrasping,"left_hand",grasping.left_hand,ipos_left_hand);
            getVectorInfo(gGrasping,"right_hand",grasping.right_hand,ipos_right_hand);
            grasping.speed_angular=gGrasping.check("speed_angular",Value(10.0)).asDouble();
            grasping.lift=gGrasping.check("lift",Value(0.1)).asDouble();
        }

        cmdPort.open("/action-gateway/cmd:io");
        opcPort.open("/action-gateway/opc/rpc");
        gazePort.open("/action-gateway/gaze/rpc");
        reachLPort.open("/action-gateway/reach/left/rpc");
        reachRPort.open("/action-gateway/reach/right/rpc");

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
                    waitUntilDone(gazePort);
                }
            }
        }
        else if ((cmd==Vocab::encode("grasp")) && (command.size()>=3))
        {
            Vector pose,approach;
            string part="select";
            if (Bottle *b=command.get(1).asList())
            {
                for (size_t i=1; i<b->size(); i++)
                {
                    pose.push_back(b->get(i).asDouble());
                }
            }
            if (Bottle *b=command.get(2).asList())
            {
                for (size_t i=1; i<b->size(); i++)
                {
                    approach.push_back(b->get(i).asDouble());
                }
            }
            if (command.size()>=4)
            {
                part=command.get(3).asString();
            }

            ok=grasp(pose,approach,part);
            pose_latch=pose;
            part_latch=part;
        }
        else if (cmd==Vocab::encode("drop"))
        {
            if (reach(pose_latch,part_latch))
            {
                if (goHome(part_latch+"_hand"))
                {
                    ok=goHome(part_latch);
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
        if (reachLPort.asPort().isOpen())
        {
            reachLPort.close();
        }
        if (reachRPort.asPort().isOpen())
        {
            reachRPort.close();
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
