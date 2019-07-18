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
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

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
    RpcClient depthPort;
    RpcClient gazePort;
    RpcClient reachLPort;
    RpcClient reachRPort;
    RpcClient mobReachLPort;
    RpcClient mobReachRPort;
    
    BufferedPort<Bottle> stopMotorsPort;
    class StopMotorsProcessor : public TypedReaderCallback<Bottle>,
                                public PortReader
    {
        Gateway *gateway;
        void onRead(Bottle& b) override { }
        bool read(ConnectionReader& connection) override
        {
            Bottle command,reply;
            command.read(connection);
            int cmd=command.get(0).asVocab();
            if (cmd==Vocab::encode("interrupt"))
            {
                gateway->stopCartesian();
                gateway->interrupted=true;
                reply.addString("interrupted");
            }
            else if (cmd==Vocab::encode("reinstate"))
            {
                gateway->interrupted=false;
                gateway->goHome("all");
                reply.addString("reinstated");
            }
            else
            {
                reply.addString("command not recognized");
            }
            ConnectionWriter *returnToSender=connection.getWriter();
            if (returnToSender!=nullptr)
            {
                reply.write(*returnToSender);
            }
            return true;
        };
    public:
        StopMotorsProcessor(Gateway *g) : gateway(g) { }
    } stopMotorsProcessor;
    friend class StopMotorsProcessor;

    BufferedPort<ImageOf<PixelRgb>> imgInPort;
    BufferedPort<ImageOf<PixelRgb>> imgOutPort;

    string robot;
    double period;
    double speed_hand;
    int ack,nack;
    bool exiting;
    bool interrupted;
    Vector latch_pose;
    Vector latch_approach;
    string latch_part;
    bool gaze_track;

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
        double lift;
    } grasping;

    struct {
        double H_offset;
        double V_offset;
        double init_inclin;
        double final_inclin;
        Vector current_hand_pose;
    } pouring;

    struct {
        double default_height;
        int num_pixels;
        vector<int> pixels_bounds;
        double ransac_threshold;
    } table;

    vector<PolyDriver> drivers;
    IControlMode     *imod_head,*imod_torso,*imod_left_arm,*imod_left_hand,
                     *imod_right_arm,*imod_right_hand;
    IPositionControl *ipos_head,*ipos_torso,*ipos_left_arm,*ipos_left_hand,
                     *ipos_right_arm,*ipos_right_hand;

    /****************************************************************/
    template<typename T>
    bool getVectorInfo(const Bottle &opt, const string &tag, vector<T> &v,
                       IPositionControl *ipos=nullptr)
    {
        v.clear();
        if (Bottle *b=opt.find(tag).asList())
        {
            size_t len=b->size();
            if (ipos!=nullptr)
            {
                int nAxes;
                ipos->getAxes(&nAxes);
                v.assign((size_t)nAxes,0);
                len=std::min(len,v.size());
            }
            else
            {
                v.assign(len,0);
            }
            for (size_t i=0; i<len; i++)
            {
                v[i]=(T)b->get(i).asDouble();
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
        for (int i=0; i<nAxes; i++)
        {
            bool done;
            ipos->checkMotionDone(i,&done);
            if (!done)
            {
                return false;
            }
        }
        return true;
    }

    /****************************************************************/
    string choosePart(const string &part, const Vector &pose)
    {
        string part_=part;
        if ((part_!="left") && (part_!="right"))
        {
            part_=(pose[1]>0.0?"left":"right");
        }
        return part_;
    }

    /****************************************************************/
    bool goHome(const string &part)
    {
        if (!part.empty() && (part!="all") && (part!="head") && (part!="gaze") &&
            (part!="torso") && (part!="left") && (part!="right") &&
            (part!="left_arm") && (part!="left_hand") &&
            (part!="right_arm") && (part!="right_hand") &&
            (part!="arms"))
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
        if (!home.torso.empty() && (part.empty() || (part=="all") || (part=="torso") || (part=="left_arm") || (part=="right_arm") ||
            (part=="left") || (part=="right") || (part=="arms")))
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
        if (!home.left_arm.empty() && (part.empty() || (part=="all") || (part=="left") || (part=="left_arm") || (part=="arms")))
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
        if (!home.left_hand.empty() && (part.empty() || (part=="all") || (part=="left") || (part=="left_hand") || (part=="arms")))
        {
            vector<int> modes(home.left_hand.size(),VOCAB_CM_POSITION);
            vector<double> accs(home.left_hand.size(),numeric_limits<double>::max());
            vector<double> spds(home.left_hand.size(),speed_hand);

            imod_left_hand->setControlModes(modes.data());
            ipos_left_hand->setRefAccelerations(accs.data());
            ipos_left_hand->setRefSpeeds(spds.data());
            ipos_left_hand->positionMove(home.left_hand.data());
        }
        if (!home.right_arm.empty() && (part.empty() || (part=="all") || (part=="right") || (part=="right_arm") || (part=="arms")))
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
        if (!home.right_hand.empty() && (part.empty() || (part=="all") || (part=="right") || (part=="right_hand") || (part=="arms")))
        {
            vector<int> modes(home.right_hand.size(),VOCAB_CM_POSITION);
            vector<double> accs(home.right_hand.size(),numeric_limits<double>::max());
            vector<double> spds(home.right_hand.size(),speed_hand);

            imod_right_hand->setControlModes(modes.data());
            ipos_right_hand->setRefAccelerations(accs.data());
            ipos_right_hand->setRefSpeeds(spds.data());
            ipos_right_hand->positionMove(home.right_hand.data());
        }

        while (!exiting && !interrupted)
        {
            Time::delay(1.0);
            if (checkMotionDonePart(ipos_head) && checkMotionDonePart(ipos_torso) &&
                checkMotionDonePart(ipos_left_arm) && checkMotionDonePart(ipos_left_hand) &&
                checkMotionDonePart(ipos_right_arm) && checkMotionDonePart(ipos_right_hand))
            {
                break;
            }
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
    bool getTableHeightOPC(double &h)
    {
        if (opcPort.getOutputCount()>0)
        {
            Bottle cmd1,rep1;
            cmd1.addVocab(Vocab::encode("ask"));
            Bottle &content=cmd1.addList().addList();
            content.addString("entity");
            content.addString("==");
            content.addString("table");

            opcPort.write(cmd1,rep1);
            if (rep1.get(0).asVocab()==ack)
            {
                Bottle *payLoad=rep1.get(1).asList()->find("id").asList();
                if (payLoad->size()>0)
                {
                    int id=payLoad->get(0).asInt();

                    Bottle cmd2,rep2;
                    cmd2.addVocab(Vocab::encode("get"));
                    Bottle &content=cmd2.addList();

                    Bottle &content_id=content.addList();
                    content_id.addString("id");
                    content_id.addInt(id);

                    opcPort.write(cmd2,rep2);
                    if (rep2.get(0).asVocab()==ack)
                    {
                        Bottle *payLoad=rep2.get(1).asList();
                        if (payLoad->check("height"))
                        {
                            h=payLoad->find("height").asDouble();
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    /****************************************************************/
    bool setTableHeightOPC(const double h)
    {
        bool ret=false;
        if (opcPort.getOutputCount()>0)
        {
            Bottle cmd1,rep1;
            cmd1.addVocab(Vocab::encode("ask"));
            Bottle &content=cmd1.addList().addList();
            content.addString("entity");
            content.addString("==");
            content.addString("table");

            opcPort.write(cmd1,rep1);
            if (rep1.get(0).asVocab()==ack)
            {
                Bottle cmd2,rep2;
                Bottle *payLoad=rep1.get(1).asList()->find("id").asList();
                if (payLoad->size()==0)
                {
                    cmd2.addVocab(Vocab::encode("add"));
                    Bottle &content=cmd2.addList();

                    Bottle &content_entity=content.addList();
                    content_entity.addString("entity");
                    content_entity.addString("table");

                    Bottle &content_table=content.addList();
                    content_table.addString("height");
                    content_table.addDouble(h);
                }
                else
                {
                    int id=payLoad->get(0).asInt();
                    cmd2.addVocab(Vocab::encode("set"));
                    Bottle &content=cmd2.addList();

                    Bottle &content_id=content.addList();
                    content_id.addString("id");
                    content_id.addInt(id);

                    Bottle &content_table=content.addList();
                    content_table.addString("height");
                    content_table.addDouble(h);
                }
                opcPort.write(cmd2,rep2);
                ret=(rep2.get(0).asVocab()==ack);
            }
        }
        if (ret)
        {
            yInfo()<<"Table height pushed in OPC";
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
            vector<double> spds(hand->size(),speed_hand);

            imod->setControlModes(modes.data());
            ipos->setRefAccelerations(accs.data());
            ipos->setRefSpeeds(spds.data());
            ipos->positionMove(hand->data());
        }

        while (!exiting && !interrupted)
        {
            Time::delay(1.0);
            if (checkMotionDonePart(ipos))
            {
                break;
            }
        }

        yInfo()<<"Closing hand"<<part<<"complete";
        return true;
    }

    /****************************************************************/
    Bottle prepareReachingParams() const
    {
        Bottle params;
        Bottle &params_info1=params.addList();
        params_info1.addString("parameters");
        Bottle &params_info2=params_info1.addList();

        Bottle &mode=params_info2.addList();
        mode.addString("mode");
        mode.addString(grasping.mode);

        Bottle &torso_heave=params_info2.addList();
        torso_heave.addString("torso_heave");
        torso_heave.addDouble(grasping.torso_heave);

        Bottle &lower_arm_heave=params_info2.addList();
        lower_arm_heave.addString("lower_arm_heave");
        lower_arm_heave.addDouble(grasping.lower_arm_heave);
        return params;
    }

    /****************************************************************/
    Bottle prepareReachingTarget(const Vector &pose) const
    {
        Bottle target;
        Bottle &target_info=target.addList();
        target_info.addString("target");
        target_info.addList().read(pose);
        return target;
    }

    /****************************************************************/
    Bottle prepareMobileGraspingTargets(const Vector &pose, const Vector &approach, const Vector &noise=Vector(2,0.0)) const
    {
        Bottle target;
        Bottle &target_info=target.addList();
        target_info.addString("target");
        Bottle &target_list=target_info.addList();

        Vector approachPose = processApproach(pose, approach);
        Vector liftPose = pose;
        liftPose[2] += grasping.lift;

        target_list.addList().read(approachPose);
        target_list.addList().read(pose);
        target_list.addList().read(liftPose);

        Bottle &margin_info=target.addList();
        margin_info.addString("margin");
        margin_info.addList().read(cat(Vector(3,noise[0]), Vector(3,noise[1])));

        return target;
    }

    /****************************************************************/
    bool reach(const Vector &pose, const string &part="select")
    {
        if (pose.length()<7)
        {
            yError()<<"Too few parameters given for reaching";
            return false;
        }

        string part_=choosePart(part,pose);
        RpcClient *port=&(part_=="left"?reachLPort:reachRPort);

        Bottle cmd,rep;
        cmd.addVocab(Vocab::encode("go"));
        Bottle &cmd_info=cmd.addList();
        cmd_info.append(prepareReachingParams());
        cmd_info.append(prepareReachingTarget(pose));
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
    bool ask(Bottle &payLoad, const Vector &pose, const string &part="select")
    {
        if (pose.length()<7)
        {
            yError()<<"Too few parameters given for reaching";
            return false;
        }

        string part_=choosePart(part,pose);
        RpcClient *port=&(part_=="left"?reachLPort:reachRPort);

        Bottle cmd,rep;
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &cmd_info=cmd.addList();
        cmd_info.append(prepareReachingParams());
        cmd_info.append(prepareReachingTarget(pose));
        if (port->write(cmd,rep))
        {
            yInfo()<<cmd.toString();
            if (rep.get(0).asVocab()==ack)
            {
                payLoad=rep.tail();
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    bool askMobileGrasp(Bottle &payLoad, const Vector &pose, const Vector &approach, const string &part="select", const Vector &noise=Vector(2,0.0))
    {
        if (pose.length()<7)
        {
            yError()<<"Too few pose parameters given for mobile reaching";
            return false;
        }

        if (part!="right" && part!="left")
        {
            yError()<<"Invalid part name given for mobile reaching (should be \"right\" or \"left\")";
            return false;
        }

        RpcClient *port=&(part=="left"?mobReachLPort:mobReachRPort);

        Bottle cmd,rep;
        cmd.addVocab(Vocab::encode("askLocal"));
        Bottle &cmd_info=cmd.addList();
        cmd_info.append(prepareReachingParams());
        cmd_info.append(prepareMobileGraspingTargets(pose, approach, noise));
        if (port->write(cmd,rep))
        {
            yInfo()<<"cmd" << cmd.toString();
            if (rep.get(0).asVocab()==ack)
            {
                yInfo()<< "rep" << rep.toString();
                payLoad=rep.tail();
                Bottle *joints=payLoad.find("q").asList();
                Bottle j = *(joints->get(1).asList());
                joints->clear();
                joints->read(j);
                Bottle *target=payLoad.find("x").asList();
                Bottle t = *(target->get(1).asList());
                target->clear();
                target->read(t);
                yInfo() << "rep fixed" << payLoad.toString();
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    Vector processApproach(const Vector &pose, const Vector &approach) const
    {
        Vector x=pose.subVector(0,2);
        Vector o=pose.subVector(3,6);

        Matrix R=axis2dcm(o);
        Vector y=R.getCol(1);
        y[3]=(M_PI/180.0)*approach[3];

        Vector dx=approach[0]*R.getCol(0).subVector(0,2);
        Vector dy=approach[1]*R.getCol(1).subVector(0,2);
        Vector dz=approach[2]*R.getCol(2).subVector(0,2);

        Vector pose_approach=x+dx+dy+dz;
        pose_approach=cat(pose_approach,dcm2axis(axis2dcm(y)*R));
        return pose_approach;
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

        string part_=choosePart(part,pose);
        if (goHome(part+"_hand"))
        {
            if (reach(processApproach(pose,approach),part_))
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
        }
        return false;
    }

    /****************************************************************/
    bool pouringApproach(const Vector &source, const Vector &destination,
                         const string &part="select")
    {
        /* source = where the liquid comes from expressed in the grabber frame (hand frame)
         *          for example, the neck of the currently grasped bottle
         * destination = where the liquid should be poured expressed in the reference frame
         *               for example, the top of a glass
         */

        if ((source.length()<3) || (destination.length()!=3))
        {
            yError()<<"Invalid parameters given for pouring approach";
            return false;
        }

        string part_=choosePart(part, destination);

        int part_sign=(part_=="left"?1:-1);

        // Find easiest hand orientation for pouring

        Vector o_tmp(4);
        o_tmp[0]=1.0;
        o_tmp[1]=0.0;
        o_tmp[2]=0.0;
        o_tmp[3]=-M_PI/2.0;
        Matrix refR=axis2dcm(o_tmp).submatrix(0,2,0,2);

        const int nb_test_orientation=9;
        vector<Vector> hand_poses(nb_test_orientation, Vector(7));
        vector<double> reaching_errors(nb_test_orientation);

        for(int i=0 ; i<nb_test_orientation ; i++)
        {
            // Add rotation around source
            o_tmp[0]=0.0;
            o_tmp[1]=1.0;
            o_tmp[2]=0.0;
            o_tmp[3]=part_sign*(-M_PI/2.0+i*M_PI/(nb_test_orientation-1));
            Matrix R1=axis2dcm(o_tmp).submatrix(0,2,0,2);

            // Add initial inclination
            o_tmp[0]=1.0;
            o_tmp[1]=0.0;
            o_tmp[2]=0.0;
            o_tmp[3]=part_sign*M_PI/180*pouring.init_inclin;
            Matrix R2=axis2dcm(o_tmp).submatrix(0,2,0,2);

            // Hand orientation
            Matrix hand_rotation=refR*R1*R2;
            Vector hand_orientation=dcm2axis(hand_rotation);

            // Final source position in ref frame
            Vector final_source_position(destination);

            final_source_position[0] += -part_sign*pouring.H_offset*hand_rotation(1,0);
            final_source_position[1] += part_sign*pouring.H_offset*hand_rotation(0,0);
            final_source_position[2] += pouring.V_offset;

            // Hand position in ref frame
            Vector hand_position=final_source_position-hand_rotation*source;

            // Final hand pose
            hand_poses.at(i).setSubvector(0,hand_position);
            hand_poses.at(i).setSubvector(3, hand_orientation);

            // Ask kinematics feasibility
            Bottle achieved_bottle;
            ask(achieved_bottle, hand_poses.at(i), part_);

            // Compute error
            Bottle *achieved_pose_bottle=achieved_bottle.find("x").asList();
            if(achieved_pose_bottle==nullptr)
                reaching_errors.at(i)=std::numeric_limits<double>::max();
            else
            {
                Vector achieved_pose(7);
                for(int j=0 ; j<7 ; j++)
                    achieved_pose[j]=achieved_pose_bottle->get(j).asDouble();

                Vector achieved_source_position=achieved_pose.subVector(0,2)+axis2dcm(achieved_pose.subVector(3,6)).submatrix(0,2,0,2)*source;
                reaching_errors.at(i)=norm(achieved_source_position-final_source_position);
            }
        }

        int min_index=0;
        double min_value =reaching_errors.front();
        for(int i=1 ; i<reaching_errors.size() ; i++)
        {
            if(reaching_errors.at(i) < min_value)
            {
                min_index=i;
                min_value = reaching_errors.at(i);
            }
        }

        pouring.current_hand_pose = hand_poses.at(min_index);
        return reach(pouring.current_hand_pose,part_);
    }

    /****************************************************************/
    bool pouringMotion(const Vector &source, double angle, const string &part="select")
    {
        /* source = where the liquid comes from expressed in the grabber frame (hand frame)
         *          for example, the neck of the currently grasped bottle
         * angle = angle of rotation of the pouring motion
         */

        if ((source.length()!=3) || (part!="right" && part!="left"))
        {
            yError()<<"Invalid parameters given for pouring motion";
            return false;
        }

        // Rotation in hand frame
        Vector o(4);
        o[0]=1.0;
        o[1]=o[2]=0.0;
        o[3]=(part=="left"?1:-1)*M_PI/180*angle;
        Matrix R=axis2dcm(o).submatrix(0,2,0,2);

        // Current hand position and orientation in ref frame
        Vector current_hand_position=pouring.current_hand_pose.subVector(0,2);
        Matrix current_hand_rotation=axis2dcm(pouring.current_hand_pose.subVector(3,6)).submatrix(0,2,0,2);

        // Final hand orientation in ref frame
        Matrix hand_rotation=current_hand_rotation*R;
        Vector hand_orientation=dcm2axis(hand_rotation);

        // Position of center of rotation in ref frame
        Vector current_source_position=current_hand_rotation*source+current_hand_position;

        // Pouring transformation expressed in ref frame
        Matrix transform_rotation=current_hand_rotation*R*current_hand_rotation.transposed();
        Vector transform_translation=current_source_position-transform_rotation*current_source_position;

        // Final hand pose in ref frame
        Vector hand_pose(7);
        hand_pose.setSubvector(0,transform_rotation*current_hand_position+transform_translation);
        hand_pose.setSubvector(3, hand_orientation);

        pouring.current_hand_pose = hand_pose;
        return reach(hand_pose,part);
    }

    /****************************************************************/
    bool pour(const Vector &source, const Vector &destination)
    {
        /* source = where the liquid comes from expressed in the grabber frame (hand frame)
         *          for example, the neck of the currently grasped bottle
         * destination = where the liquid should be poured expressed in the reference frame
         *               for example, the top of a glass
         */

        if ((source.length()!=3) || (destination.length()!=3))
        {
            yError()<<"Invalid dimensions of parameters given for pouring";
            return false;
        }

        if (latch_part!="right" && latch_part!="left")
        {
            yError()<<"no object was previously grasped";
            return false;
        }

        yInfo() << "start pouring approach";
        if(pouringApproach(source, destination, latch_part))
        {
            yInfo() << "pouring approach done, start pouring motion";
            if(pouringMotion(source, pouring.final_inclin-pouring.init_inclin, latch_part))
            {
                // Wait a bit for the pouring 
                Time::delay(5.0);

                yInfo() << "pouring motion done, start pouring stop motion";
                if(pouringMotion(source, -(pouring.final_inclin-pouring.init_inclin), latch_part))
                {
                    Vector pose_end=latch_pose;
                    pose_end[2]+=grasping.lift;
                    yInfo() << "pouring stop motion done, start homing";
                    return reach(pose_end,latch_part);
                }
                else yInfo() << "pouring stop motion failed";
            }
            else yInfo() << "pouring motion failed";
        }
        else yInfo() << "pouring approach failed";

        return false;
    }

    /****************************************************************/
    bool drop()
    {
        if (reach(latch_pose,latch_part))
        {
            if (goHome(latch_part+"_hand"))
            {
                Vector x=processApproach(latch_pose,latch_approach);
                if (reach(x,latch_part))
                {
                    string part_=choosePart(latch_part,latch_pose);
                    x[0]=0.5;
                    x[1]=(part_=="left"?0.3:-0.3);
                    x[2]+=grasping.lift;
                    x[3]=1.0;
                    x[4]=x[5]=0.0;
                    x[6]=-M_PI/2.0;

                    if (reach(x,latch_part))
                    {
                        if (goHome(latch_part))
                        {
                            return goHome("gaze");
                        }
                    }
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

        while (!exiting && !interrupted && (port.getOutputCount()>0))
        {
            Time::delay(std::min(10.0*period,1.0));

            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("get"));
            cmd.addString("done");
            if (port.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==ack)
                {
                    if (rep.get(1).asInt()>0)
                    {
                        yInfo()<<part<<"movements complete";
                        return true;
                    }
                }
                else
                    break;
            }
            else
                break;
        }
        return false;
    }

    /****************************************************************/
    bool ransac(const vector<double> &data, vector<bool> &consensus_set,
                double &result)
    {
        for (size_t i=0; i<data.size(); i++)
        {
            consensus_set.assign(data.size(),false);
            double val=0.0;
            size_t n=0;
            for (size_t j=0; j<data.size(); j++)
            {
                if (abs(data[j]-data[i])<=table.ransac_threshold)
                {
                    val+=data[j];
                    consensus_set[j]=true;
                    n++;
                }
            }
            val/=n;

            if (n>data.size()/2)
            {
                result=val;
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    bool calibTable()
    {
        if (goHome("all"))
        {
            ImageOf<PixelRgb> *img=nullptr;
            if (imgInPort.getInputCount()>0)
            {
                img=imgInPort.read(false);
            }

            vector<vector<int>> pixels;
            vector<int> pixel(2);

            Bottle cmd,rep;
            cmd.addString("Points");
            for (size_t i=0; i<table.num_pixels; i++)
            {
                pixel[0]=(int)Rand::scalar(table.pixels_bounds[0],table.pixels_bounds[2]);
                pixel[1]=(int)Rand::scalar(table.pixels_bounds[1],table.pixels_bounds[3]);
                pixels.push_back(pixel);

                cmd.addInt(pixel[0]);
                cmd.addInt(pixel[1]);
            }

            if (depthPort.getOutputCount()>0)
            {
                if (depthPort.write(cmd,rep))
                {
                    vector<vector<int>> pixels_used;
                    vector<double> z;
                    Vector x(3);
                    for (size_t i=0; i<rep.size(); i+=3)
                    {
                        x[0]=rep.get(i).asDouble();
                        x[1]=rep.get(i+1).asDouble();
                        x[2]=rep.get(i+2).asDouble();
                        if (norm(x)>0)
                        {
                            pixels_used.push_back(pixels[i/3]);
                            z.push_back(x[2]);
                        }
                    }

                    vector<bool> consensus_set; double h;
                    bool ok=ransac(z,consensus_set,h);
                    if ((imgOutPort.getOutputCount()>0) && (img!=nullptr))
                    {
                        int cx=(table.pixels_bounds[2]+table.pixels_bounds[0])>>1;
                        int cy=(table.pixels_bounds[3]+table.pixels_bounds[1])>>1;
                        int w=(table.pixels_bounds[2]-table.pixels_bounds[0])>>1;
                        int h=(table.pixels_bounds[3]-table.pixels_bounds[1])>>1;
                        draw::addRectangleOutline(*img,PixelRgb(0,0,255),cx,cy,w,h);

                        for (size_t i=0; i<consensus_set.size(); i++)
                        {
                            if (ok && consensus_set[i])
                            {
                                draw::addCrossHair(*img,PixelRgb(0,255,0),pixels_used[i][0],pixels_used[i][1],2);
                            }
                            else
                            {
                                draw::addCircleOutline(*img,PixelRgb(255,0,0),pixels_used[i][0],pixels_used[i][1],2);
                            }
                        }
                        imgOutPort.prepare()=*img;
                        imgOutPort.writeStrict();
                    }
                    if (ok)
                    {
                        return setTableHeightOPC(h);
                    }
                }
            }
        }
        return false;
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        // default values
        robot="cer";
        period=0.1;
        ack=Vocab::encode("ack");
        nack=Vocab::encode("nack");
        exiting=false;
        interrupted=false;
        gaze_track=false;

        // retrieve values from config file
        Bottle &gGeneral=rf.findGroup("general");
        if (!gGeneral.isNull())
        {
            robot=gGeneral.check("robot",Value(robot)).asString();
            period=gGeneral.check("period",Value(period)).asDouble();
            speed_hand=gGeneral.check("speed_hand",Value(10.0)).asDouble();
        }

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
            grasping.lift=gGrasping.check("lift",Value(0.1)).asDouble();
        }

        Bottle &gPouring=rf.findGroup("pouring");
        if (!gPouring.isNull())
        {
            pouring.H_offset=gPouring.check("H_offset",Value(0.01)).asDouble();
            pouring.V_offset=gPouring.check("V_offset",Value(0.02)).asDouble();
            pouring.init_inclin=gPouring.check("init_inclin",Value(30)).asDouble();
            pouring.final_inclin=gPouring.check("final_inclin",Value(120)).asDouble();
        }

        Bottle &gTable=rf.findGroup("table");
        if (!gTable.isNull())
        {
            table.default_height=gTable.check("default_height",Value(0.7)).asDouble();
            table.num_pixels=gTable.check("num_pixels",Value(100)).asInt();
            getVectorInfo(gTable,"pixels_bounds",table.pixels_bounds);
            table.ransac_threshold=gTable.check("ransac_threshold",Value(0.01)).asDouble();
        }

        cmdPort.open("/action-gateway/cmd:io");
        attach(cmdPort);

        opcPort.open("/action-gateway/opc/rpc");
        depthPort.open("/action-gateway/depth/rpc");
        gazePort.open("/action-gateway/gaze/rpc");
        reachLPort.open("/action-gateway/reach/left/rpc");
        reachRPort.open("/action-gateway/reach/right/rpc");
        mobReachLPort.open("/action-gateway/mobile-reach/left/rpc");
        mobReachRPort.open("/action-gateway/mobile-reach/right/rpc");
        stopMotorsPort.open("/action-gateway/motor_stop:rpc");
        stopMotorsPort.setReplier(stopMotorsProcessor);
        
        imgInPort.open("/action-gateway/img:i");
        imgOutPort.open("/action-gateway/img:o");

        Rand::init();
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
        if (gaze_track)
        {
            look(latch_pose.subVector(0,2));
        }
        return true;
    }

    /****************************************************************/
    bool respond(const Bottle &command, Bottle &reply) override
    {
        bool ok=false;
        Bottle payLoad;

        int cmd=command.get(0).asVocab();
        if (cmd==Vocab::encode("home") && !interrupted)
        {
            string part="all";
            if (command.size()>=2)
            {
                part=command.get(1).asString();
            }
            ok=goHome(part);
        }
        else if ((cmd==Vocab::encode("look")) && (command.size()>=2) && !interrupted)
        {
            Value &target=command.get(1);
            if (target.isList())
            {
                if (Bottle *b=target.asList())
                {
                    if (b->size()>=3)
                    {
                        string type=b->get(0).asString();
                        if ((type=="cartesian") && (b->size()>=4))
                        {
                            Vector x(3);
                            x[0]=b->get(1).asDouble();
                            x[1]=b->get(2).asDouble();
                            x[2]=b->get(3).asDouble();
                            ok=look(x);
                        }
                        else
                        {
                            int u,v;
                            u=b->get(1).asInt();
                            v=b->get(2).asInt();
                            ok=look(u,v);
                        }
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
        else if ((cmd==Vocab::encode("grasp")) && (command.size()>=3) && !interrupted)
        {
            Vector pose,approach;
            string part="select";

            if (Bottle *b1=command.get(1).asList())
            {
                for (size_t i=1; i<b1->size(); i++)
                {
                    pose.push_back(b1->get(i).asDouble());
                }
            }
            if (Bottle *b1=command.get(2).asList())
            {
                if (b1->size()>=2)
                {
                    if (Bottle *b2=b1->get(1).asList())
                    {
                        for (size_t i=0; i<b2->size(); i++)
                        {
                            approach.push_back(b2->get(i).asDouble());
                        }
                    }
                }
            }
            if (command.size()>=4)
            {
                part=command.get(3).asString();
            }

            latch_pose=pose;
            latch_approach=approach;
            latch_part=part;

            gaze_track=true;
            ok=grasp(pose,approach,part);
            gaze_track=false;
        }
        else if ((cmd==Vocab::encode("pour")) && (command.size()>=3) && !interrupted)
        {
            /* command format: pour ("source" x y z) ("destination" x y z)
             * source = where the liquid comes from expressed in the grabber frame (hand frame)
             *          for example, the neck of the currently grasped bottle
             * destination = where the liquid should be poured expressed in the reference frame
             *               for example, the top of a glass
             */

            Vector source, destination;

            if (Bottle *b1=command.get(1).asList())
            {
                for (size_t i=1; i<b1->size(); i++)
                {
                    source.push_back(b1->get(i).asDouble());
                }
            }
            if (Bottle *b2=command.get(2).asList())
            {
                for (size_t i=1; i<b2->size(); i++)
                {
                    destination.push_back(b2->get(i).asDouble());
                }
            }

            look(destination);

            ok=pour(source, destination);
        }
        else if (cmd==Vocab::encode("drop") && !interrupted)
        {
            look(latch_pose.subVector(0,2));
            ok=drop();
        }
        else if ((cmd==Vocab::encode("ask")) && (command.size()>=2))
        {
            Vector pose;
            if (Bottle *b1=command.get(1).asList())
            {
                for (size_t i=0; i<b1->size(); i++)
                {
                    pose.push_back(b1->get(i).asDouble());
                }
            }

            string part="select";
            if (command.size()>=3)
            {
                part=command.get(2).asString();
            }

            ok=ask(payLoad,pose,part);
        }
        else if ((cmd==Vocab::encode("askMobile")) && (command.size()>=3))
        {
            Vector pose;
            if (Bottle *b1=command.get(1).asList())
            {
                for (size_t i=0; i<b1->size(); i++)
                {
                    pose.push_back(b1->get(i).asDouble());
                }
            }

            Vector approach;
            if (Bottle *b1=command.get(2).asList())
            {
                for (size_t i=0; i<b1->size(); i++)
                {
                    approach.push_back(b1->get(i).asDouble());
                }
            }

            string part="select";
            part=command.get(3).asString();

            Vector noise(2,0.0);
            if (command.size()>=4)
            {
                if (Bottle *b1=command.get(3).asList())
                {
                    if (b1->size() == 2)
                    {
                        noise[0] = b1->get(0).asDouble();
                        noise[1] = b1->get(1).asDouble();
                    }
                }
            }

            ok=askMobileGrasp(payLoad,pose,approach,part,noise);
        }
        else if (cmd==Vocab::encode("calibrate"))
        {
            if (command.size()>=2)
            {
                if (command.get(1).asVocab()==Vocab::encode("table"))
                {
                    ok=calibTable();
                }
            }
        }
        else if (cmd==Vocab::encode("get"))
        {
            if (command.size()>=2)
            {
                if (command.get(1).asVocab()==Vocab::encode("table"))
                {
                    double h=table.default_height;
                    if (!getTableHeightOPC(h))
                    {
                        setTableHeightOPC(h);
                    }
                    Bottle &table_reply=reply.addList();
                    table_reply.addString("table_height");
                    table_reply.addDouble(h);
                    return true;
                }
            }
        }

        reply.addVocab(ok?ack:nack);
        if (ok && (payLoad.size()>0))
        {
            reply.append(payLoad);
        }
        return true;
    }

    /****************************************************************/
    bool interruptModule() override
    {
        exiting=true;
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
        if (depthPort.asPort().isOpen())
        {
            depthPort.close();
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
        if (mobReachLPort.asPort().isOpen())
        {
            mobReachLPort.close();
        }
        if (mobReachRPort.asPort().isOpen())
        {
            mobReachRPort.close();
        }
        if (!stopMotorsPort.isClosed())
        {
            stopMotorsPort.close();
        }
        if (!imgInPort.isClosed())
        {
            imgInPort.close();
        }
        if (!imgOutPort.isClosed())
        {
            imgOutPort.close();
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

public:
    Gateway() : stopMotorsProcessor(this)
    {
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

