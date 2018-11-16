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
#include <string>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

/****************************************************************/
class Gateway : public RFModule
{
    BufferedPort<ImageOf<PixelFloat>> depthPort;
    BufferedPort<Property> gazePort;
    RpcServer rpcPort;
    RpcClient camPort;

    bool camera_configured;

    double fx,fy;
    bool camera_focalLength_configured;

    double ppx,ppy;
    bool camera_principalPoint_configured;

    double fov_x,fov_y;
    bool camera_fov_configured;

    Mutex depthMutex;
    ImageOf<PixelFloat> depth;
    bool camera_resolution_configured;

    Matrix Hcam;

    /****************************************************************/
    bool getCameraOptions()
    {
        if (camPort.getOutputCount()>0)
        {
            Bottle cmd,rep;   
            cmd.addVocab(Vocab::encode("visr"));
            cmd.addVocab(Vocab::encode("get"));
            cmd.addVocab(Vocab::encode("intp"));
            if (camPort.write(cmd,rep))
            {
                if (rep.size()>=4)
                {
                    const Bottle *list = rep.get(3).asList();

                    ppx = list->find("principalPointX").asDouble();
                    ppy = list->find("principalPointY").asDouble();
                    camera_principalPoint_configured=true;
                    yInfo()<<"camera principal point X (from sensor) ="<<ppx;
                    yInfo()<<"camera principal point Y (from sensor) ="<<ppy;

                    fx = list->find("focalLengthX").asDouble();
                    fy = list->find("focalLengthY").asDouble();
                    camera_focalLength_configured=true;
                    yInfo()<<"camera focal length X (from sensor) ="<<fx;
                    yInfo()<<"camera focal length Y (from sensor) ="<<fy;
                }
            }

            cmd.clear();
            cmd.addVocab(Vocab::encode("visr"));
            cmd.addVocab(Vocab::encode("get"));
            cmd.addVocab(Vocab::encode("res"));
            if (camPort.write(cmd,rep))
            {
                if (rep.size()>=5)
                {
                    depth.resize(rep.get(3).asDouble(), rep.get(4).asDouble());
                    camera_resolution_configured=true;
                    yInfo()<<"camera resolution X (from sensor) ="<<depth.width();
                    yInfo()<<"camera resolution Y (from sensor) ="<<depth.height();

                    if (!camera_principalPoint_configured)
                    {
                        ppx = 0.5*depth.width();
                        ppy = 0.5*depth.height();
                        camera_principalPoint_configured=true;
                        yInfo()<<"camera principal point X (from sensor res) ="<<ppx;
                        yInfo()<<"camera principal point Y (from sensor res) ="<<ppy;
                    }
                    if (camera_focalLength_configured)
                    {
                        fov_x = 2.0*(180.0/M_PI)*atan(0.5*depth.width()/fx);
                        fov_y = 2.0*(180.0/M_PI)*atan(0.5*depth.height()/fy);
                        camera_fov_configured=true;
                        yInfo()<<"camera horizontal field of view (from sensor res+focalLength) ="<<fov_x;
                        yInfo()<<"camera vertical field of view (from sensor res+focalLength) ="<<fov_y;
                    }
                }
            }

            if (!camera_fov_configured)
            {
                cmd.clear();
                cmd.addVocab(Vocab::encode("visr"));
                cmd.addVocab(Vocab::encode("get"));
                cmd.addVocab(Vocab::encode("fov"));
                if (camPort.write(cmd,rep))
                {
                    if (rep.size()>=5)
                    {
                        fov_x=rep.get(3).asDouble();
                        fov_y=rep.get(4).asDouble();
                        camera_fov_configured=true;
                        yInfo()<<"camera horizontal field of view (from sensor) ="<<fov_x;
                        yInfo()<<"camera vertical field of view (from sensor) ="<<fov_y;

                        if(camera_resolution_configured && !camera_focalLength_configured)
                        {
                            fx = depth.width()/(2.0*tan(fov_x*(M_PI/180.0)/2.0));
                            fy = depth.height()/(2.0*tan(fov_y*(M_PI/180.0)/2.0));
                            camera_focalLength_configured=true;
                            yInfo()<<"camera focal length X (from sensor fov) ="<<fx;
                            yInfo()<<"camera focal length Y (from sensor fov) ="<<fy;
                        }

                        if (!camera_resolution_configured && camera_focalLength_configured)
                        {
                            double width=fx*2.0*tan(fov_x*(M_PI/180.0)/2.0);
                            double height=fy*2.0*tan(fov_y*(M_PI/180.0)/2.0);
                            depth.resize(width,height);
                            yInfo()<<"camera res X (from sensor fov+focalLength) ="<<depth.width();
                            yInfo()<<"camera res Y (from sensor fov+focalLength) ="<<depth.height();
                        }
                    }
                }
            }
            return camera_principalPoint_configured && camera_focalLength_configured && camera_resolution_configured;
        }
        return false;
    }

    /****************************************************************/
    Vector getPoint3D(const int u, const int v) const
    {
        Vector p(3,0.0);
        if ((u>=0) && (u<depth.width()) && (v>=0) && (v<depth.height()))
        {
            double d=depth(u,v);
            if ((d>0.0) && (fx>0.0) && (fy>0.0))
            {
                double x=u-ppx;
                double y=v-ppy;

                p[0]=x*d/fx;
                p[1]=y*d/fy;
                p[2]=d;
            }
        }
        return p;
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        // default values

        camera_configured=false;

        ppx=0;
        ppy=0;
        camera_principalPoint_configured=false;

        fx=0;
        fy=0;
        camera_focalLength_configured=false;

        fov_x=0;
        fov_y=0;
        camera_fov_configured=false;

        camera_resolution_configured=false;

        Hcam=eye(4,4);

        // retrieve values from config file
        Bottle &gCamera=rf.findGroup("camera");
        if (!gCamera.isNull())
        {
            if (gCamera.check("principalPoint"))
            {
                if (Bottle *pp=gCamera.find("principalPoint").asList())
                {
                    if (pp->size()>=2)
                    {
                        ppx=pp->get(0).asDouble();
                        ppy=pp->get(1).asDouble();
                        camera_principalPoint_configured=true;
                        yInfo()<<"camera principal point X (from file) ="<<ppx;
                        yInfo()<<"camera principal point Y (from file) ="<<ppy;
                    }
                }
            }

            if (gCamera.check("focalLength"))
            {
                if (Bottle *fl=gCamera.find("focalLength").asList())
                {
                    if (fl->size()>=2)
                    {
                        fx=fl->get(0).asDouble();
                        fy=fl->get(1).asDouble();
                        camera_focalLength_configured=true;
                        yInfo()<<"camera focal length X (from file) ="<<fx;
                        yInfo()<<"camera focal length Y (from file) ="<<fy;
                    }
                }
            }

            if (gCamera.check("res"))
            {
                if (Bottle *res=gCamera.find("res").asList())
                {
                    if (res->size()>=2)
                    {
                        depth.resize(res->get(0).asDouble(), res->get(1).asDouble());
                        camera_resolution_configured=true;
                        yInfo()<<"camera resolution X (from file) ="<<depth.width();
                        yInfo()<<"camera resolution Y (from file) ="<<depth.height();

                        if (!camera_principalPoint_configured)
                        {
                            ppx = 0.5*depth.width();
                            ppy = 0.5*depth.height();
                            camera_principalPoint_configured=true;
                            yInfo()<<"camera principal point X (from file res) ="<<ppx;
                            yInfo()<<"camera principal point Y (from file res) ="<<ppy;
                        }
                        if (camera_focalLength_configured)
                        {
                            fov_x = 2.0*(180.0/M_PI)*atan(0.5*depth.width()/fx);
                            fov_y = 2.0*(180.0/M_PI)*atan(0.5*depth.height()/fy);
                            camera_fov_configured=true;
                            yInfo()<<"camera horizontal field of view (from file res+focalLength) ="<<fov_x;
                            yInfo()<<"camera vertical field of view (from file res+focalLength) ="<<fov_y;
                        }
                    }
                }
            }

            if (gCamera.check("fov"))
            {
                if (Bottle *fov=gCamera.find("fov").asList())
                {
                    if (fov->size()>=2)
                    {
                        if (camera_fov_configured)
                        {
                            yError()<<"Configuration file contains incompatible redundant parameters (focalLength+resolution+fov)";
                        }
                        else
                        {
                            fov_x=fov->get(0).asDouble();
                            fov_y=fov->get(1).asDouble();
                            camera_fov_configured=true;
                            yInfo()<<"camera horizontal field of view (from file) ="<<fov_x;
                            yInfo()<<"camera vertical field of view (from file) ="<<fov_y;

                            if(camera_resolution_configured && !camera_focalLength_configured)
                            {
                                fx = depth.width()/(2.0*tan(fov_x*(M_PI/180.0)/2.0));
                                fy = depth.height()/(2.0*tan(fov_y*(M_PI/180.0)/2.0));
                                camera_focalLength_configured=true;
                                yInfo()<<"camera focal length X (from file fov) ="<<fx;
                                yInfo()<<"camera focal length Y (from file fov) ="<<fy;
                            }

                            if (!camera_resolution_configured && camera_focalLength_configured)
                            {
                                double width=fx*2.0*tan(fov_x*(M_PI/180.0)/2.0);
                                double height=fy*2.0*tan(fov_y*(M_PI/180.0)/2.0);
                                depth.resize(width,height);
                                  yInfo()<<"camera res X (from file fov+focalLength) ="<<depth.width();
                                yInfo()<<"camera res Y (from file fov+focalLength) ="<<depth.height();
                            }
                        }
                    }
                }
            }

            camera_configured = camera_principalPoint_configured && camera_focalLength_configured && camera_resolution_configured;
        }

        depthPort.open("/vision3d-gateway/depth:i");
        gazePort.open("/vision3d-gateway/gaze/state:i");
        rpcPort.open("/vision3d-gateway/rpc");
        camPort.open("/vision3d-gateway/cam:rpc");

        attach(rpcPort);

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
        if (!camera_configured)
        {
            camera_configured=getCameraOptions();
        }

        if (ImageOf<PixelFloat> *ptr=depthPort.read(true))
        {
            if (camera_resolution_configured && ((ptr->width()!=depth.width()) || (ptr->height()!=depth.height())) )
            {
                yInfo()<<"Warning received depth image dimensions do not match configuration";
            }
            LockGuard lg(depthMutex);
            depth=*ptr;
            camera_resolution_configured=true;
        }
        if (Property *ptr=gazePort.read(false))
        {
            if (Bottle *pose=ptr->find("depth").asList())
            {
                if (pose->size()>=7)
                {
                    Vector orientation(4);
                    orientation[0]=pose->get(3).asDouble();
                    orientation[1]=pose->get(4).asDouble();
                    orientation[2]=pose->get(5).asDouble();
                    orientation[3]=pose->get(6).asDouble();
                    Hcam=axis2dcm(orientation);
                    Hcam(0,3)=pose->get(0).asDouble();
                    Hcam(1,3)=pose->get(1).asDouble();
                    Hcam(2,3)=pose->get(2).asDouble();
                }
            }
        }
        return true;
    }

    /****************************************************************/
    bool respond(const Bottle &command, Bottle &reply) override
    {
        reply.clear();

        if(command.size()==0)
            return false;

        string cmd=command.get(0).asString();

        if (cmd=="quit")
            return false;
        else if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands are:");
            reply.addString("- [Rect tlx tly w h step]: Given the pixels in the rectangle defined by {(tlx,tly) (tlx+w,tly+h)} (parsed by columns), the response contains the corresponding 3D points in the ROOT frame. The optional parameter step defines the sampling quantum; by default step=1.");
            reply.addString("- [Points u_1 v_1 ... u_n v_n]: Given a list of n pixels, the response contains the corresponding 3D points in the ROOT frame.");
            reply.addString("For more details on the commands, check the module's documentation");
            return true;
        }
        else if ((cmd=="Rect") && (command.size()>=5))
        {
            int tlx=command.get(1).asInt();
            int tly=command.get(2).asInt();
            int w=command.get(3).asInt();
            int h=command.get(4).asInt();
            int step=1;
            if (command.size()>=6)
            {
                step=command.get(5).asInt();
            }

            LockGuard lg(depthMutex);
            for (int u=tlx; u<tlx+w; u+=step)
            {
                for (int v=tly; v<tly+h; v+=step)
                {
                    Vector p=getPoint3D(u,v);
                    p.push_back(1.0);
                    p=Hcam*p;

                    reply.addDouble(p[0]);
                    reply.addDouble(p[1]);
                    reply.addDouble(p[2]);
                }
            }
        }
        else if ((cmd=="Points") && (command.size()>=3))
        {
            LockGuard lg(depthMutex);
            for (int cnt=1; cnt<command.size()-1; cnt+=2)
            {
                int u=command.get(cnt).asInt();
                int v=command.get(cnt+1).asInt();
                Vector p=getPoint3D(u,v);
                p.push_back(1.0);
                p=Hcam*p;

                reply.addDouble(p[0]);
                reply.addDouble(p[1]);
                reply.addDouble(p[2]);
            }
        }
        else
            reply.addString("NACK");
        return true;
    }

    /****************************************************************/
    bool interruptModule() override
    {
        depthPort.interrupt();
        gazePort.interrupt();
        rpcPort.interrupt();
        camPort.interrupt();
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        depthPort.close();
        gazePort.close();
        rpcPort.close();
        camPort.close();
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
    rf.setDefaultContext("vision3d-gateway");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    Gateway gateway;
    return gateway.runModule(rf);
}
