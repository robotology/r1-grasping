/******************************************************************************
*                                                                            *
* Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
* All Rights Reserved.                                                       *
*                                                                            *
******************************************************************************/
/**
* @file walk_example_module.cpp
* @authors: Michele Colledanchise <michele.colledanchise@iit.it>
*/
//standard imports
#include <iostream>           // for std::cout
#include <chrono>             // for seconds
#include <thread>             //for this_thread::sleep_for
//YARP imports
#include <yarp/os/Network.h>  // for yarp::os::Network
#include <yarp/os/RFModule.h> // for yarp::os::ResourceFinder
#include <yarp/os/LogStream.h> // for yError()

#include <math.h>
#include<yarp/os/RpcClient.h>


#include <random>


using namespace yarp::os;
using namespace std;


class EnvironmentDummy : public RFModule
{
public:
    Port inv_pose_port; // a port to handle messages
    int count;
    string inv_pose = "";
    bool is_at_inv_pose = false;
public:
    double getPeriod()
    {
        // module periodicity (seconds), called implicitly by the module.
        return 1.0;
    }
    // This is our main function. Will be called periodically every getPeriod() seconds
    bool updateModule()
    {
        count++;
        //cout << "[" << count << "]" << " updateModule..." << endl;
        return true;
    }
    // Message handler. Just echo all received messages.
    bool respond(const Bottle& command, Bottle& reply)
    {
        if (command.get(0).asString() == "setInvPose")
        {
            inv_pose = command.get(1).asString();
            reply.addString(inv_pose);

            cout << "setting inv pose to " << inv_pose << endl;
        }
        else if (command.get(0).asString() == "getInvPose")
        {
            cout << "getting inv pose" << inv_pose << endl;

            reply.addString(inv_pose);

            cout << "reply" << reply.toString() << endl;

        }
        else if (command.get(0).asString() == "isAtInvPose")
        {
            cout << "getting inv pose" << inv_pose << endl;

            reply.addInt(is_at_inv_pose);

            cout << "reply" << reply.toString() << endl;

        }
        else if (command.get(0).asString() == "setIsAtInvPose")
        {
            cout << "getting inv pose" << inv_pose << endl;

            inv_pose = command.get(1).asString();
            reply.addString(inv_pose);
            cout << "reply" << reply.toString() << endl;

        }
        else{
            cout << "command not recognized: " << command.toString() << endl;
            reply.addString("error");

            return false;
        }
        return true;
    }
    // Configure function. Receive a previously initialized
    // resource finder object. Use it to configure your module.
    // If you are migrating from the old module, this is the function
    // equivalent to the "open" method.
    bool configure(yarp::os::ResourceFinder &rf)
       {
           count=0;
           // optional, attach a port to the module
           // so that messages received from the port are redirected
           // to the respond method
           inv_pose_port.open("/inv_pose_request:i");
           attach(inv_pose_port);
           return true;
       }
       // Interrupt function.
       bool interruptModule()
       {
           cout << "Interrupting your module, for port cleanup" << endl;
           return true;
       }
       // Close function, to perform cleanup.
       bool close()
       {
           // optional, close port explicitly
           cout << "Calling close function\n";
           inv_pose_port.close();
           return true;
       }

};
int main(int argc, char * argv[])
{
	/* prepare and configure the resource finder */
	yarp::os::ResourceFinder rf;

	rf.configure(argc, argv);
	// print available command-line options
	// upon specifying --help; refer to xml
	// descriptor for the relative documentation
	if (rf.check("help"))
	{
        std::cout << " commands to sent to the port /inv_pose_request:i are: setInvPos and getInvPose " << std::endl;
		std::cout << std::endl;
		return EXIT_SUCCESS;
	}
	/* initialize yarp network */
	yarp::os::Network yarp;
	if (!yarp::os::Network::checkNetwork(5.0))
	{
		yError() << " YARP server not available!";
		return EXIT_FAILURE;
	}
	/* create your module */
    EnvironmentDummy module;
	rf.configure(argc, argv);
	module.runModule(rf);
	return 0;
}
