/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_GraspingModule_IDL
#define YARP_THRIFT_GENERATOR_GraspingModule_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class GraspingModule_IDL;


class GraspingModule_IDL : public yarp::os::Wire {
public:
  GraspingModule_IDL();
  virtual bool serviceGraspObject(const std::string& objectName);
  virtual bool serviceGraspObjectAtPosition(const double x, const double y, const double z);
  virtual bool start();
  virtual bool halt();
  virtual bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
