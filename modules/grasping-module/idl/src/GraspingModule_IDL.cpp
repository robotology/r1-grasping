/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <src/GraspingModule_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class GraspingModule_IDL_serviceGraspObject : public yarp::os::Portable {
public:
  std::string objectName;
  bool _return;
  void init(const std::string& objectName);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class GraspingModule_IDL_serviceGraspObjectAtPosition : public yarp::os::Portable {
public:
  double x;
  double y;
  double z;
  bool _return;
  void init(const double x, const double y, const double z);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class GraspingModule_IDL_start : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class GraspingModule_IDL_halt : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

bool GraspingModule_IDL_serviceGraspObject::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("serviceGraspObject",1,1)) return false;
  if (!writer.writeString(objectName)) return false;
  return true;
}

bool GraspingModule_IDL_serviceGraspObject::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void GraspingModule_IDL_serviceGraspObject::init(const std::string& objectName) {
  _return = false;
  this->objectName = objectName;
}

bool GraspingModule_IDL_serviceGraspObjectAtPosition::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("serviceGraspObjectAtPosition",1,1)) return false;
  if (!writer.writeFloat64(x)) return false;
  if (!writer.writeFloat64(y)) return false;
  if (!writer.writeFloat64(z)) return false;
  return true;
}

bool GraspingModule_IDL_serviceGraspObjectAtPosition::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void GraspingModule_IDL_serviceGraspObjectAtPosition::init(const double x, const double y, const double z) {
  _return = false;
  this->x = x;
  this->y = y;
  this->z = z;
}

bool GraspingModule_IDL_start::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("start",1,1)) return false;
  return true;
}

bool GraspingModule_IDL_start::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void GraspingModule_IDL_start::init() {
  _return = false;
}

bool GraspingModule_IDL_halt::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("halt",1,1)) return false;
  return true;
}

bool GraspingModule_IDL_halt::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void GraspingModule_IDL_halt::init() {
  _return = false;
}

GraspingModule_IDL::GraspingModule_IDL() {
  yarp().setOwner(*this);
}
bool GraspingModule_IDL::serviceGraspObject(const std::string& objectName) {
  bool _return = false;
  GraspingModule_IDL_serviceGraspObject helper;
  helper.init(objectName);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool GraspingModule_IDL::serviceGraspObject(const std::string& objectName)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool GraspingModule_IDL::serviceGraspObjectAtPosition(const double x, const double y, const double z) {
  bool _return = false;
  GraspingModule_IDL_serviceGraspObjectAtPosition helper;
  helper.init(x,y,z);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool GraspingModule_IDL::serviceGraspObjectAtPosition(const double x, const double y, const double z)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool GraspingModule_IDL::start() {
  bool _return = false;
  GraspingModule_IDL_start helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool GraspingModule_IDL::start()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool GraspingModule_IDL::halt() {
  bool _return = false;
  GraspingModule_IDL_halt helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool GraspingModule_IDL::halt()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool GraspingModule_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  std::string tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "serviceGraspObject") {
      std::string objectName;
      if (!reader.readString(objectName)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = serviceGraspObject(objectName);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "serviceGraspObjectAtPosition") {
      double x;
      double y;
      double z;
      if (!reader.readFloat64(x)) {
        reader.fail();
        return false;
      }
      if (!reader.readFloat64(y)) {
        reader.fail();
        return false;
      }
      if (!reader.readFloat64(z)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = serviceGraspObjectAtPosition(x,y,z);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "start") {
      bool _return;
      _return = start();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "halt") {
      bool _return;
      _return = halt();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT32, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    std::string next_tag = reader.readTag();
    if (next_tag=="") break;
    tag.append("_").append(next_tag);
  }
  return false;
}

std::vector<std::string> GraspingModule_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.emplace_back("*** Available commands:");
    helpString.emplace_back("serviceGraspObject");
    helpString.emplace_back("serviceGraspObjectAtPosition");
    helpString.emplace_back("start");
    helpString.emplace_back("halt");
    helpString.emplace_back("help");
  }
  else {
    if (functionName=="serviceGraspObject") {
      helpString.emplace_back("bool serviceGraspObject(const std::string& objectName) ");
    }
    if (functionName=="serviceGraspObjectAtPosition") {
      helpString.emplace_back("bool serviceGraspObjectAtPosition(const double x, const double y, const double z) ");
    }
    if (functionName=="start") {
      helpString.emplace_back("bool start() ");
    }
    if (functionName=="halt") {
      helpString.emplace_back("bool halt() ");
    }
    if (functionName=="help") {
      helpString.emplace_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.emplace_back("Return list of available commands, or help message for a specific function");
      helpString.emplace_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.emplace_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.emplace_back("Command not found");
  return helpString;
}


