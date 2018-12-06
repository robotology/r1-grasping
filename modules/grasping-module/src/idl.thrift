#idl.thrift
service GraspingModule_IDL {
  bool serviceGraspObject(1:string objectName)
  bool serviceGraspObjectAtPosition(1:double x, 2:double y, 3:double z)
  bool start()
  bool halt()
}
