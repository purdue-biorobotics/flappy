#include <iostream>
#include <string>
#include <vector>
#include <map>
using std::cout;
using std::cerr;
using std::endl;

// Boost headers
#include <boost/algorithm/string.hpp>

#include "pydart2_manager.h"
#include "pydart2_api.h"
#include "pydart2_joint_api.h"
#include "pydart2_draw.h"

using namespace pydart;




////////////////////////////////////////////////////////////////////////////////
// Joint

////////////////////////////////////////
// Joint::Property Functions
const char* JOINT(getName)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getName().c_str();
}


const char* JOINT(setName)(int wid, int skid, int jid, const char* _name, bool _renameDofs) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->setName(_name, _renameDofs).c_str();
}


bool JOINT(isKinematic)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->isKinematic();
}


bool JOINT(isDynamic)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->isDynamic();
}


const char* JOINT(getType)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getType().c_str();
}


void JOINT(setActuatorType)(int wid, int skid, int jid, int actuator_type) {
  dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
  joint->setActuatorType(static_cast<dart::dynamics::Joint::ActuatorType>(actuator_type));
}


int JOINT(getActuatorType)(int wid, int skid, int jid) {
  dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
  return (int)joint->getActuatorType();
}

////////////////////////////////////////
// Joint::Parent and child Functions
int JOINT(getParentBodyNode)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    if (joint->getParentBodyNode()) {
        return joint->getParentBodyNode()->getIndexInSkeleton();
    } else {
        return -1;
    }
}


int JOINT(getChildBodyNode)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getChildBodyNode()->getIndexInSkeleton();
}


void JOINT(setTransformFromParentBodyNode)(int wid, int skid, int jid, double inv44[4][4]) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setTransformFromParentBodyNode(read_isometry(inv44));
}


void JOINT(setTransformFromChildBodyNode)(int wid, int skid, int jid, double inv44[4][4]) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setTransformFromChildBodyNode(read_isometry(inv44));
}


void JOINT(getTransformFromParentBodyNode)(int wid, int skid, int jid, double outv44[4][4]) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    write_isometry(joint->getTransformFromParentBodyNode(), outv44);
}


void JOINT(getTransformFromChildBodyNode)(int wid, int skid, int jid, double outv44[4][4]) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    write_isometry(joint->getTransformFromChildBodyNode(), outv44);
}

////////////////////////////////////////
// Joint::Limit Functions
void JOINT(setPositionLimitEnforced)(int wid, int skid, int jid, bool _isPositionLimitEnforced) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setPositionLimitEnforced(_isPositionLimitEnforced);
}


bool JOINT(isPositionLimitEnforced)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->isPositionLimitEnforced();
}


bool JOINT(hasPositionLimit)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->hasPositionLimit(_index);
}


double JOINT(getPositionLowerLimit)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getPositionLowerLimit(_index);
}


void JOINT(setPositionLowerLimit)(int wid, int skid, int jid, int _index, double _position) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setPositionLowerLimit(_index, _position);
}


double JOINT(getPositionUpperLimit)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getPositionUpperLimit(_index);
}


void JOINT(setPositionUpperLimit)(int wid, int skid, int jid, int _index, double _position) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setPositionUpperLimit(_index, _position);
}

////////////////////////////////////////
// Joint::Dof Functions
int JOINT(getDof)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getDof(_index)->getIndexInSkeleton();
}


int JOINT(getNumDofs)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getNumDofs();
}

////////////////////////////////////////
// Joint::Passive Force Functions
double JOINT(getSpringStiffness)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getSpringStiffness(_index);
}


void JOINT(setSpringStiffness)(int wid, int skid, int jid, int _index, double _k) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setSpringStiffness(_index, _k);
}


double JOINT(getRestPosition)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getRestPosition(_index);
}


void JOINT(setRestPosition)(int wid, int skid, int jid, int _index, double _q0) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setRestPosition(_index, _q0);
}


double JOINT(getDampingCoefficient)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getDampingCoefficient(_index);
}


void JOINT(setDampingCoefficient)(int wid, int skid, int jid, int _index, double _coeff) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setDampingCoefficient(_index, _coeff);
}


double JOINT(getCoulombFriction)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getCoulombFriction(_index);
}


void JOINT(setCoulombFriction)(int wid, int skid, int jid, int _index, double _friction) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setCoulombFriction(_index, _friction);
}


////////////////////////////////////////
// Joint::REVOLUTE_JOINT Functions
void REVOLUTE_JOINT(getAxis)(int wid, int skid, int jid, double outv3[3]) {
    dart::dynamics::RevoluteJoint* joint = GET_REVOLUTE_JOINT(wid, skid, jid);
    if (joint == NULL) {
      ERR << " [pydart2_api] joint is not RevoluteJoint\n";
      return;
    }
    write(joint->getAxis(), outv3);
}


void REVOLUTE_JOINT(setAxis)(int wid, int skid, int jid, double inv3[3]) {
  dart::dynamics::RevoluteJoint* joint = GET_REVOLUTE_JOINT(wid, skid, jid);
  if (joint == NULL) {
    ERR << " [pydart2_api] joint is not RevoluteJoint\n";
    return;
  }
  joint->setAxis(read(inv3, 3));
}


////////////////////////////////////////
// Joint::PRISMATIC_JOINT Functions
void PRISMATIC_JOINT(getAxis)(int wid, int skid, int jid, double outv3[3]) {
    dart::dynamics::PrismaticJoint* joint = GET_PRISMATIC_JOINT(wid, skid, jid);
    if (joint == NULL) {
      ERR << " [pydart2_api] joint is not PrismaticJoint\n";
      return;
    }
    write(joint->getAxis(), outv3);
}


void PRISMATIC_JOINT(setAxis)(int wid, int skid, int jid, double inv3[3]) {
  dart::dynamics::PrismaticJoint* joint = GET_PRISMATIC_JOINT(wid, skid, jid);
  if (joint == NULL) {
    ERR << " [pydart2_api] joint is not PrismaticJoint\n";
    return;
  }
  joint->setAxis(read(inv3, 3));
}


////////////////////////////////////////
// Joint::UNIVERSAL_JOINT Functions
void UNIVERSAL_JOINT(getAxis1)(int wid, int skid, int jid, double outv3[3]) {
    dart::dynamics::UniversalJoint* joint = GET_UNIVERSAL_JOINT(wid, skid, jid);
    if (joint == NULL) {
      ERR << " [pydart2_api] joint is not UniversalJoint\n";
      return;
    }
    write(joint->getAxis1(), outv3);
}


void UNIVERSAL_JOINT(setAxis1)(int wid, int skid, int jid, double inv3[3]) {
  dart::dynamics::UniversalJoint* joint = GET_UNIVERSAL_JOINT(wid, skid, jid);
  if (joint == NULL) {
    ERR << " [pydart2_api] joint is not UniversalJoint\n";
    return;
  }
  joint->setAxis1(read(inv3, 3));
}


void UNIVERSAL_JOINT(getAxis2)(int wid, int skid, int jid, double outv3[3]) {
    dart::dynamics::UniversalJoint* joint = GET_UNIVERSAL_JOINT(wid, skid, jid);
    if (joint == NULL) {
      ERR << " [pydart2_api] joint is not UniversalJoint\n";
      return;
    }
    write(joint->getAxis2(), outv3);
}


void UNIVERSAL_JOINT(setAxis2)(int wid, int skid, int jid, double inv3[3]) {
  dart::dynamics::UniversalJoint* joint = GET_UNIVERSAL_JOINT(wid, skid, jid);
  if (joint == NULL) {
    ERR << " [pydart2_api] joint is not UniversalJoint\n";
    return;
  }
  joint->setAxis2(read(inv3, 3));
}


////////////////////////////////////////
// Joint::EULER_JOINT Functions
const char* EULER_JOINT(getAxisOrder)(int wid, int skid, int jid) {
    dart::dynamics::EulerJoint* joint = GET_EULER_JOINT(wid, skid, jid);
    if (joint == NULL) {
      ERR << " [pydart2_api] joint is not EulerJoint\n";
      return "Wrong Joint Type";
    }
    auto ordering = joint->getAxisOrder();
    if (ordering == dart::dynamics::EulerJoint::AxisOrder::XYZ) {
        return "XYZ";
    }
    if (ordering == dart::dynamics::EulerJoint::AxisOrder::ZYX) {
        return "ZYX";
    }
    return "Invalid Order";
}


void EULER_JOINT(setAxisOrder)(int wid, int skid, int jid, const char* axisorder) {
  dart::dynamics::EulerJoint* joint = GET_EULER_JOINT(wid, skid, jid);
  if (joint == NULL) {
    ERR << " [pydart2_api] joint is not EulerJoint\n";
    return;
  }
  std::string ordering(axisorder);
  if (ordering == "XYZ") {
    joint->setAxisOrder(dart::dynamics::EulerJoint::AxisOrder::XYZ);
  } else if (ordering == "ZYX"){
    joint->setAxisOrder(dart::dynamics::EulerJoint::AxisOrder::ZYX);
  } else {
    ERR << " [pydart2_api] invalid EulerJoint AxisOrder" << ordering << "\n";
  }
}
