#ifndef PYDART2_JOINT_API_H
#define PYDART2_JOINT_API_H

////////////////////////////////////////////////////////////////////////////////
// Joint
#define JOINT(funcname) joint__##funcname
#define GET_JOINT(wid, skid, jid) Manager::skeleton(wid, skid)->getJoint(jid)

////////////////////////////////////////
// Joint::Property Functions
const char* JOINT(getName)(int wid, int skid, int jid);
const char* JOINT(setName)(int wid, int skid, int jid, const char* _name, bool _renameDofs);
bool JOINT(isKinematic)(int wid, int skid, int jid);
bool JOINT(isDynamic)(int wid, int skid, int jid);
const char* JOINT(getType)(int wid, int skid, int jid);

void JOINT(setActuatorType)(int wid, int skid, int jid, int actuator_type);
int JOINT(getActuatorType)(int wid, int skid, int jid);

////////////////////////////////////////
// Joint::Parent and child bodynode Functions
int JOINT(getParentBodyNode)(int wid, int skid, int jid);
int JOINT(getChildBodyNode)(int wid, int skid, int jid);
void JOINT(setTransformFromParentBodyNode)(int wid, int skid, int jid, double inv44[4][4]);
void JOINT(setTransformFromChildBodyNode)(int wid, int skid, int jid, double inv44[4][4]);
void JOINT(getTransformFromParentBodyNode)(int wid, int skid, int jid, double outv44[4][4]);
void JOINT(getTransformFromChildBodyNode)(int wid, int skid, int jid, double outv44[4][4]);

////////////////////////////////////////
// Joint::Limit Functions
void JOINT(setPositionLimitEnforced)(int wid, int skid, int jid, bool _isPositionLimitEnforced);
bool JOINT(isPositionLimitEnforced)(int wid, int skid, int jid);
bool JOINT(hasPositionLimit)(int wid, int skid, int jid, int _index);
double JOINT(getPositionLowerLimit)(int wid, int skid, int jid, int _index);
void JOINT(setPositionLowerLimit)(int wid, int skid, int jid, int _index, double _position);
double JOINT(getPositionUpperLimit)(int wid, int skid, int jid, int _index);
void JOINT(setPositionUpperLimit)(int wid, int skid, int jid, int _index, double _position);

////////////////////////////////////////
// Joint::Dof Functions
int JOINT(getDof)(int wid, int skid, int jid, int _index);
int JOINT(getNumDofs)(int wid, int skid, int jid);

////////////////////////////////////////
// Joint::Passive Force Functions
double JOINT(getSpringStiffness)(int wid, int skid, int jid, int _index);
void JOINT(setSpringStiffness)(int wid, int skid, int jid, int _index, double _k);
double JOINT(getRestPosition)(int wid, int skid, int jid, int _index);
void JOINT(setRestPosition)(int wid, int skid, int jid, int _index, double _q0);
double JOINT(getDampingCoefficient)(int wid, int skid, int jid, int _index);
void JOINT(setDampingCoefficient)(int wid, int skid, int jid, int _index, double _coeff);
double JOINT(getCoulombFriction)(int wid, int skid, int jid, int _index);
void JOINT(setCoulombFriction)(int wid, int skid, int jid, int _index, double _friction);


////////////////////////////////////////
// Joint::REVOLUTE_JOINT Functions
#define REVOLUTE_JOINT(funcname) revolute_joint__##funcname
#define GET_REVOLUTE_JOINT(wid, skid, jid) dynamic_cast<dart::dynamics::RevoluteJoint*>(Manager::skeleton(wid, skid)->getJoint(jid));
void REVOLUTE_JOINT(getAxis)(int wid, int skid, int jid, double outv3[3]);
void REVOLUTE_JOINT(setAxis)(int wid, int skid, int jid, double inv3[3]);


////////////////////////////////////////
// PrismaticJoint Functions
#define PRISMATIC_JOINT(funcname) prismatic_joint__##funcname
#define GET_PRISMATIC_JOINT(wid, skid, jid) dynamic_cast<dart::dynamics::PrismaticJoint*>(Manager::skeleton(wid, skid)->getJoint(jid));
void PRISMATIC_JOINT(getAxis)(int wid, int skid, int jid, double outv3[3]);
void PRISMATIC_JOINT(setAxis)(int wid, int skid, int jid, double inv3[3]);


////////////////////////////////////////
// UniversalJoint Functions
#define UNIVERSAL_JOINT(funcname) universal_joint__##funcname
#define GET_UNIVERSAL_JOINT(wid, skid, jid) dynamic_cast<dart::dynamics::UniversalJoint*>(Manager::skeleton(wid, skid)->getJoint(jid));
void UNIVERSAL_JOINT(getAxis1)(int wid, int skid, int jid, double outv3[3]);
void UNIVERSAL_JOINT(setAxis1)(int wid, int skid, int jid, double inv3[3]);
void UNIVERSAL_JOINT(getAxis2)(int wid, int skid, int jid, double outv3[3]);
void UNIVERSAL_JOINT(setAxis2)(int wid, int skid, int jid, double inv3[3]);


////////////////////////////////////////
// EulerJoint Functions
#define EULER_JOINT(funcname) euler_joint__##funcname
#define GET_EULER_JOINT(wid, skid, jid) dynamic_cast<dart::dynamics::EulerJoint*>(Manager::skeleton(wid, skid)->getJoint(jid));
const char* EULER_JOINT(getAxisOrder)(int wid, int skid, int jid);
void EULER_JOINT(setAxisOrder)(int wid, int skid, int jid, const char* axisorder);


#endif // #ifndef PYDART2_JOINT_API_H
