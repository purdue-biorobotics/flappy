#ifndef PYDART2_DOF_API_H
#define PYDART2_DOF_API_H


////////////////////////////////////////////////////////////////////////////////
// DegreeOfFreedom
#define DOF(funcname) dof__##funcname
#define GET_DOF(wid, skid, dofid) Manager::skeleton(wid, skid)->getDof(dofid)

const char* DOF(getName)(int wid, int skid, int dofid);

////////////////////////////////////////
// Dof::Index Functions
int DOF(getIndexInSkeleton)(int wid, int skid, int dofid);
int DOF(getIndexInTree)(int wid, int skid, int dofid);
int DOF(getIndexInJoint)(int wid, int skid, int dofid);
int DOF(getTreeIndex)(int wid, int skid, int dofid);

////////////////////////////////////////
// Dof::Position Functions
double DOF(getPosition)(int wid, int skid, int dofid);
void DOF(setPosition)(int wid, int skid, int dofid, double _position);
double DOF(getInitialPosition)(int wid, int skid, int dofid);
void DOF(setInitialPosition)(int wid, int skid, int dofid, double _initial);
bool DOF(hasPositionLimit)(int wid, int skid, int dofid);
double DOF(getPositionLowerLimit)(int wid, int skid, int dofid);
void DOF(setPositionLowerLimit)(int wid, int skid, int dofid, double _limit);
double DOF(getPositionUpperLimit)(int wid, int skid, int dofid);
void DOF(setPositionUpperLimit)(int wid, int skid, int dofid, double _limit);

////////////////////////////////////////
// Dof::Velocity Functions
double DOF(getVelocity)(int wid, int skid, int dofid);
void DOF(setVelocity)(int wid, int skid, int dofid, double _velocity);
double DOF(getInitialVelocity)(int wid, int skid, int dofid);
void DOF(setInitialVelocity)(int wid, int skid, int dofid, double _initial);
double DOF(getVelocityLowerLimit)(int wid, int skid, int dofid);
void DOF(setVelocityLowerLimit)(int wid, int skid, int dofid, double _limit);
double DOF(getVelocityUpperLimit)(int wid, int skid, int dofid);
void DOF(setVelocityUpperLimit)(int wid, int skid, int dofid, double _limit);

////////////////////////////////////////
// Dof::Passive Force Functions
double DOF(getSpringStiffness)(int wid, int skid, int dofid);
void DOF(setSpringStiffness)(int wid, int skid, int dofid, double _k);
double DOF(getRestPosition)(int wid, int skid, int dofid);
void DOF(setRestPosition)(int wid, int skid, int dofid, double _q0);
double DOF(getDampingCoefficient)(int wid, int skid, int dofid);
void DOF(setDampingCoefficient)(int wid, int skid, int dofid, double _coeff);
double DOF(getCoulombFriction)(int wid, int skid, int dofid);
void DOF(setCoulombFriction)(int wid, int skid, int dofid, double _friction);

////////////////////////////////////////
// Dof::Constraint Force Functions
double DOF(getConstraintImpulse)(int wid, int skid, int dofid);
void DOF(setConstraintImpulse)(int wid, int skid, int dofid, double _impulse);


#endif // #ifndef PYDART2_DOF_API_H
