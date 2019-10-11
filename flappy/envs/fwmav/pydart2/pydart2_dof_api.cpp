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
#include "pydart2_dof_api.h"
#include "pydart2_draw.h"

using namespace pydart;



////////////////////////////////////////////////////////////////////////////////
// DegreeOfFreedom
const char* DOF(getName)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getName().c_str();
}

////////////////////////////////////////
// Dof::Index Functions
int DOF(getIndexInSkeleton)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getIndexInSkeleton();
}


int DOF(getIndexInTree)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getIndexInTree();
}


int DOF(getIndexInJoint)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getIndexInJoint();
}


int DOF(getTreeIndex)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getTreeIndex();
}

////////////////////////////////////////
// Dof::Position Functions
double DOF(getPosition)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getPosition();
}


void DOF(setPosition)(int wid, int skid, int dofid, double _position) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setPosition(_position);
}


double DOF(getInitialPosition)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getInitialPosition();
}


void DOF(setInitialPosition)(int wid, int skid, int dofid, double _initial) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setInitialPosition(_initial);
}


bool DOF(hasPositionLimit)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->hasPositionLimit();
}


double DOF(getPositionLowerLimit)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getPositionLowerLimit();
}


void DOF(setPositionLowerLimit)(int wid, int skid, int dofid, double _limit) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setPositionLowerLimit(_limit);
}


double DOF(getPositionUpperLimit)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getPositionUpperLimit();
}


void DOF(setPositionUpperLimit)(int wid, int skid, int dofid, double _limit) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setPositionUpperLimit(_limit);
}

////////////////////////////////////////
// Dof::Velocity Functions
double DOF(getVelocity)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getVelocity();
}


void DOF(setVelocity)(int wid, int skid, int dofid, double _velocity) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setVelocity(_velocity);
}


double DOF(getInitialVelocity)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getInitialVelocity();
}


void DOF(setInitialVelocity)(int wid, int skid, int dofid, double _initial) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setInitialVelocity(_initial);
}


double DOF(getVelocityLowerLimit)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getVelocityLowerLimit();
}


void DOF(setVelocityLowerLimit)(int wid, int skid, int dofid, double _limit) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setVelocityLowerLimit(_limit);
}


double DOF(getVelocityUpperLimit)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getVelocityUpperLimit();
}


void DOF(setVelocityUpperLimit)(int wid, int skid, int dofid, double _limit) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setVelocityUpperLimit(_limit);
}

////////////////////////////////////////
// Dof::Passive Force Functions
double DOF(getSpringStiffness)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getSpringStiffness();
}


void DOF(setSpringStiffness)(int wid, int skid, int dofid, double _k) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setSpringStiffness(_k);
}


double DOF(getRestPosition)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getRestPosition();
}


void DOF(setRestPosition)(int wid, int skid, int dofid, double _q0) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setRestPosition(_q0);
}


double DOF(getDampingCoefficient)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getDampingCoefficient();
}


void DOF(setDampingCoefficient)(int wid, int skid, int dofid, double _coeff) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setDampingCoefficient(_coeff);
}


double DOF(getCoulombFriction)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getCoulombFriction();
}


void DOF(setCoulombFriction)(int wid, int skid, int dofid, double _friction) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setCoulombFriction(_friction);
}


////////////////////////////////////////
// Dof::Passive Force Functions
double DOF(getConstraintImpulse)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getConstraintImpulse();
}


void DOF(setConstraintImpulse)(int wid, int skid, int dofid, double _impulse) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->setConstraintImpulse(_impulse);
}
