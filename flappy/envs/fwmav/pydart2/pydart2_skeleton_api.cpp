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
#include "pydart2_skeleton_api.h"
#include "pydart2_draw.h"

using namespace pydart;


////////////////////////////////////////////////////////////////////////////////
// Skeleton
void SKEL(render)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawSkeleton(ri, skel.get());
}

void SKEL(renderWithColor)(int wid, int skid, double inv4[4]) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    Eigen::Vector4d color(inv4);
    // MSG << "color = " << color.transpose() << "\n";
    drawSkeleton(ri, skel.get(), color, false);
}

const char* SKEL(getName)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getName().c_str();
}

double SKEL(getMass)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getMass();
}

////////////////////////////////////////
// Skeleton::Property Functions
bool SKEL(isMobile)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->isMobile();
}

void SKEL(setMobile)(int wid, int skid, bool mobile) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setMobile(mobile);
}

bool SKEL(getSelfCollisionCheck)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getSelfCollisionCheck();
}

void SKEL(setSelfCollisionCheck)(int wid, int skid, int enable) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setSelfCollisionCheck(enable);
}

bool SKEL(getAdjacentBodyCheck)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getAdjacentBodyCheck();
}

void SKEL(setAdjacentBodyCheck)(int wid, int skid, int enable) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setAdjacentBodyCheck(enable);
}

void SKEL(setRootJointToTransAndEuler)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    // change the joint type to euler
    dart::dynamics::BodyNode* oldRoot = skel->getRootBodyNode();
    oldRoot->changeParentJointType<dart::dynamics::EulerJoint>();
    oldRoot->getParentJoint()->setName("root_r");
    // create a new root
    std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> ret =
        skel->createJointAndBodyNodePair
        <dart::dynamics::TranslationalJoint, dart::dynamics::BodyNode>();
    dart::dynamics::Joint* newJoint = ret.first;
    newJoint->setName("root_t");
    dart::dynamics::BodyNode* newBody = ret.second;
    newBody->setMass(0.0);
    // rearrange the root joints
    oldRoot->moveTo(newBody);
}


void SKEL(setRootJointToWeld)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    // change the joint type to euler
    dart::dynamics::BodyNode* oldRoot = skel->getRootBodyNode();
    oldRoot->changeParentJointType<dart::dynamics::WeldJoint>();

    // oldRoot->getParentJoint()->setName("root_r");
    // // create a new root
    // std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> ret =
    //     skel->createJointAndBodyNodePair
    //     <dart::dynamics::TranslationalJoint, dart::dynamics::BodyNode>();
    // dart::dynamics::Joint* newJoint = ret.first;
    // newJoint->setName("root_t");
    // dart::dynamics::BodyNode* newBody = ret.second;
    // newBody->setMass(0.0);
    // // rearrange the root joints
    // oldRoot->moveTo(newBody);
}


////////////////////////////////////////
// Skeleton::Structure Information Functions
int SKEL(getNumBodyNodes)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumBodyNodes();
}

int SKEL(getNumJoints)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumJoints();
}

int SKEL(getNumDofs)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumDofs();
}

int SKEL(getNumMarkers)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumMarkers();
}

////////////////////////////////////////
// Skeleton::Pose Functions
void SKEL(getPositions)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getPositions(), outv);
}

void SKEL(setPositions)(int wid, int skid, double* inv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setPositions(read(inv, ndofs));
}

void SKEL(getVelocities)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getVelocities(), outv);
}

void SKEL(setVelocities)(int wid, int skid, double* inv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setVelocities(read(inv, ndofs));
}

void SKEL(getAccelerations)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getAccelerations(), outv);
}

void SKEL(setForces)(int wid, int skid, double* inv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setForces(read(inv, ndofs));
}

////////////////////////////////////////
// Skeleton::Difference Functions
void SKEL(getPositionDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::VectorXd dq1 = read(inv1, indofs1);
    Eigen::VectorXd dq2 = read(inv2, indofs2);
    Eigen::VectorXd dq_diff = skel->getVelocityDifferences(dq1, dq2);
    write(dq_diff, outv);
}

void SKEL(getVelocityDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::VectorXd q1 = read(inv1, indofs1);
    Eigen::VectorXd q2 = read(inv2, indofs2);
    Eigen::VectorXd q_diff = skel->getPositionDifferences(q1, q2);
    write(q_diff, outv);
}

////////////////////////////////////////
// Skeleton::Limit Functions
void SKEL(getPositionLowerLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getPositionLowerLimits(), outv);
}

void SKEL(getPositionUpperLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getPositionUpperLimits(), outv);
}

void SKEL(getForceLowerLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getForceLowerLimits(), outv);
}

void SKEL(getForceUpperLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getForceUpperLimits(), outv);
}

////////////////////////////////////////
// Skeleton::Momentum Functions
void SKEL(getCOM)(int wid, int skid, double outv3[3]) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getCOM(), outv3);
}

void SKEL(getCOMLinearVelocity)(int wid, int skid, double outv3[3]) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getCOMLinearVelocity(), outv3);
}

void SKEL(getCOMLinearAcceleration)(int wid, int skid, double outv3[3]) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getCOMLinearAcceleration(), outv3);
}

////////////////////////////////////////
// Skeleton::Lagrangian Functions
void SKEL(getMassMatrix)(int wid, int skid, double* outm, int nrows, int ncols) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write_matrix(skel->getMassMatrix(), outm);
}

void SKEL(getCoriolisAndGravityForces)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getCoriolisAndGravityForces(), outv);
}

void SKEL(getConstraintForces)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getConstraintForces(), outv);
}
