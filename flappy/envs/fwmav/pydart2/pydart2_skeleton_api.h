#ifndef PYDART2_SKELETON_API_H
#define PYDART2_SKELETON_API_H


////////////////////////////////////////////////////////////////////////////////
// Skeleton
#define SKEL(funcname) skeleton__##funcname
#define GET_SKELETON(wid, skid) Manager::skeleton(wid, skid);

void SKEL(render)(int wid, int skid);
void SKEL(renderWithColor)(int wid, int skid, double inv4[4]);
const char* SKEL(getName)(int wid, int skid);
double SKEL(getMass)(int wid, int skid);

////////////////////////////////////////
// Skeleton::Property Functions
bool SKEL(isMobile)(int wid, int skid);
void SKEL(setMobile)(int wid, int skid, bool mobile);
bool SKEL(getSelfCollisionCheck)(int wid, int skid);
void SKEL(setSelfCollisionCheck)(int wid, int skid, int enable);
bool SKEL(getAdjacentBodyCheck)(int wid, int skid);
void SKEL(setAdjacentBodyCheck)(int wid, int skid, int enable);
void SKEL(setRootJointToTransAndEuler)(int wid, int skid);
void SKEL(setRootJointToWeld)(int wid, int skid);

////////////////////////////////////////
// Skeleton::Structure Information Functions
int SKEL(getNumBodyNodes)(int wid, int skid);
int SKEL(getNumJoints)(int wid, int skid);
int SKEL(getNumDofs)(int wid, int skid);
int SKEL(getNumMarkers)(int wid, int skid);

////////////////////////////////////////
// Skeleton::Pose Functions
void SKEL(getPositions)(int wid, int skid, double* outv, int ndofs);
void SKEL(setPositions)(int wid, int skid, double* inv, int ndofs);
void SKEL(getVelocities)(int wid, int skid, double* outv, int ndofs);
void SKEL(setVelocities)(int wid, int skid, double* inv, int ndofs);
void SKEL(getAccelerations)(int wid, int skid, double* outv, int ndofs);
void SKEL(setForces)(int wid, int skid, double* inv, int ndofs);

////////////////////////////////////////
// Skeleton::Difference Functions
void SKEL(getPositionDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs);
void SKEL(getVelocityDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs);

////////////////////////////////////////
// Skeleton::Limit Functions
void SKEL(getPositionLowerLimits)(int wid, int skid, double* outv, int ndofs);
void SKEL(getPositionUpperLimits)(int wid, int skid, double* outv, int ndofs);
void SKEL(getForceLowerLimits)(int wid, int skid, double* outv, int ndofs);
void SKEL(getForceUpperLimits)(int wid, int skid, double* outv, int ndofs);

////////////////////////////////////////
// Skeleton::Momentum Functions
void SKEL(getCOM)(int wid, int skid, double outv3[3]);
void SKEL(getCOMLinearVelocity)(int wid, int skid, double outv3[3]);
void SKEL(getCOMLinearAcceleration)(int wid, int skid, double outv3[3]);

////////////////////////////////////////
// Skeleton::Lagrangian Functions
void SKEL(getMassMatrix)(int wid, int skid, double* outm, int nrows, int ncols);
void SKEL(getCoriolisAndGravityForces)(int wid, int skid, double* outv, int ndofs);
void SKEL(getConstraintForces)(int wid, int skid, double* outv, int ndofs);


#endif // #ifndef PYDART2_SKELETON_API_H
