#ifndef PYDART2_BODYNODE_API_H
#define PYDART2_BODYNODE_API_H

////////////////////////////////////////////////////////////////////////////////
// BodyNode
#define BODY(funcname) bodynode__##funcname
#define GET_BODY(wid, skid, bid) Manager::skeleton(wid, skid)->getBodyNode(bid)

const char* BODY(getName)(int wid, int skid, int bid);

////////////////////////////////////////
// BodyNode::Structure Functions
int BODY(getParentBodyNode)(int wid, int skid, int bid);
int BODY(getNumChildBodyNodes)(int wid, int skid, int bid);
int BODY(getChildBodyNode)(int wid, int skid, int bid, int _index);

////////////////////////////////////////
// BodyNode::Joint and Dof Functions
int BODY(getParentJoint)(int wid, int skid, int bid);
int BODY(getNumChildJoints)(int wid, int skid, int bid);
int BODY(getChildJoint)(int wid, int skid, int bid, int _index);
int BODY(getNumDependentDofs)(int wid, int skid, int bid);
int BODY(getDependentDof)(int wid, int skid, int bid, int _index);

////////////////////////////////////////
// BodyNode::Shape
int BODY(getNumShapeNodes)(int wid, int skid, int bid);

////////////////////////////////////////
// BodyNode::Index Functions
int BODY(getIndexInSkeleton)(int wid, int skid, int bid);
int BODY(getIndexInTree)(int wid, int skid, int bid);
int BODY(getTreeIndex)(int wid, int skid, int bid);

////////////////////////////////////////
// BodyNode::Property Functions
void BODY(setGravityMode)(int wid, int skid, int bid, bool _gravityMode);
bool BODY(getGravityMode)(int wid, int skid, int bid);
bool BODY(isCollidable)(int wid, int skid, int bid);
void BODY(setCollidable)(int wid, int skid, int bid, bool _isCollidable);

////////////////////////////////////////
// BodyNode::Inertia Functions
double BODY(getMass)(int wid, int skid, int bid);
void BODY(setMass)(int wid, int skid, int bid, double mass);
void BODY(getInertia)(int wid, int skid, int bid, double outv33[3][3]);
void BODY(setInertia)(int wid, int skid, int bid, double inv33[3][3]);

////////////////////////////////////////
// BodyNode::Momentum Functions
void BODY(getLocalCOM)(int wid, int skid, int bid, double outv3[3]);
void BODY(getCOM)(int wid, int skid, int bid, double outv3[3]);
void BODY(getCOMLinearVelocity)(int wid, int skid, int bid, double outv3[3]);
void BODY(getCOMSpatialVelocity)(int wid, int skid, int bid, double outv6[6]);
void BODY(getCOMLinearAcceleration)(int wid, int skid, int bid, double outv3[3]);
void BODY(getCOMSpatialAcceleration)(int wid, int skid, int bid, double outv6[6]);

////////////////////////////////////////
// BodyNode::Friction and Restitution Functions
void BODY(setFrictionCoeff)(int wid, int skid, int bid, double _coeff);
double BODY(getFrictionCoeff)(int wid, int skid, int bid);
void BODY(setRestitutionCoeff)(int wid, int skid, int bid, double _coeff);
double BODY(getRestitutionCoeff)(int wid, int skid, int bid);

////////////////////////////////////////
// BodyNode::Transforms
void BODY(getTransform)(int wid, int skid, int bid, double outv44[4][4]);
void BODY(getWorldTransform)(int wid, int skid, int bid, double outv44[4][4]);
void BODY(getRelativeTransform)(int wid, int skid, int bid, double outv44[4][4]);

////////////////////////////////////////
// BodyNode::Ext Force and Torque
void BODY(addExtForce)(int wid, int skid, int bid, double inv3[3], double inv3_2[3], bool _isForceLocal, bool _isOffsetLocal);
void BODY(setExtForce)(int wid, int skid, int bid, double inv3[3], double inv3_2[3], bool _isForceLocal, bool _isOffsetLocal);
void BODY(addExtTorque)(int wid, int skid, int bid, double inv3[3], bool _isLocal);
void BODY(setExtTorque)(int wid, int skid, int bid, double inv3[3], bool _isLocal);

////////////////////////////////////////
// BodyNode::Jacobian Functions
void BODY(getJacobian)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols);
void BODY(getLinearJacobian)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols);
void BODY(getAngularJacobian)(int wid, int skid, int bid, double* outm, int nrows, int ncols);
void BODY(getWorldJacobian)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols);
void BODY(getLinearJacobianDeriv)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols);
void BODY(getAngularJacobianDeriv)(int wid, int skid, int bid, double* outm, int nrows, int ncols);


#endif // #ifndef PYDART2_BODYNODE_API_H
