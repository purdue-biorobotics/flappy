#ifndef PYDART2_PYDART2_API_H
#define PYDART2_PYDART2_API_H


// #if __has_include("dart/dynamics/CapsuleShape.hpp")
// #define DART6_NEW_SHAPE_API
// #endif


#include <vector>

////////////////////////////////////////////////////////////////////////////////
// Init Functions
void init(bool verbose=true);
void destroy();

void setVerbose(bool verbose=true);
bool getVerbose();

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Miscellaneous API implementations

////////////////////////////////////////////////////////////////////////////////
// Marker
#define MARKER(funcname) marker__##funcname
#define GET_MARKER(wid, skid, mid) Manager::skeleton(wid, skid)->getMarker(mid)

int MARKER(getBodyNode)(int wid, int skid, int mid);
void MARKER(getLocalPosition)(int wid, int skid, int mid, double outv3[3]);
void MARKER(setLocalPosition)(int wid, int skid, int mid, double inv3[3]);
void MARKER(getWorldPosition)(int wid, int skid, int mid, double outv3[3]);
void MARKER(render)(int wid, int skid, int mid);

////////////////////////////////////////////////////////////////////////////////
// Contacts
#define COLLISION_RESULT(funcname) collisionresult__##funcname
#define GET_COLLISION_RESULT(wid) Manager::world(wid)->getConstraintSolver()->getLastCollisionResult();

int COLLISION_RESULT(getNumContacts)(int wid);
void COLLISION_RESULT(getContacts)(int wid, double* outv, int nout);
std::vector<int> COLLISION_RESULT(getCollidingBodyNodes)(int wid);
void COLLISION_RESULT(renderContact)(double inv6[6], double size, double scale);

////////////////////////////////////////////////////////////////////////////////
// Constraints
int addBallJointConstraint(int wid, int skid1, int bid1, int skid2, int bid2,
                           double inv3[3]);


#endif // #ifndef PYDART2_PYDART2_API_H
