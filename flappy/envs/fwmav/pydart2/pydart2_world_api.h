#ifndef PYDART2_WORLD_API_H
#define PYDART2_WORLD_API_H

////////////////////////////////////////////////////////////////////////////////
// World
#define WORLD(funcname) world__##funcname
#define GET_WORLD(wid) Manager::world(wid)

int createWorld(double timestep);
int createWorldFromSkel(const char* const path);
void destroyWorld(int wid);

int WORLD(addSkeleton)(int wid, const char* const path);
int WORLD(getNumSkeletons)(int wid);

void WORLD(reset)(int wid);
void WORLD(step)(int wid);
void WORLD(checkCollision)(int wid);
void WORLD(render)(int wid);

////////////////////////////////////////
// World::Time Functions
void WORLD(setTimeStep)(int wid, double _timeStep);
double WORLD(getTimeStep)(int wid);
void WORLD(setTime)(int wid, double _time);
double WORLD(getTime)(int wid);
int WORLD(getSimFrames)(int wid);
int WORLD(getIndex)(int wid, int _index);

////////////////////////////////////////
// World::Property Functions
void WORLD(setGravity)(int wid, double inv3[3]);
void WORLD(getGravity)(int wid, double outv3[3]);

////////////////////////////////////////
// World::CollisionDetector Functions
void WORLD(setCollisionDetector)(int wid, int detector_type);
int WORLD(getCollisionDetector)(int wid);


////////////////////////////////////////
// World::Constraint Functions
void WORLD(removeAllConstraints)(int wid);

#endif // #ifndef PYDART2_WORLD_API_H
