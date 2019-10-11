#ifndef PYDART2_SHAPE_API_H
#define PYDART2_SHAPE_API_H

////////////////////////////////////////////////////////////////////////////////
// ShapeNode and Shape
#define SHAPENODE(funcname) shapenode__##funcname
#define GET_SHAPENODE(wid, skid, bid, sid) (Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid]);
#define SHAPE(funcname) shape__##funcname
#define GET_SHAPE(wid, skid, bid, sid) (Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get();

////////////////////////////////////////
// ShapeNode Functions
void SHAPENODE(getOffset)(int wid, int skid, int bid, int sid, double outv3[3]);
void SHAPENODE(setOffset)(int wid, int skid, int bid, int sid, double inv3[3]);
void SHAPENODE(getRelativeTransform)(int wid, int skid, int bid, int sid, double outv44[4][4]);
void SHAPENODE(setRelativeTransform)(int wid, int skid, int bid, int sid, double inv44[4][4]);

bool SHAPENODE(hasVisualAspect)(int wid, int skid, int bid, int sid);
bool SHAPENODE(hasCollisionAspect)(int wid, int skid, int bid, int sid);

void SHAPENODE(getVisualAspectRGBA)(int wid, int skid, int bid, int sid, double outv4[4]);
void SHAPENODE(setVisualAspectRGBA)(int wid, int skid, int bid, int sid, double inv4[4]);

////////////////////////////////////////
// Shape Functions
double SHAPE(getVolume)(int wid, int skid, int bid, int sid);
const char* SHAPE(getType)(int wid, int skid, int bid, int sid);
int SHAPE(getTypeID)(int wid, int skid, int bid, int sid);

// const char* SHAPE(getType)(int wid, int skid, int bid, int sid);
void SHAPE(render)(int wid, int skid, int bid, int sid);
void SHAPE(getBoundingBoxMin)(int wid, int skid, int bid, int sid, double outv3[3]);
void SHAPE(getBoundingBoxMax)(int wid, int skid, int bid, int sid, double outv3[3]);


////////////////////////////////////////
// SphereShape Functions
#define SPHERE_SHAPE(funcname) sphere_shape__##funcname
#define GET_SPHERE_SHAPE(wid, skid, bid, sid) dynamic_cast<dart::dynamics::SphereShape*>((Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get());

double SPHERE_SHAPE(getRadius)(int wid, int skid, int bid, int sid);
void SPHERE_SHAPE(setRadius)(int wid, int skid, int bid, int sid, double radius);


////////////////////////////////////////
// BoxShape Functions
#define BOX_SHAPE(funcname) box_shape__##funcname
#define GET_BOX_SHAPE(wid, skid, bid, sid) dynamic_cast<dart::dynamics::BoxShape*>((Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get());

void BOX_SHAPE(getSize)(int wid, int skid, int bid, int sid, double outv3[3]);
void BOX_SHAPE(setSize)(int wid, int skid, int bid, int sid, double inv3[3]);

////////////////////////////////////////
// EllipsoidShape Functions
#define ELLIPSOID_SHAPE(funcname) ellipsoid_shape__##funcname
#define GET_ELLIPSOID_SHAPE(wid, skid, bid, sid) dynamic_cast<dart::dynamics::EllipsoidShape*>((Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get());

void ELLIPSOID_SHAPE(getSize)(int wid, int skid, int bid, int sid, double outv3[3]);
void ELLIPSOID_SHAPE(setSize)(int wid, int skid, int bid, int sid, double inv3[3]);

////////////////////////////////////////
// CylinderShape Functions
#define CYLINDER_SHAPE(funcname) cylindershape__##funcname
#define GET_CYLINDER_SHAPE(wid, skid, bid, sid) dynamic_cast<dart::dynamics::CylinderShape*>((Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get());

double CYLINDER_SHAPE(getRadius)(int wid, int skid, int bid, int sid);
void CYLINDER_SHAPE(setRadius)(int wid, int skid, int bid, int sid, double _radius);
double CYLINDER_SHAPE(getHeight)(int wid, int skid, int bid, int sid);
void CYLINDER_SHAPE(setHeight)(int wid, int skid, int bid, int sid, double _height);


////////////////////////////////////////
// CapsuleShape Functions
#define CAPSULE_SHAPE(funcname) capsuleshape__##funcname
#define GET_CAPSULE_SHAPE(wid, skid, bid, sid) dynamic_cast<dart::dynamics::CapsuleShape*>((Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get());

double CAPSULE_SHAPE(getRadius)(int wid, int skid, int bid, int sid);
void CAPSULE_SHAPE(setRadius)(int wid, int skid, int bid, int sid, double radius);
double CAPSULE_SHAPE(getHeight)(int wid, int skid, int bid, int sid);
void CAPSULE_SHAPE(setHeight)(int wid, int skid, int bid, int sid, double height);


////////////////////////////////////////
// ConeShape Functions
#define CONE_SHAPE(funcname) coneshape__##funcname
#define GET_CONE_SHAPE(wid, skid, bid, sid) dynamic_cast<dart::dynamics::ConeShape*>((Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get());

double CONE_SHAPE(getRadius)(int wid, int skid, int bid, int sid);
void CONE_SHAPE(setRadius)(int wid, int skid, int bid, int sid, double radius);
double CONE_SHAPE(getHeight)(int wid, int skid, int bid, int sid);
void CONE_SHAPE(setHeight)(int wid, int skid, int bid, int sid, double height);


////////////////////////////////////////
// PlaneShape Functions
#define PLANE_SHAPE(funcname) planeshape__##funcname
#define GET_PLANE_SHAPE(wid, skid, bid, sid) dynamic_cast<dart::dynamics::PlaneShape*>((Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get());

void PLANE_SHAPE(getNormal)(int wid, int skid, int bid, int sid, double outv3[3]);
void PLANE_SHAPE(setNormal)(int wid, int skid, int bid, int sid, double inv3[3]);
double PLANE_SHAPE(getOffset)(int wid, int skid, int bid, int sid);
void PLANE_SHAPE(setOffset)(int wid, int skid, int bid, int sid, double _offset);


////////////////////////////////////////
// MultiSphereShape Functions
#define MULTISPHERE_SHAPE(funcname) multisphereshape__##funcname
#define GET_MULTISPHERE_SHAPE(wid, skid, bid, sid) dynamic_cast<dart::dynamics::MultiSphereShape*>((Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get());

void MULTISPHERE_SHAPE(addSphere)(int wid, int skid, int bid, int sid, double inv4[4]);
void MULTISPHERE_SHAPE(getSpheres)(int wid, int skid, int bid, int sid, double* outv, int nout);
int MULTISPHERE_SHAPE(getNumSpheres)(int wid, int skid, int bid, int sid);



////////////////////////////////////////
// MeshShape Functions
#define MESH_SHAPE(funcname) mesh_shape__##funcname
#define GET_MESH_SHAPE(wid, skid, bid, sid) dynamic_cast<dart::dynamics::MeshShape*>((Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get());

void MESH_SHAPE(getScale)(int wid, int skid, int bid, int sid, double outv3[3]);
const char* MESH_SHAPE(getMeshPath)(int wid, int skid, int bid, int sid);


#endif // #ifndef PYDART2_SHAPE_API_H
