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
#include "pydart2_shape_api.h"
#include "pydart2_draw.h"

using namespace pydart;


////////////////////////////////////////////////////////////////////////////////
// ShapeNode and Shape
void SHAPENODE(getOffset)(int wid, int skid, int bid, int sid, double outv3[3]) {
    dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
    write(shapenode->getOffset(), outv3);
}


void SHAPENODE(setOffset)(int wid, int skid, int bid, int sid, double inv3[3]) {
    dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
    shapenode->setOffset(read(inv3, 3));
}


void SHAPENODE(getRelativeTransform)(int wid, int skid, int bid, int sid, double outv44[4][4]) {
    dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
    write_isometry(shapenode->getRelativeTransform(), outv44);
}


void SHAPENODE(setRelativeTransform)(int wid, int skid, int bid, int sid, double inv44[4][4]) {
    dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
    shapenode->setRelativeTransform(read_isometry(inv44));
}

bool SHAPENODE(hasVisualAspect)(int wid, int skid, int bid, int sid) {
    dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
    return shapenode->has<dart::dynamics::VisualAspect>();
}

bool SHAPENODE(hasCollisionAspect)(int wid, int skid, int bid, int sid) {
    dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
    return shapenode->has<dart::dynamics::CollisionAspect>();
}

void SHAPENODE(getVisualAspectRGBA)(int wid, int skid, int bid, int sid, double outv4[4]) {
  dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
  auto visualAspect = shapenode->getVisualAspect();
  write(visualAspect->getRGBA(), outv4);

}
void SHAPENODE(setVisualAspectRGBA)(int wid, int skid, int bid,
  int sid, double inv4[4]) {
  dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
  auto visualAspect = shapenode->getVisualAspect();
  visualAspect->setRGBA(read(inv4, 4));
}


////////////////////////////////////////
// Shape Functions

double SHAPE(getVolume)(int wid, int skid, int bid, int sid) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    return shape->getVolume();
}

const char* SHAPE(getType)(int wid, int skid, int bid, int sid) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    return shape->getType().c_str();
}


// #ifdef DART6_NEW_SHAPE_API
int SHAPE(getTypeID)(int wid, int skid, int bid, int sid) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    using dart::dynamics::Shape;
    using dart::dynamics::SphereShape;
    using dart::dynamics::BoxShape;
    using dart::dynamics::EllipsoidShape;
    using dart::dynamics::CylinderShape;
    using dart::dynamics::CapsuleShape;
    using dart::dynamics::ConeShape;
    using dart::dynamics::PlaneShape;
    using dart::dynamics::MultiSphereShape;
    using dart::dynamics::MeshShape;
    using dart::dynamics::SoftMeshShape;
    using dart::dynamics::LineSegmentShape;

    // dtmsg << shape->getType() << "\n";

    if (shape->is<SphereShape>()) return 0;
    else if (shape->is<BoxShape>()) return 1;
    else if (shape->is<EllipsoidShape>()) return 2;
    else if (shape->is<CylinderShape>()) return 3;
    else if (shape->is<CapsuleShape>()) return 4;
    else if (shape->is<ConeShape>()) return 5;
    else if (shape->is<PlaneShape>()) return 6;
    else if (shape->is<MultiSphereShape>()) return 7;
    else if (shape->is<MeshShape>()) return 8;
    else if (shape->is<SoftMeshShape>()) return 9;
    else if (shape->is<LineSegmentShape>()) return 10;
    return -1;
}


void SHAPE(render)(int wid, int skid, int bid, int sid) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawShape(ri, shape);
}

void SHAPE(getBoundingBoxMin)(int wid, int skid, int bid, int sid, double outv3[3]) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    write(shape->getBoundingBox().getMin(), outv3);
}

void SHAPE(getBoundingBoxMax)(int wid, int skid, int bid, int sid, double outv3[3]) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    write(shape->getBoundingBox().getMax(), outv3);
}


////////////////////////////////////////
// SphereShape Functions
double SPHERE_SHAPE(getRadius)(int wid, int skid, int bid, int sid) {
    dart::dynamics::SphereShape* shape = GET_SPHERE_SHAPE(wid, skid, bid, sid);
    return shape->getRadius();
}

void SPHERE_SHAPE(setRadius)(int wid, int skid, int bid, int sid, double radius) {
    dart::dynamics::SphereShape* shape = GET_SPHERE_SHAPE(wid, skid, bid, sid);
    shape->setRadius(radius);
}

////////////////////////////////////////
// BoxShape Functions
void BOX_SHAPE(getSize)(int wid, int skid, int bid, int sid, double outv3[3]) {
    dart::dynamics::BoxShape* shape = GET_BOX_SHAPE(wid, skid, bid, sid);
    write(shape->getSize(), outv3);
}

void BOX_SHAPE(setSize)(int wid, int skid, int bid, int sid, double inv3[3]) {
    dart::dynamics::BoxShape* shape = GET_BOX_SHAPE(wid, skid, bid, sid);
    shape->setSize(read(inv3, 3));
}


////////////////////////////////////////
// EllipsoidShape Functions
void ELLIPSOID_SHAPE(getSize)(int wid, int skid, int bid, int sid, double outv3[3]) {
    dart::dynamics::EllipsoidShape* shape = GET_ELLIPSOID_SHAPE(wid, skid, bid, sid);
    if (shape != NULL) {
      write(shape->getSize(), outv3);
    }
}

void ELLIPSOID_SHAPE(setSize)(int wid, int skid, int bid, int sid, double inv3[3]) {
    dart::dynamics::EllipsoidShape* shape = GET_ELLIPSOID_SHAPE(wid, skid, bid, sid);
    if (shape != NULL) {
        shape->setSize(read(inv3, 3));
    }
}


////////////////////////////////////////
// CylinderShape Functions
double CYLINDER_SHAPE(getRadius)(int wid, int skid, int bid, int sid) {
    auto shape = GET_CYLINDER_SHAPE(wid, skid, bid, sid);
    return shape->getRadius();
}


void CYLINDER_SHAPE(setRadius)(int wid, int skid, int bid, int sid, double _radius) {
    auto shape = GET_CYLINDER_SHAPE(wid, skid, bid, sid);
    shape->setRadius(_radius);
}


double CYLINDER_SHAPE(getHeight)(int wid, int skid, int bid, int sid) {
    auto shape = GET_CYLINDER_SHAPE(wid, skid, bid, sid);
    return shape->getHeight();
}


void CYLINDER_SHAPE(setHeight)(int wid, int skid, int bid, int sid, double _height) {
    auto shape = GET_CYLINDER_SHAPE(wid, skid, bid, sid);
    shape->setHeight(_height);
}


////////////////////////////////////////
// Capsule Functions
double CAPSULE_SHAPE(getRadius)(int wid, int skid, int bid, int sid) {
    auto shape = GET_CAPSULE_SHAPE(wid, skid, bid, sid);
    return shape->getRadius();
}


void CAPSULE_SHAPE(setRadius)(int wid, int skid, int bid, int sid, double radius) {
    auto shape = GET_CAPSULE_SHAPE(wid, skid, bid, sid);
    shape->setRadius(radius);
}


double CAPSULE_SHAPE(getHeight)(int wid, int skid, int bid, int sid) {
    auto shape = GET_CAPSULE_SHAPE(wid, skid, bid, sid);
    return shape->getHeight();
}


void CAPSULE_SHAPE(setHeight)(int wid, int skid, int bid, int sid, double height) {
    auto shape = GET_CAPSULE_SHAPE(wid, skid, bid, sid);
    shape->setHeight(height);
}

////////////////////////////////////////
// ConeShape Functions
double CONE_SHAPE(getRadius)(int wid, int skid, int bid, int sid) {
    auto shape = GET_CONE_SHAPE(wid, skid, bid, sid);
    return shape->getRadius();
}


void CONE_SHAPE(setRadius)(int wid, int skid, int bid, int sid, double radius) {
    auto shape = GET_CONE_SHAPE(wid, skid, bid, sid);
    shape->setRadius(radius);
}


double CONE_SHAPE(getHeight)(int wid, int skid, int bid, int sid) {
    auto shape = GET_CONE_SHAPE(wid, skid, bid, sid);
    return shape->getHeight();
}


void CONE_SHAPE(setHeight)(int wid, int skid, int bid, int sid, double height) {
    auto shape = GET_CONE_SHAPE(wid, skid, bid, sid);
    shape->setHeight(height);
}

////////////////////////////////////////
// PlaneShape Functions
void PLANE_SHAPE(getNormal)(int wid, int skid, int bid, int sid, double outv3[3]) {
    auto shape = GET_PLANE_SHAPE(wid, skid, bid, sid);
    write(shape->getNormal(), outv3);
}


void PLANE_SHAPE(setNormal)(int wid, int skid, int bid, int sid, double inv3[3]) {
    auto shape = GET_PLANE_SHAPE(wid, skid, bid, sid);
    shape->setNormal(read(inv3, 3));
}


double PLANE_SHAPE(getOffset)(int wid, int skid, int bid, int sid) {
    auto shape = GET_PLANE_SHAPE(wid, skid, bid, sid);
    return shape->getOffset();
}


void PLANE_SHAPE(setOffset)(int wid, int skid, int bid, int sid, double _offset) {
    auto shape = GET_PLANE_SHAPE(wid, skid, bid, sid);
    shape->setOffset(_offset);
}

////////////////////////////////////////
// MultiSphereShape Functions
void MULTISPHERE_SHAPE(addSphere)(int wid, int skid, int bid, int sid, double inv4[4]) {
    auto shape = GET_MULTISPHERE_SHAPE(wid, skid, bid, sid);
    shape->addSphere(inv4[0], Eigen::Vector3d(inv4[1], inv4[2], inv4[3]));
}

void MULTISPHERE_SHAPE(getSpheres)(int wid, int skid, int bid, int sid, double* outv, int nout) {
    auto shape = GET_MULTISPHERE_SHAPE(wid, skid, bid, sid);
    auto spheres = shape->getSpheres();

    int ptr = 0;
    for (auto sph: spheres) {
        auto r = sph.first;
        auto pos = sph.second;
        outv[ptr++] = r;
        outv[ptr++] = pos[0];
        outv[ptr++] = pos[1];
        outv[ptr++] = pos[2];
    }
}

int MULTISPHERE_SHAPE(getNumSpheres)(int wid, int skid, int bid, int sid) {
    auto shape = GET_MULTISPHERE_SHAPE(wid, skid, bid, sid);
    return shape->getNumSpheres();
}


////////////////////////////////////////
// MeshShape Functions
void MESH_SHAPE(getScale)(int wid, int skid, int bid, int sid, double outv3[3]) {
    dart::dynamics::MeshShape* shape = GET_MESH_SHAPE(wid, skid, bid, sid);
    write(shape->getScale(), outv3);
}

const char* MESH_SHAPE(getMeshPath)(int wid, int skid, int bid, int sid) {
    dart::dynamics::MeshShape* shape = GET_MESH_SHAPE(wid, skid, bid, sid);
    return shape->getMeshPath().c_str();
}
