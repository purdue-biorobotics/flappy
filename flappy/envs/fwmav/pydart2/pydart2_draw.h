#ifndef PYDART2_PYDART2_DRAW_H
#define PYDART2_PYDART2_DRAW_H

#include <Eigen/Dense>

#include "dart/simulation/World.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/MultiSphereShape.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/LineSegmentShape.hpp"
#include "dart/dynamics/Marker.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/collision/CollisionDetector.hpp"
#include "dart/gui/LoadGlut.hpp"
#include "dart/gui/GLFuncs.hpp"
#include "dart/gui/GraphWindow.hpp"
#include "dart/utils/FileInfoWorld.hpp"


void drawWorld(
    dart::gui::RenderInterface* ri,
    dart::simulation::WorldPtr world);

void drawSkeletons(
    dart::gui::RenderInterface* ri,
    dart::simulation::WorldPtr world);

void drawSkeleton(
    dart::gui::RenderInterface* ri,
    const dart::dynamics::Skeleton* skeleton,
    const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5),
    bool useDefaultColor = true);

void drawEntity(
    dart::gui::RenderInterface* ri,
    const dart::dynamics::Entity* entity,
    const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5),
    bool useDefaultColor = true);

void drawBodyNode(
    dart::gui::RenderInterface* ri,
    const dart::dynamics::BodyNode* bodyNode,
    const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5),
    bool useDefaultColor = true,
    bool recursive = false);

void drawShapeFrame(
    dart::gui::RenderInterface* ri,
    const dart::dynamics::ShapeFrame* shapeFrame,
    const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5),
    bool useDefaultColor = true);

void drawShape(
    dart::gui::RenderInterface* ri,
    const dart::dynamics::Shape* shape,
    const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5));

void drawPointMasses(
    dart::gui::RenderInterface* ri,
    const std::vector<dart::dynamics::PointMass*> pointMasses,
    const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5),
    bool useDefaultColor = true);

void drawMarker(
    dart::gui::RenderInterface* ri,
    const dart::dynamics::Marker* marker,
    const Eigen::Vector4d& color = Eigen::Vector4d::Constant(0.5),
    bool useDefaultColor = true);

void drawContact(
    dart::gui::RenderInterface* ri,
    const Eigen::Vector6d& state,
    double size,
    double scale);

#endif // #ifndef PYDART2_PYDART2_DRAW_H
