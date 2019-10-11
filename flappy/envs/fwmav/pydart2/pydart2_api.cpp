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
#include "pydart2_world_api.h"
#include "pydart2_skeleton_api.h"
#include "pydart2_bodynode_api.h"
#include "pydart2_api.h"


#include "pydart2_draw.h"

using namespace pydart;

////////////////////////////////////////////////////////////////////////////////
// Init Functions
void init(bool verbose) {
    setVerbose(verbose);
    if (Manager::getInstance()) {
        Manager::destroy();
    }
    Manager::init();
}

void destroy() {
    Manager::destroy();
}

void setVerbose(bool verbose) {
    Manager::g_verbose = verbose;
}

bool getVerbose() {
    return Manager::g_verbose;
}

////////////////////////////////////////////////////////////////////////////////
// Marker

int MARKER(getBodyNode)(int wid, int skid, int mid) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    return marker->getBodyNodePtr()->getIndexInSkeleton();
}


void MARKER(getLocalPosition)(int wid, int skid, int mid, double outv3[3]) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    write(marker->getLocalPosition(), outv3);
}


void MARKER(setLocalPosition)(int wid, int skid, int mid, double inv3[3]) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    marker->setLocalPosition(read(inv3, 3));
}


void MARKER(getWorldPosition)(int wid, int skid, int mid, double outv3[3]) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    write(marker->getWorldPosition(), outv3);
}

void MARKER(render)(int wid, int skid, int mid) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawMarker(ri, marker);
}

////////////////////////////////////////////////////////////////////////////////
// Collision Result
int COLLISION_RESULT(getNumContacts)(int wid) {
    const auto result = GET_COLLISION_RESULT(wid);
    return result.getNumContacts();
}

void COLLISION_RESULT(getContacts)(int wid, double* outv, int nout) {
    const auto result = GET_COLLISION_RESULT(wid);
    const auto nContacts = static_cast<int>(result.getNumContacts());

    // Construct the skeleton index map
    std::map<const dart::dynamics::Skeleton*, int> indices;
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    for (size_t i = 0; i < world->getNumSkeletons(); ++i) {
        indices[world->getSkeleton(i).get()] = i;
    }

    Eigen::VectorXd state(10 * nContacts);
    for (auto i = 0; i < nContacts; ++i) {
        auto begin = i * 10;
        auto contact = result.getContact(i);

        state.segment(begin, 3)     = contact.point;
        state.segment(begin + 3, 3) = contact.force;
        state(begin + 6) = -1;
        state(begin + 7) = -1;
        state(begin + 8) = -1;
        state(begin + 9) = -1;

        auto shapeNode1 = contact.collisionObject1->getShapeFrame()->asShapeNode();
        if (shapeNode1) {
            auto b = shapeNode1->getBodyNodePtr();
            state(begin + 6) = indices[b->getSkeleton().get()];
            state(begin + 7) = b->getIndexInSkeleton();
        }

        auto shapeNode2 = contact.collisionObject2->getShapeFrame()->asShapeNode();
        if (shapeNode2) {
            auto b = shapeNode2->getBodyNodePtr();
            state(begin + 8) = indices[b->getSkeleton().get()];
            state(begin + 9) = b->getIndexInSkeleton();
        }
    }
    write(state, outv);
}

std::vector<int> COLLISION_RESULT(getCollidingBodyNodes)(int wid) {
    const auto result = GET_COLLISION_RESULT(wid);
    const auto bodynodes = result.getCollidingBodyNodes();
    std::vector<int> ret;
    std::map<const dart::dynamics::Skeleton*, int> indices;
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    for (size_t i = 0; i < world->getNumSkeletons(); ++i) {
        indices[world->getSkeleton(i).get()] = i;
    }

    for (auto b: bodynodes) {
        ret.push_back(indices[b->getSkeleton().get()]);
        ret.push_back(b->getIndexInSkeleton());
    }
    return ret;
}

void COLLISION_RESULT(renderContact)(double inv6[6], double size, double scale) {
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawContact(ri, read(inv6, 6), size, scale);
}

////////////////////////////////////////////////////////////////////////////////
// Constraints
int addBallJointConstraint(int wid, int skid1, int bid1, int skid2, int bid2,
                           double inv3[3]) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    dart::dynamics::BodyNodePtr bd1 = GET_BODY(wid, skid1, bid1);
    dart::dynamics::BodyNodePtr bd2 = GET_BODY(wid, skid2, bid2);
    Eigen::Vector3d jointPos = read(inv3, 3);
    // MSG << bd1->getName() << "\n";
    // MSG << bd2->getName() << "\n";
    // MSG << jointPos << "\n";
    dart::constraint::BallJointConstraintPtr cl =
        std::make_shared<dart::constraint::BallJointConstraint>(bd1, bd2, jointPos);
    world->getConstraintSolver()->addConstraint(cl);
    return 0;
}
