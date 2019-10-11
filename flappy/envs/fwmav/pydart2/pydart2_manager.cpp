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
#include "dart/gui/LoadOpengl.hpp"

namespace pydart {

Manager* Manager::g_manager = NULL;
bool Manager::g_verbose = true;
dart::gui::RenderInterface* Manager::g_ri = NULL;


void Manager::init() {
    MSG << " [pydart2_api] init...\n";
    g_manager = new Manager();
    MSG << " [pydart2_api] manager OK...\n";
    g_ri = new dart::gui::OpenGLRenderInterface();
    MSG << " [pydart2_api] RI OK...\n";
    // g_ri->initialize();
    // glMatrixMode(GL_MODELVIEW);
    // glLoadIdentity();
    // glMatrixMode(GL_PROJECTION);
    // glLoadIdentity();
    // glCullFace(GL_FRONT);
    // glDisable(GL_LIGHTING);
    // glEnable(GL_DEPTH_TEST);
    // //glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    // glShadeModel(GL_SMOOTH);
    // clear(Eigen::Vector3d(1.0, 1.0, 1.0));

    MSG << " [pydart2_api] RI init OK...\n";
    g_manager->next_id = 0;
    // if (verbose) dtmsg << "Hello!";
    MSG << " [pydart2_api] Initialize pydart manager OK\n";
}

void Manager::destroy() {
    if (g_manager) {
        delete g_manager;
        g_manager = NULL;
    }
    if (g_ri) {
        delete g_ri;
        g_ri = NULL;
    }
    MSG << " [pydart2_api] Destroy pydart manager OK\n";
}

dart::simulation::WorldPtr Manager::world(int index) {
    Manager* manager = getInstance();
    return manager->worlds[index];
}

dart::dynamics::SkeletonPtr Manager::skeleton(int index) {
    return world()->getSkeleton(index);
}

dart::dynamics::SkeletonPtr Manager::skeleton(int wid, int skid) {
    return world(wid)->getSkeleton(skid);
}


int Manager::createWorld(double timestep) {
    Manager* manager = getInstance();

    dart::simulation::WorldPtr w(new dart::simulation::World());
    w->setTimeStep(timestep);
    w->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
    // int id = manager->worlds.size();
    // manager->worlds.push_back(w);
    int id = manager->next_id++;
    manager->worlds[id] = w;
    return id;
}

int Manager::createWorldFromSkel(const char* const path) {
    Manager* manager = getInstance();

    dart::simulation::WorldPtr w(dart::utils::SkelParser::readWorld(path));
    // w->setTimeStep(timestep);
    // w->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
    int id = manager->next_id++;
    manager->worlds[id] = w;
    // int id = manager->worlds.size();
    // manager->worlds.push_back(w);
    MSG << " [pydart2_api] worlds.size = " << manager->worlds.size() << "\n";
    MSG << " [pydart2_api] worlds.# skeletons = " << w->getNumSkeletons() << "\n";
    return id;
}

void Manager::destroyWorld(int id) {
    Manager* manager = getInstance();
    dart::simulation::WorldPtr w = manager->worlds[id];
    manager->worlds.erase(id);
    MSG << " [pydart2_api] worlds.size = " << manager->worlds.size() << "\n";
    // w.reset();
    MSG << " [pydart2_api] Destroy world OK: " << id << "\n";
}

// class Manager
////////////////////////////////////////////////////////////

} // namespace pydart



////////////////////////////////////////////////////////////////////////////////
// Helper functions
void write(const Eigen::VectorXd& src, double* dst) {
    for (int i = 0; i < src.size(); i++) {
        dst[i] = src(i);
    }
}

void write_matrix(const Eigen::MatrixXd& src, double* dst) {
    int ptr = 0;
    for (int i = 0; i < src.rows(); i++) {
        for (int j = 0; j < src.cols(); j++) {
            dst[ptr++] = src(i, j);
        }
    }
}

void write_isometry(const Eigen::Isometry3d& src, double dst[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            dst[i][j] = src(i, j);
        }
    }
}

Eigen::VectorXd read(double* src, int n) {
    Eigen::VectorXd dst(n);
    for (int i =0; i < n; i++) {
        dst(i) = src[i];
    }
    return dst;
}

Eigen::Isometry3d read_isometry(double src[4][4]) {
    Eigen::Isometry3d dst;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            dst(i, j) = src[i][j];
        }
    }
    return dst;
}
