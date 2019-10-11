#ifndef PYDART2_MANAGER_H
#define PYDART2_MANAGER_H



// Dart headers
#include "dart/dart.hpp"
#ifdef PYDART2_BULLET_FOUND
#include "dart/collision/bullet/BulletCollisionDetector.hpp"
#endif
#ifdef PYDART2_ODE_FOUND
#include "dart/collision/ode/OdeCollisionDetector.hpp"
#endif
#include "dart/gui/gui.hpp"
#include "dart/utils/utils.hpp"
#include "dart/utils/urdf/DartLoader.hpp"

namespace pydart {

////////////////////////////////////////////////////////////
// class Manager
class Manager {
public:
    static void init();
    static void destroy();
    static Manager* getInstance() { return g_manager; }
    static dart::gui::RenderInterface* getRI() {
        return g_ri;
    }

    static dart::simulation::WorldPtr world(int index = 0);
    static dart::dynamics::SkeletonPtr skeleton(int index);
    static dart::dynamics::SkeletonPtr skeleton(int wid, int skid);
    static int createWorld(double timestep);
    static int createWorldFromSkel(const char* const path);
    static void destroyWorld(int id);
    static bool g_verbose;
protected:
    static Manager* g_manager;
    static dart::gui::RenderInterface* g_ri;

    // std::vector<dart::simulation::WorldPtr> worlds;
    int next_id;
    std::map<int, dart::simulation::WorldPtr> worlds;

};

} // namespace pydart

void write(const Eigen::VectorXd& src, double* dst);
void write_matrix(const Eigen::MatrixXd& src, double* dst);
void write_isometry(const Eigen::Isometry3d& src, double dst[4][4]);
Eigen::VectorXd read(double* src, int n);
Eigen::Isometry3d read_isometry(double src[4][4]);

#define MSG if (Manager::g_verbose) dtmsg
#define DBG if (Manager::g_verbose) dtdbg
#define WARN if (Manager::g_verbose) dtwarn
#define ERR if (Manager::g_verbose) dterr

#endif // #ifndef PYDART2_MANAGER_H
