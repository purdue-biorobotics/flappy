# Author(s): Sehoon Ha <sehoon.ha@gmail.com>
import numpy as np
from pydart2.skeleton import Skeleton


class MusculoSkeleton(Skeleton):
    """
    world = pydart2.world.World(0.001)
    world.add_skeleton("data/skeleton_file", MusculoSkeleton)
    """

    def __init__(self, _world, _filename=None,
                 _id=None, _friction=None, ):
        Skeleton.__init__(self, _world, _filename, _id, _friction)
        self.muscles = list()

    def num_muscles(self, ):
        return len(self.muscles)

    def add_muscle(self, mtu, route):
        route.initialize_points(self)
        route.update_geometry_variables()
        self.muscles.append((mtu, route))

    def update(self, ):
        for mtu in self.tendon_units:
            mtu.update()

    def update_route_geometry(self, ):
        for _, route in self.muscles:
            route.update_geometry_variables()

    def reset_muscles(self, ):
        for (mtu, route) in self.muscles:
            mtu.reset()

    def stimulate(self, stimulation):
        self.update_route_geometry()
        for S, (mtu, route) in zip(stimulation, self.muscles):
            mtu.update(S, route.length)

    def apply_muscle_forces(self, ):
        for (mtu, route) in self.muscles:
            f_mtu = mtu.F_mtu
            for (o0, o1), (b0, b1), u in zip(route.local_points_as_pair(),
                                             route.bodynodes_as_pair(),
                                             route.world_directions):
                if b0 == b1:
                    continue
                F_ext = f_mtu * u
                b0.add_ext_force(F_ext, o0)
                b1.add_ext_force(-F_ext, o1)

    def render_with_ri(self, ri):
        r = np.array([1.0, 0.0, 0.0])
        b = np.array([0.0, 0.0, 1.0])
        for mtu, route in self.muscles:
            w = mtu.a
            color = (1 - w) * b + w * r
            ri.set_color(*color)
            route.render_with_ri(ri)

    def __repr__(self):
        args = (self.id, self.name, self.num_muscles())
        return '[MusculoSkeleton(%d): %s # Muscles = %d]' % args
