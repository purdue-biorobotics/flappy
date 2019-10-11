from __future__ import absolute_import
# from builtins import range
# from builtins import object
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group


import os.path
import numpy as np
from . import pydart2_api as papi
from .skeleton import Skeleton
# from .bodynode import BodyNode

from .collision_result import CollisionResult
from .recording import Recording


def create_world(step, skel_path=None):
    skel_path = os.path.realpath(skel_path)
    return World(step, skel_path)


class World(object):
    DART_COLLISION_DETECTOR, \
        FCL_COLLISION_DETECTOR, \
        BULLET_COLLISION_DETECTOR, \
        ODE_COLLISION_DETECTOR = list(range(4))

    CLASS_SKELETON = Skeleton  # Modify this for inherited skeleton class

    def __init__(self, step, skel_path=None):
        self.skeletons = list()
        self.control_skel = None
        self.recording = None

        if skel_path is not None:
            skel_path = os.path.realpath(skel_path)
            self.id = papi.createWorldFromSkel(skel_path)
            self.set_time_step(step)
            nskels = self.num_skeletons()
            for i in range(nskels):
                self.add_skeleton_from_id(i)
        else:
            self.id = papi.createWorld(step)

        self.reset()
        self.enable_recording()

    def destroy(self):
        papi.destroyWorld(self.id)

    def add_skeleton(self, filename, CLASS=None):
        if CLASS is not None:
            skel = CLASS(self, _filename=filename)
        else:
            skel = World.CLASS_SKELETON(self, _filename=filename)
        self.skeletons.append(skel)
        return skel

    def add_skeleton_from_id(self, _skel_id):
        skel = World.CLASS_SKELETON(_world=self, _id=_skel_id)
        self.skeletons.append(skel)
        return skel

    def num_skeletons(self):
        return papi.world__getNumSkeletons(self.id)

    @property
    def skel(self):
        """ returns the default control skeleton """
        return self.control_skel

    def time(self):
        return papi.world__getTime(self.id)

    @property
    def t(self):
        return self.time()

    def time_step(self):
        return papi.world__getTimeStep(self.id)

    @property
    def frame(self):
        return self._frame

    @property
    def dt(self):
        return self.time_step()

    def set_time_step(self, _time_step):
        papi.world__setTimeStep(self.id, _time_step)

    @dt.setter
    def dt(self, _dt):
        self.set_time_step(_dt)

    def num_frames(self):
        # return papi.world__getSimFrames(self.id)
        if self.recording:
            return self.recording.num_frames()
        else:
            return 0

    @property
    def nframes(self):
        return self.num_frames()

    def set_frame(self, frame_index):
        if self.recording:
            return self.recording.set_frame(frame_index)
        return False

    def reset(self):
        papi.world__reset(self.id)
        self._frame = 0
        self.collision_result = CollisionResult(self)
        if self.recording:
            self.recording.clear()

    def step(self):
        for skel in self.skeletons:
            if skel.controller is not None:
                skel.tau = skel.controller.compute()

        papi.world__step(self.id)
        self._frame += 1
        self.collision_result.update()
        if self.recording:
            self.recording.bake()

    def check_collision(self, ):
        papi.world__checkCollision(self.id)
        self.collision_result.update()

    def render(self,
               render_markers=True,
               render_contacts=True,
               render_contact_size=0.01,
               render_contact_force_scale=-0.005):
        papi.world__render(self.id)
        if render_markers:
            self.render_markers()

        if render_contacts:
            self.render_contacts(render_contact_size,
                                 render_contact_force_scale)

    def render_markers(self, ):
        for skel in self.skeletons:
            for marker in skel.markers:
                marker.render()

    def render_contacts(self,
                        render_contact_size=0.01,
                        render_contact_force_scale=-0.005):
        for contact in self.collision_result.contacts:
            contact.render(size=render_contact_size,
                           scale=render_contact_force_scale)

    def states(self):
        if len(self.skeletons) == 0:
            return np.array(())
        return np.concatenate([skel.x for skel in self.skeletons])

    @property
    def x(self):
        return self.states()

    def set_states(self, _x):
        lo = 0
        for skel in self.skeletons:
            hi = lo + 2 * skel.ndofs
            skel.x = _x[lo:hi]
            lo = hi

    @x.setter
    def x(self, _x):
        self.set_states(_x)

    def gravity(self):
        return papi.world__getGravity(self.id)

    @property
    def g(self):
        return self.gravity()

    def set_gravity(self, _g):
        papi.world__setGravity(self.id, _g)

    @g.setter
    def g(self, _g):
        self.set_gravity(_g)

    def remove_all_constraints(self, ):
        papi.world__removeAllConstraints(self.id)

    def set_collision_detector(self, detector_type):
        """
        self.set_collision_detector(World.DART_COLLISION_DETECTOR)
        """
        papi.world__setCollisionDetector(self.id, detector_type)
        assert(detector_type == self.collision_detector())

    def collision_detector(self, ):
        return papi.world__getCollisionDetector(self.id)

    def collision_detector_string(self, ):
        typenames = ["DART_COLLISION_DETECTOR",
                     "FCL_COLLISION_DETECTOR",
                     "BULLET_COLLISION_DETECTOR", ]
        detector_type = self.collision_detector()
        return typenames[detector_type]

    # def set_collision_pair(self, body1, body2, is_enable):
    #     flag_enable = 1 if is_enable else 0
    #     papi.setWorldCollisionPair(self.id,
    #                                body1.skel.id, body1.id,
    #                                body2.skel.id, body2.id,
    #                                flag_enable)

    def is_recording(self, ):
        return (self.recording is not None)

    def set_recording(self, enable):
        if enable:
            self.enable_recording()
        else:
            self.disable_recording()

    def enable_recording(self, ):
        if self.recording is None:
            self.recording = Recording(self)
        assert(self.recording)

    def disable_recording(self, ):
        self.recording = None

    def __repr__(self):
        return "[World ID:%d time:%.4f # frames: %d]" % (
            self.id, self.t, self.num_frames())
