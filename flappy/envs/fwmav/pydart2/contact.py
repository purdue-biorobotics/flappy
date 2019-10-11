from __future__ import absolute_import
# from builtins import object
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
from . import pydart2_api as papi


class Contact(object):
    """
    """
    def __init__(self, _world, _state):
        """
        """
        self.world = _world
        self.state = _state
        self.point = _state[:3]
        self.force = _state[3:6]
        self.skel_id1 = int(_state[6])
        self.bodynode_id1 = int(_state[7])
        self.skel_id2 = int(_state[8])
        self.bodynode_id2 = int(_state[9])

        # self.skel_id1, self.bodynode_id1 = _state[6:8]
        # self.skel_id2, self.bodynode_id2 = _state[8:10]
        self.bodynode1 = None
        self.bodynode2 = None
        # print self.skel_id1, self.bodynode_id1,
        # print self.skel_id2, self.bodynode_id2, _state
        if self.skel_id1 >= 0 and self.bodynode_id1 >= 0:
            skel = self.world.skeletons[self.skel_id1]
            self.bodynode1 = skel.bodynodes[self.bodynode_id1]
        if self.skel_id2 >= 0 and self.bodynode_id2 >= 0:
            skel = self.world.skeletons[self.skel_id2]
            self.bodynode2 = skel.bodynodes[self.bodynode_id2]
        # print self.bodynode1, self.bodynode2

    @property
    def p(self, ):
        return self.point

    @property
    def f(self, ):
        return self.force

    def render(self, size, scale):
        papi.collisionresult__renderContact(self.state[:6], size, scale)

    def __repr__(self, ):
        return "[Contact %s %s]" % (self.point, self.force)
