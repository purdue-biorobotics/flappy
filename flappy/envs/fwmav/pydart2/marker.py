from __future__ import absolute_import
# from builtins import object
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
from . import pydart2_api as papi


class Marker(object):
    """
    """
    def __init__(self, _skeleton, _id):
        """
        """
        self.skeleton = _skeleton
        self.id = _id
        self.name = "M%04d" % _id
        self.bodynode = None

    @property
    def skel(self):
        return self.skeleton

    @property
    def wid(self):
        return self.skel.world.id

    @property
    def skid(self):
        return self.skel.id

    def build(self):
        self.bodynode = None

        ret_id = papi.marker__getBodyNode(self.wid, self.skid, self.id)
        if ret_id >= 0:
            self.bodynode = self.skel.bodynodes[ret_id]
            self.bodynode.markers.append(self)

    def local_position(self, ):
        return papi.marker__getLocalPosition(self.wid, self.skid, self.id)

    def set_local_position(self, inv3):
        papi.marker__setLocalPosition(self.wid, self.skid, self.id, inv3)

    def world_position(self, ):
        return papi.marker__getWorldPosition(self.wid, self.skid, self.id)

    @property
    def p(self, ):
        return self.world_position

    def render(self, ):
        return papi.marker__render(self.wid, self.skid, self.id)

    def __repr__(self):
        return '[Marker(%d)]' % (self.id)
