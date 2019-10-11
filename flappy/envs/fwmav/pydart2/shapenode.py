from __future__ import absolute_import
# from builtins import object
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
from . import pydart2_api as papi
from .shape import create_shape


class ShapeNode(object):
    """
    """

    def __init__(self, _bodynode, _id):
        """
        """
        self.bodynode = _bodynode
        self.skeleton = _bodynode.skeleton
        self.id = _id
        # self.shape = Shape(self)
        self.shape = create_shape(self)

    @property
    def skel(self):
        return self.skeleton

    @property
    def wid(self):
        return self.skel.world.id

    @property
    def skid(self):
        return self.skel.id

    @property
    def bid(self):
        return self.bodynode.id

    def build(self):
        pass

    def has_visual_aspect(self, ):
        return papi.shapenode__hasVisualAspect(self.wid,
                                               self.skid,
                                               self.bid,
                                               self.id)

    def has_collision_aspect(self, ):
        return papi.shapenode__hasCollisionAspect(self.wid,
                                                  self.skid,
                                                  self.bid,
                                                  self.id)

    def visual_aspect_rgba(self, ):
        if not self.has_visual_aspect():
            return None
        return papi.shapenode__getVisualAspectRGBA(self.wid,
                                                   self.skid,
                                                   self.bid,
                                                   self.id)

    def set_visual_aspect_rgba(self, rgba):
        if not self.has_visual_aspect():
            return
        papi.shapenode__setVisualAspectRGBA(self.wid,
                                            self.skid,
                                            self.bid,
                                            self.id,
                                            rgba)

    def offset(self, ):
        return papi.shapenode__getOffset(self.wid,
                                         self.skid,
                                         self.bid,
                                         self.id)

    def set_offset(self, offset):
        papi.shapenode__setOffset(self.wid,
                                  self.skid,
                                  self.bid,
                                  self.id,
                                  offset)

    def relative_transform(self, ):
        return papi.shapenode__getRelativeTransform(self.wid,
                                                    self.skid,
                                                    self.bid,
                                                    self.id)

    def set_relative_transform(self, transform):
        papi.shapenode__setRelativeTransform(self.wid,
                                             self.skid,
                                             self.bid,
                                             self.id, transform)

    def __repr__(self):
        return '[ShapeNode(%d:%d)]' % (self.bid, self.id)
