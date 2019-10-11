from __future__ import division
from __future__ import absolute_import
# from builtins import object
from past.utils import old_div
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
import numpy as np
from . import pydart2_api as papi
from .contact import Contact


class CollisionResult(object):
    """
    """

    def __init__(self, _world):
        """
        """
        self.world = _world
        self.id = self.world.id
        self.contacts = list()
        self.contacted_bodies = list()

    def num_contacts(self,):
        return len(self.contacts)

    def num_contacted_bodies(self,):
        return len(self.contacted_bodies)

    def copy(self, ):
        ret = CollisionResult(self.world)
        ret.id = self.id
        ret.contacts = list(self.contacts)
        ret.contacted_bodies = list(self.contacted_bodies)
        return ret

    def update(self,):
        self.contacts = list()
        self.contacted_bodies = list()

        # contacts
        n = papi.collisionresult__getNumContacts(self.id)
        v = papi.collisionresult__getContacts(self.id, n * 10)
        if n > 0:
            self.contacts = [Contact(self.world, v_i)
                             for v_i in np.split(v, n)]

        # contacted_bodies
        ids = np.array(papi.collisionresult__getCollidingBodyNodes(self.id))
        n = old_div(len(ids), 2)
        if n > 0:
            self.contacted_bodies = [self.world.skeletons[i].bodynodes[j]
                                     for i, j in np.split(ids, n)]

    def __repr__(self,):
        ret = "[CollisionResult: %d contacts %d contacted bodies]" % (
            self.num_contacts(), self.num_contacted_bodies())
        return ret
