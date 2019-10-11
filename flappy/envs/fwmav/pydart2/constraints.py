from __future__ import absolute_import
# from builtins import object
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
from . import pydart2_api as papi


class BallJointConstraint(object):
    def __init__(self, body1, body2, jointPos):
        self.body1 = body1
        self.body2 = body2
        self.jointPos = jointPos

    def add_to_world(self, world):
        papi.addBallJointConstraint(world.id,
                                    self.body1.skid,
                                    self.body1.id,
                                    self.body2.skid,
                                    self.body2.id,
                                    self.jointPos)
