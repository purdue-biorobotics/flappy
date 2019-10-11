from __future__ import absolute_import
# from builtins import object
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
from . import pydart2_api as papi


class Dof(object):
    """
    """
    def __init__(self, _skeleton, _id):
        """
        """
        self.skeleton = _skeleton
        self.id = _id
        self.name = papi.dof__getName(self.wid, self.skid, self.id)
        self.index = self.index_in_skeleton()
        self.joint = None

    @property
    def skel(self):
        return self.skeleton

    @property
    def wid(self):
        return self.skel.world.id

    @property
    def skid(self):
        return self.skel.id

########################################
# Dof::Index functions
    def index_in_skeleton(self, ):
        return papi.dof__getIndexInSkeleton(self.wid,
                                            self.skid,
                                            self.id)

    def index_in_tree(self, ):
        return papi.dof__getIndexInTree(self.wid,
                                        self.skid,
                                        self.id)

    def index_in_joint(self, ):
        return papi.dof__getIndexInJoint(self.wid,
                                         self.skid,
                                         self.id)

    def tree_index(self, ):
        return papi.dof__getTreeIndex(self.wid, self.skid, self.id)

########################################
# Dof::Position functions
    def position(self, ):
        return papi.dof__getPosition(self.wid, self.skid, self.id)

    def set_position(self, _position):
        papi.dof__setPosition(self.wid, self.skid, self.id, _position)

    def initial_position(self, ):
        return papi.dof__getInitialPosition(self.wid, self.skid, self.id)

    def set_initial_position(self, _initial):
        papi.dof__setInitialPosition(self.wid, self.skid, self.id, _initial)

    def has_position_limit(self, ):
        return papi.dof__hasPositionLimit(self.wid, self.skid, self.id)

    def position_lower_limit(self, ):
        return papi.dof__getPositionLowerLimit(self.wid, self.skid, self.id)

    def set_position_lower_limit(self, _limit):
        papi.dof__setPositionLowerLimit(self.wid, self.skid, self.id, _limit)

    def position_upper_limit(self, ):
        return papi.dof__getPositionUpperLimit(self.wid, self.skid, self.id)

    def set_position_upper_limit(self, _limit):
        papi.dof__setPositionUpperLimit(self.wid, self.skid, self.id, _limit)

########################################
# Dof::Velocity functions
    def velocity(self, ):
        return papi.dof__getVelocity(self.wid, self.skid, self.id)

    def set_velocity(self, _velocity):
        papi.dof__setVelocity(self.wid, self.skid, self.id, _velocity)

    def initial_velocity(self, ):
        return papi.dof__getInitialVelocity(self.wid, self.skid, self.id)

    def set_initial_velocity(self, _initial):
        papi.dof__setInitialVelocity(self.wid, self.skid, self.id, _initial)

    def velocity_lower_limit(self, ):
        return papi.dof__getVelocityLowerLimit(self.wid, self.skid, self.id)

    def set_velocity_lower_limit(self, _limit):
        papi.dof__setVelocityLowerLimit(self.wid, self.skid, self.id, _limit)

    def velocity_upper_limit(self, ):
        return papi.dof__getVelocityUpperLimit(self.wid, self.skid, self.id)

    def set_velocity_upper_limit(self, _limit):
        papi.dof__setVelocityUpperLimit(self.wid, self.skid, self.id, _limit)

########################################
# Dof::Passive Force functions
    def spring_stiffness(self, ):
        return papi.dof__getSpringStiffness(self.wid, self.skid, self.id)

    def set_spring_stiffness(self, _k):
        papi.dof__setSpringStiffness(self.wid, self.skid, self.id, _k)

    def rest_position(self, ):
        return papi.dof__getRestPosition(self.wid, self.skid, self.id)

    def set_rest_position(self, _q0):
        papi.dof__setRestPosition(self.wid, self.skid, self.id, _q0)

    def damping_coefficient(self, ):
        return papi.dof__getDampingCoefficient(self.wid, self.skid, self.id)

    def set_damping_coefficient(self, _coeff):
        papi.dof__setDampingCoefficient(self.wid, self.skid, self.id, _coeff)

    def coulomb_friction(self, ):
        return papi.dof__getCoulombFriction(self.wid, self.skid, self.id)

    def set_coulomb_friction(self, _friction):
        papi.dof__setCoulombFriction(self.wid, self.skid, self.id, _friction)

########################################
# Dof::Constraint Impulse functions
    def constraint_impulse(self, ):
        return papi.dof__getConstraintImpulse(self.wid, self.skid, self.id)

    def set_constraint_impulse(self, _friction):
        papi.dof__setConstraintImpulse(self.wid, self.skid, self.id, _friction)

    def __repr__(self):
        return '[Dof(%d): %s]' % (self.id, self.name)
