from __future__ import print_function
from __future__ import absolute_import
from builtins import str
# from builtins import range
# from builtins import object
from six import string_types

# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group


import os.path
from . import pydart2_api as papi
import numpy as np
from .skel_vector import SkelVector

from .bodynode import BodyNode
from .dof import Dof
from .joint import create_joint
from .marker import Marker


class Skeleton(object):
    def __init__(self, _world, _filename=None,
                 _id=None, _friction=None, ):
        self.world = _world
        self.friction = _friction
        if _filename is not None:
            self.filename = os.path.realpath(_filename)
            self.id = papi.world__addSkeleton(self.world.id, self.filename)
        else:
            self.filename = None
            self.id = _id

        self.controller = None
        self.build()

    def build(self, ):
        # Initialize dofs
        _ndofs = papi.skeleton__getNumDofs(self.world.id, self.id)
        self.dofs = [Dof(self, i) for i in range(_ndofs)]
        self.name_to_dof = {dof.name: dof for dof in self.dofs}

        # Initialize joints
        _njoints = papi.skeleton__getNumJoints(self.world.id, self.id)
        self.joints = [create_joint(self, i) for i in range(_njoints)]
        self.name_to_joint = {joint.name: joint for joint in self.joints}

        # Initialize bodynodes
        _nbodynodes = papi.skeleton__getNumBodyNodes(self.world.id, self.id)
        self.bodynodes = [BodyNode(self, i)
                          for i in range(_nbodynodes)]
        self.name_to_body = {body.name: body for body in self.bodynodes}

        # Initialize markers
        _nmarkers = papi.skeleton__getNumMarkers(self.world.id, self.id)
        self.markers = [Marker(self, i) for i in range(_nmarkers)]

        for joint in self.joints:
            joint.build()

        for body in self.bodynodes:
            body.build()

        for marker in self.markers:
            marker.build()

    def set_root_joint_to_trans_and_euler(self, ):
        papi.skeleton__setRootJointToTransAndEuler(self.world.id,
                                                   self.id)
        self.build()

    def set_root_joint_to_weld(self, ):
        papi.skeleton__setRootJointToWeld(self.world.id,
                                          self.id)
        self.build()

    def set_controller(self, _controller):
        self.controller = _controller

    @property
    def name(self):
        return papi.skeleton__getName(self.world.id, self.id)

    def num_dofs(self):
        return len(self.dofs)

    @property
    def ndofs(self):
        return self.num_dofs()

    def num_joints(self):
        return len(self.joints)

    @property
    def njoints(self):
        return self.num_joints()

    def num_bodynodes(self):
        return len(self.bodynodes)

    @property
    def nbodies(self):
        return self.num_bodynodes()

    def is_mobile(self):
        return papi.skeleton__isMobile(self.world.id, self.id)

    def set_mobile(self, mobile):
        papi.skeleton__setMobile(self.world.id, self.id, mobile)

    def mass(self):
        return papi.skeleton__getMass(self.world.id, self.id)

    @property
    def m(self):
        return self.mass()

    def positions(self):
        q = papi.skeleton__getPositions(self.world.id, self.id, self.ndofs)
        # return q
        return SkelVector(q, self)

    @property
    def q(self):
        return self.positions()

    def set_positions(self, _q):
        papi.skeleton__setPositions(self.world.id, self.id, _q)

    @q.setter
    def q(self, _q):
        """ Setter also updates the internal skeleton kinematics """
        self.set_positions(_q)

    def velocities(self):
        qdot = papi.skeleton__getVelocities(self.world.id, self.id, self.ndofs)
        return SkelVector(qdot, self)

    @property
    def dq(self):
        return self.velocities()

    def set_velocities(self, _qdot):
        papi.skeleton__setVelocities(self.world.id, self.id, _qdot)

    @dq.setter
    def dq(self, _qdot):
        """ Setter also updates the internal skeleton kinematics """
        self.set_velocities(_qdot)

    def accelerations(self):
        ddq = papi.skeleton__getAccelerations(self.world.id,
                                              self.id,
                                              self.ndofs)
        return SkelVector(ddq, self)

    @property
    def ddq(self):
        return self.accelerations()

    def states(self):
        return np.concatenate((self.positions(), self.velocities()))

    @property
    def x(self):
        return np.concatenate((self.positions(), self.velocities()))

    def set_states(self, _x):
        self.set_positions(_x[:self.ndofs])
        self.set_velocities(_x[self.ndofs:])

    @x.setter
    def x(self, _x):
        self.set_states(_x)

    def position_differences(self, q1, q2):
        ret = papi.skeleton__getPositionDifferences(self.world.id, self.id,
                                                    q1, q2, self.ndofs)
        return ret

    def velocity_differences(self, dq1, dq2):
        ret = papi.skeleton__getVelocityDifferences(self.world.id, self.id,
                                                    dq1, dq2, self.ndofs)
        return ret

    def mass_matrix(self):
        M = np.zeros((self.ndofs, self.ndofs))
        papi.skeleton__getMassMatrix(self.world.id, self.id, M)
        return M

    @property
    def M(self):
        return self.mass_matrix()

    def coriolis_and_gravity_forces(self):
        return papi.skeleton__getCoriolisAndGravityForces(self.world.id,
                                                          self.id, self.ndofs)

    @property
    def c(self):
        return self.coriolis_and_gravity_forces()

    def self_collision_check(self):
        return papi.skeleton__getSelfCollisionCheck(self.world.id, self.id)

    def set_self_collision_check(self, _enable):
        papi.skeleton__setSelfCollisionCheck(self.world.id, self.id,
                                             _enable)

    def adjacent_body_check(self):
        return papi.skeleton__getAdjacentBodyCheck(self.world.id, self.id)

    def set_adjacent_body_check(self, _enable):
        papi.skeleton__setAdjacentBodyCheck(self.world.id, self.id,
                                            _enable)

    # def remove_all_collision_pairs(self):
    #     for b1 in self.bodies:
    #         for b2 in self.bodies:
    #             self.world.set_collision_pair(b1, b2, False)

    def constraint_forces(self):
        return papi.skeleton__getConstraintForces(self.world.id,
                                                  self.id, self.ndofs)

    def bodynode(self, query):
        if isinstance(query, string_types):
            return self.name_to_body[query]
        elif isinstance(query, int):
            return self.bodies[query]
        else:
            print('Cannot find body. query = %s' % str(query))
            return None

    def root_bodynodes(self, ):
        return [body for body in self.bodynodes
                if body.parent_bodynode is None]

    def root_bodynode(self, index=0):
        return self.root_bodynodes()[index]

    def bodynode_index(self, _name):
        return self.name_to_body[_name].id

    def body(self, query):
        if isinstance(query, string_types):
            return self.name_to_body[query]
        elif isinstance(query, int):
            return self.bodies[query]
        else:
            print('Cannot find body. query = %s' % str(query))
            return None

    def joint(self, query):
        if isinstance(query, string_types):
            return self.name_to_joint[query]
        elif isinstance(query, int):
            return self.joints[query]
        else:
            print('Cannot find joint. query = %s' % str(query))
            return None

    def dof(self, query):
        if isinstance(query, string_types):
            return self.name_to_dof[query]
        elif isinstance(query, int):
            return self.dofs[query]
        else:
            print('Cannot find dof. query = %s' % str(query))
            return None

    def dof_index(self, _name):
        return self.name_to_dof[_name].id

    def dof_indices(self, _names):
        return np.array([self.dof_index(n) for n in _names])

    def com(self):
        return papi.skeleton__getCOM(self.world.id, self.id)

    @property
    def C(self):
        return self.com()

    def com_velocity(self):
        return papi.skeleton__getCOMLinearVelocity(self.world.id, self.id)

    @property
    def dC(self):
        return self.com_velocity()

    def com_acceleration(self):
        return papi.skeleton__getCOMLinearAcceleration(self.world.id, self.id)

    @property
    def ddC(self):
        return self.com_acceleration()

    def linear_momentum(self):
        return self.Cdot * self.m

    @property
    def P(self):
        return self.linear_momentum()

    def forces(self):
        return self._tau

    @property
    def tau(self):
        return self.forces()

    def set_forces(self, _tau):
        self._tau = _tau
        papi.skeleton__setForces(self.world.id, self.id, _tau)

    @tau.setter
    def tau(self, _tau):
        self.set_forces(_tau)

    def position_lower_limits(self):
        return papi.skeleton__getPositionLowerLimits(self.world.id,
                                                     self.id, self.ndofs)

    def position_upper_limits(self):
        return papi.skeleton__getPositionUpperLimits(self.world.id,
                                                     self.id, self.ndofs)

    @property
    def q_lower(self):
        return self.position_lower_limits()

    @property
    def q_upper(self):
        return self.position_upper_limits()

    def force_lower_limits(self):
        return papi.skeleton__getForceLowerLimits(self.world.id,
                                                  self.id, self.ndofs)

    def force_upper_limits(self):
        return papi.skeleton__getForceUpperLimits(self.world.id,
                                                  self.id, self.ndofs)

    @property
    def tau_lower(self):
        return self.force_lower_limits()

    @property
    def tau_upper(self):
        return self.force_upper_limits()

    # def approx_inertia(self, axis):
    #     """Calculates the point-masses approximated inertia
    #     with respect to the given axis """
    #     axis = np.array(axis) / np.linalg.norm(axis)
    #     I = 0
    #     C = self.C
    #     for body in self.bodies:
    #         d = body.C - C
    #         # Subtract the distance along the axis
    #         r_sq = np.linalg.norm(d) ** 2 - np.linalg.norm(d.dot(axis)) ** 2
    #         I += body.m * r_sq
    #     return I

    # def approx_inertia_x(self):
    #     return self.approx_inertia([1, 0, 0])

    # def approx_inertia_y(self):
    #     return self.approx_inertia([0, 1, 0])

    # def approx_inertia_z(self):
    #     return self.approx_inertia([0, 0, 1])

    # def external_contacts_and_body_id(self):
    #     cid_cnt = dict()
    #     contacts = []
    #     for body in self.bodies:
    #         for c in body.contacts():
    #             contacts += [(c, body.id)]
    #             cid = int(c[6])
    #             if cid not in cid_cnt:
    #                 cid_cnt[cid] = 1
    #             else:
    #                 cid_cnt[cid] += 1
    #     return [(c, bid) for (c, bid) in contacts if cid_cnt[int(c[6])] < 2]

    # def contact_id_set(self):
    #     id_list = []
    #     for b in self.bodies:
    #         id_list += [c.i for c in b.contacts()]
    #     return set(id_list)

    # def contacted_bodies(self):
    #     return [body for body in self.bodies if body.num_contacts() > 0]

    # def world_cop(self):
    #     bodies = self.contacted_bodies()
    #     if len(bodies) == 0:
    #         return None
    #     pos_list = [b.C for b in bodies]
    #     avg = sum(pos_list) / len(pos_list)
    #     return avg

    # @property
    # def COP(self):
    #     return self.world_cop()

    # def contacted_body_names(self):
    #     return [body.name for body in self.contacted_bodies()]

    def render(self):
        papi.skeleton__render(self.world.id, self.id)

    def render_with_color(self, color):
        if len(color) == 3:
            color = np.concatenate([color, [1.0]])
        papi.skeleton__renderWithColor(self.world.id, self.id, color)

    # def render_markers(self):
    #     papi.renderSkeletonMarkers(self.world.id, self.id)

    def __repr__(self):
        return '[Skeleton(%d): %s]' % (self.id, self.name)
