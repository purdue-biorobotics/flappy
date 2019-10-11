from __future__ import absolute_import
# from builtins import range
# from builtins import object
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
from . import pydart2_api as papi


class Joint(object):
    """
    """
    FORCE, PASSIVE, SERVO, ACCELERATION, VELOCITY, LOCKED = list(range(6))

    def __init__(self, _skeleton, _id):
        """
        """
        self.skeleton = _skeleton
        self.id = _id
        self.name = papi.joint__getName(self.wid, self.skid, self.id)
        self.parent_bodynode = None
        self.child_bodynode = None
        self.dofs = list()

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
        skel = self.skel
        self.parent_bodynode = None
        self.child_bodynode = None

        parent_id = self.parent_body_node_id()
        if parent_id >= 0:
            self.parent_bodynode = skel.bodynodes[
                self.parent_body_node_id()]

        child_id = self.child_body_node_id()
        if child_id >= 0:
            self.child_bodynode = skel.bodynodes[
                self.child_body_node_id()]

        self.dofs = list()
        _ndofs = papi.joint__getNumDofs(self.wid, self.skid, self.id)
        for i in range(_ndofs):
            id = papi.joint__getDof(self.wid, self.skid, self.id, i)
            self.dofs.append(skel.dofs[id])
            skel.dofs[id].joint = self

########################################
# Joint::Property Functions
    def set_name(self, _name, _renameDofs):
        return papi.joint__setName(self.wid,
                                   self.skid,
                                   self.id,
                                   _name,
                                   _renameDofs)

    def is_kinematic(self, ):
        return papi.joint__isKinematic(self.wid, self.skid, self.id)

    def is_dynamic(self, ):
        return papi.joint__isDynamic(self.wid, self.skid, self.id)

    def type(self, ):
        return papi.joint__getType(self.wid, self.skid, self.id)

    def set_actuator_type(self, actuator_type):
        """
        joint.set_actuator_type(Joint.LOCKED)
        """
        papi.joint__setActuatorType(
            self.wid, self.skid, self.id, actuator_type)

    def actuator_type(self, ):
        return papi.joint__getActuatorType(self.wid, self.skid, self.id)

########################################
# Joint::Parent and child functions
    def position_in_world_frame(self, ):
        T0 = self.child_bodynode.transform()
        T1 = self.transform_from_child_body_node()
        T = T0.dot(T1)
        return T[:3, 3]

    def parent_body_node_id(self, ):
        return papi.joint__getParentBodyNode(self.wid, self.skid, self.id)

    def child_body_node_id(self, ):
        return papi.joint__getChildBodyNode(self.wid, self.skid, self.id)

    def set_transform_from_parent_body_node(self, T):
        papi.joint__setTransformFromParentBodyNode(self.wid,
                                                   self.skid,
                                                   self.id,
                                                   T)

    def set_transform_from_child_body_node(self, T):
        papi.joint__setTransformFromChildBodyNode(self.wid,
                                                  self.skid,
                                                  self.id,
                                                  T)

    def transform_from_parent_body_node(self, ):
        return papi.joint__getTransformFromParentBodyNode(self.wid,
                                                          self.skid,
                                                          self.id)

    def transform_from_child_body_node(self, ):
        return papi.joint__getTransformFromChildBodyNode(self.wid,
                                                         self.skid,
                                                         self.id)


########################################
# Joint::Limit functions
    def set_position_limit_enforced(self, _isPositionLimitEnforced=True):
        papi.joint__setPositionLimitEnforced(self.wid,
                                             self.skid,
                                             self.id,
                                             _isPositionLimitEnforced)

    def is_position_limit_enforced(self, ):
        return papi.joint__isPositionLimitEnforced(self.wid,
                                                   self.skid,
                                                   self.id)

    def has_position_limit(self, _index):
        return papi.joint__hasPositionLimit(self.wid,
                                            self.skid,
                                            self.id,
                                            _index)

    def position_lower_limit(self, _index):
        return papi.joint__getPositionLowerLimit(self.wid,
                                                 self.skid,
                                                 self.id,
                                                 _index)

    def set_position_lower_limit(self, _index, _position):
        papi.joint__setPositionLowerLimit(self.wid,
                                          self.skid,
                                          self.id,
                                          _index,
                                          _position)

    def position_upper_limit(self, _index):
        return papi.joint__getPositionUpperLimit(self.wid,
                                                 self.skid,
                                                 self.id,
                                                 _index)

    def set_position_upper_limit(self, _index, _position):
        papi.joint__setPositionUpperLimit(self.wid,
                                          self.skid,
                                          self.id,
                                          _index,
                                          _position)

########################################
# Joint::Dof functions
    def num_dofs(self, ):
        return len(self.dofs)

########################################
# Joint::Passive Force functions
    def spring_stiffness(self, _index):
        return papi.joint__getSpringStiffness(self.wid,
                                              self.skid,
                                              self.id,
                                              _index)

    def set_spring_stiffness(self, _index, _k):
        papi.joint__setSpringStiffness(self.wid,
                                       self.skid,
                                       self.id,
                                       _index,
                                       _k)

    def rest_position(self, _index):
        return papi.joint__getRestPosition(self.wid,
                                           self.skid,
                                           self.id,
                                           _index)

    def set_rest_position(self, _index, _q0):
        papi.joint__setRestPosition(self.wid, self.skid, self.id,
                                    _index,
                                    _q0)

    def damping_coefficient(self, _index):
        return papi.joint__getDampingCoefficient(self.wid,
                                                 self.skid,
                                                 self.id,
                                                 _index)

    def set_damping_coefficient(self, _index, _coeff):
        papi.joint__setDampingCoefficient(self.wid,
                                          self.skid,
                                          self.id,
                                          _index,
                                          _coeff)

    def coulomb_friction(self, _index):
        return papi.joint__getCoulombFriction(self.wid,
                                              self.skid,
                                              self.id,
                                              _index)

    def set_coulomb_friction(self, _index, _friction):
        papi.joint__setCoulombFriction(self.wid,
                                       self.skid,
                                       self.id,
                                       _index,
                                       _friction)

    def __repr__(self):
        return '[Joint(%d): %s]' % (self.id, self.name)


class WeldJoint(Joint):

    def __init__(self, _skeleton, _id):
        Joint.__init__(self, _skeleton, _id)

    def __repr__(self):
        return '[WeldJoint(%d): %s]' % (self.id, self.name)


class RevoluteJoint(Joint):

    def __init__(self, _skeleton, _id):
        Joint.__init__(self, _skeleton, _id)

    def axis(self, ):
        return papi.revolute_joint__getAxis(self.wid, self.skid, self.id)

    def axis_in_world_frame(self, ):
        # if self.parent_bodynode:
        #     R = self.parent_bodynode.T[:3, :3]
        #     return R.dot(self.axis())
        if self.child_bodynode:
            R0 = self.child_bodynode.T[:3, :3]
            R1 = self.transform_from_child_body_node()[:3, :3]
            R = R0.dot(R1)
            return R.dot(self.axis())
        else:
            return self.axis()

    def set_axis(self, _axis):
        return papi.revolute_joint__setAxis(
            self.wid, self.skid, self.id, _axis)

    def __repr__(self):
        return '[RevoluteJoint(%d): %s]' % (self.id, self.name)


class PrismaticJoint(Joint):

    def __init__(self, _skeleton, _id):
        Joint.__init__(self, _skeleton, _id)

    def axis(self, ):
        return papi.prismatic_joint__getAxis(self.wid, self.skid, self.id)

    def set_axis(self, _axis):
        return papi.prismatic_joint__setAxis(
            self.wid, self.skid, self.id, _axis)

    def __repr__(self):
        return '[PrismaticJoint(%d): %s]' % (self.id, self.name)


class UniversalJoint(Joint):

    def __init__(self, _skeleton, _id):
        Joint.__init__(self, _skeleton, _id)

    def axis1(self, ):
        return papi.universal_joint__getAxis1(self.wid, self.skid, self.id)

    def axis1_in_world_frame(self, ):
        R = self.parent_bodynode.T[:3, :3]
        return R.dot(self.axis1())

    def set_axis1(self, _axis):
        return papi.universal_joint__setAxis1(
            self.wid, self.skid, self.id, _axis)

    def axis2_in_world_frame(self, ):
        R = self.parent_bodynode.T[:3, :3]
        return R.dot(self.axis2())

    def axis2(self, ):
        return papi.universal_joint__getAxis2(self.wid, self.skid, self.id)

    def set_axis2(self, _axis):
        return papi.universal_joint__setAxis2(
            self.wid, self.skid, self.id, _axis)

    def __repr__(self):
        return '[UniversalJoint(%d): %s]' % (self.id, self.name)


class PlanarJoint(Joint):

    def __init__(self, _skeleton, _id):
        Joint.__init__(self, _skeleton, _id)

    def __repr__(self):
        return '[PlanarJoint(%d): %s]' % (self.id, self.name)


class ScrewJoint(Joint):

    def __init__(self, _skeleton, _id):
        Joint.__init__(self, _skeleton, _id)

    def __repr__(self):
        return '[ScrewJoint(%d): %s]' % (self.id, self.name)


class BallJoint(Joint):

    def __init__(self, _skeleton, _id):
        Joint.__init__(self, _skeleton, _id)

    def __repr__(self):
        return '[BallJoint(%d): %s]' % (self.id, self.name)


class TranslationalJoint(Joint):

    def __init__(self, _skeleton, _id):
        Joint.__init__(self, _skeleton, _id)

    def __repr__(self):
        return '[TranslationalJoint(%d): %s]' % (self.id, self.name)


class EulerJoint(Joint):

    def __init__(self, _skeleton, _id):
        Joint.__init__(self, _skeleton, _id)

    def axis_order(self, ):
        return papi.euler_joint__getAxisOrder(self.wid, self.skid, self.id)

    def set_axis_order(self, _axis_order):
        return papi.euler_joint__setAxisOrder(
            self.wid, self.skid, self.id, _axis_order)

    def __repr__(self):
        return '[EulerJoint(%d): %s]' % (self.id, self.name)


class FreeJoint(Joint):

    def __init__(self, _skeleton, _id):
        Joint.__init__(self, _skeleton, _id)

    def __repr__(self):
        return '[FreeJoint(%d): %s]' % (self.id, self.name)


def create_joint(_skeleton, _id):
    wid = _skeleton.world.id
    skid = _skeleton.id
    id = _id
    type_str = papi.joint__getType(wid, skid, id)

    type_str_to_class = dict()
    type_str_to_class['WeldJoint'] = WeldJoint
    type_str_to_class['RevoluteJoint'] = RevoluteJoint
    type_str_to_class['PrismaticJoint'] = PrismaticJoint
    type_str_to_class['UniversalJoint'] = UniversalJoint
    type_str_to_class['PlanarJoint'] = PlanarJoint
    type_str_to_class['ScrewJoint'] = ScrewJoint
    type_str_to_class['BallJoint'] = BallJoint
    type_str_to_class['TranslationalJoint'] = TranslationalJoint
    type_str_to_class['EulerJoint'] = EulerJoint
    type_str_to_class['FreeJoint'] = FreeJoint

    cls = type_str_to_class.get(type_str, None)
    if cls is None:
        print("Invalid type string: %s" % type_str)
        return None
    else:
        return cls(_skeleton, _id)
