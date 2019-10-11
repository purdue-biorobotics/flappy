# Author(s): Sehoon Ha <sehoon.ha@gmail.com>
#          : Seungmoon Song <ssm0445@gmail.com>
import numpy as np
from pydart2.utils.misc import S
import pydart2.utils.transformations as trans
from itertools import tee


class AttachmentPoint(object):
    """
    """
    def __init__(self, bodyname, offset):
        self.bodyname = bodyname
        self.skeleton = None
        self.body = None
        self.offset = np.array(offset)

    def is_initialized(self, ):
        return (self.body is not None)

    def initialize(self, skeleton):
        self.skeleton = skeleton
        self.body = skeleton.body(self.bodyname)

    def to_world(self, ):
        return self.body.to_world(self.offset)

    def __str__(self, ):
        return "(%s, %s)" % (self.bodyname, S(self.offset, 3))


class Route(object):
    """
    route = Route([("Upper", [0.0, 0.2, 0.0]), ("Lower", [0.0, 0.2, 0.0])])
    """
    def __init__(self, points=None):
        if points is None:
            self.points = []
        else:
            self.points = [AttachmentPoint(name, offset)
                           for name, offset in points]

        self.world_points = None
        self.world_directions = None
        self.length = None

    def num_points(self, ):
        return len(self.points)

    def __len__(self, ):
        return self.num_points()

    def add_point(self, bodyname, offset):
        pt = AttachmentPoint(bodyname=bodyname,
                             offset=offset)
        self.points.append(pt)

    def initialize_points(self, skeleton):
        for pt in self.points:
            pt.initialize(skeleton)
        self.local_points = [pt.offset for pt in self.points]
        self.bodynodes = [pt.body for pt in self.points]
        self.update_geometry_variables()

    def update_geometry_variables(self, ):
        self.world_points = [pt.to_world() for pt in self.points]
        self.length = 0.0
        self.world_directions = list()

        pt0 = self.world_points[0]
        for pt1 in self.world_points[1:]:
            diff = pt1 - pt0
            length = np.linalg.norm(diff)
            direction = diff / length

            self.length += length
            self.world_directions.append(direction)

            pt0 = pt1

    def local_points_as_pair(self, ):
        "s -> (offset0, offset1), (offset1, offset2), ..."
        a, b = tee(self.local_points)
        next(b, None)
        return zip(a, b)

    def bodynodes_as_pair(self, ):
        "s -> (body0, body1), (body1, body2), ..."
        a, b = tee(self.bodynodes)
        next(b, None)
        return zip(a, b)

    def render_with_ri(self, ri, ):
        if self.num_points() < 2:
            return
        ri.set_line_width(3)
        world_points = [pt.to_world() for pt in self.points]
        ri.render_lines(world_points)

    def __repr__(self, ):
        tokens = [str(pt) for pt in self.points]
        return "[%s: length = %.4f]" % (", ".join(tokens), self.length)
