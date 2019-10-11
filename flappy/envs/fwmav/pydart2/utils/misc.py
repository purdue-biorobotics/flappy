from __future__ import division
# from builtins import range
from past.utils import old_div
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group

import numpy as np


def S(x, precision=6):
    if x is None:
        return "None"
    fmt = "%%.%df" % precision
    if isinstance(x, float):
        return fmt % x
    ret = "["
    tokens = [fmt % v for v in x]
    ret += ", ".join(tokens)
    ret += "]"
    return ret


def split_by_sizes(array, sizes):
    cum_indices = np.cumsum(sizes)
    return np.split(array, cum_indices[:-1])


def norm2(x):
    return (np.linalg.norm(x) ** 2)


def penalize(x, lo, hi):
    if x < lo:
        return lo - x
    if hi < x:
        return x - hi
    return 0.0


def confine(x, lo, hi):
    if isinstance(x, float):
        return min(max(lo, x), hi)
    else:
        return np.array([confine(v, lo, hi) for v in x])


def vel(h, x0, x1, x2):
    if x0 is not None and x2 is not None:
        return old_div((x2 - x0), (2 * h))
    elif x0 is not None and x2 is None:
        return old_div((x1 - x0), h)
    elif x0 is None and x2 is not None:
        return old_div((x2 - x1), h)
    else:
        return np.zeros(x1.shape)


def acc(h, x0, x1, x2):
    if x0 is not None and x2 is not None:
        return old_div((x2 - 2 * x1 + x0), (h * h))
    else:
        return np.zeros(x1.shape)


def grad(fun, x, h):
    n = len(x)
    g = np.zeros(n)
    for i in range(n):
        dx = np.zeros(n)
        dx[i] = h
        f1 = fun(x - dx)
        f2 = fun(x + dx)
        g[i] = old_div((0.5 * f2 - 0.5 * f1), h)
    return g


def grad4(fun, x, h):
    n = len(x)
    g = np.zeros(n)
    for i in range(n):
        dx = np.zeros(n)
        dx[i] = h
        f0 = fun(x - 2.0 * dx)
        f1 = fun(x - 1.0 * dx)
        f2 = fun(x + 1.0 * dx)
        f3 = fun(x + 2.0 * dx)
        g[i] = old_div((1.0 / 12.0 * f0 - 2.0 / 3.0 * f1 +
                2.0 / 3.0 * f2 - 1.0 / 12.0 * f3), (h))
    return g


def deg2rad(x):
    return x / 180.0 * np.pi


def rad2deg(x):
    return x / np.pi * 180.0
