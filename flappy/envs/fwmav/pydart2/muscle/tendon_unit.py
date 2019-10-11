# Author(s): Sehoon Ha <sehoon.ha@gmail.com>
#          : Seungmoon Song <ssm0445@gmail.com>
import numpy as np


class MusculoTendonUnit(object):
    """
    """
    # f_lce
    W = .56
    C = np.log(.05)

    # f_vce
    N = 1.5
    K = 5

    # f_pe
    E_REF_PE = W
    # f_be
    E_REF_BE = .5 * W
    E_REF_BE2 = 1 - W
    # f_se
    E_REF = .04

    # ECC
    TAU_ACT = .01  # [s]
    TAU_DACT = .04  # [s]

    # S range
    R_S = [.01, 1]

    def __init__(self,
                 F_MAX, L_OPT, V_MAX, L_SLACK,
                 L_MTU=None, A=0.01, DEL_T=0.02,
                 name=None, TIMESTEP=0.001):
        self.TIMESTEP = TIMESTEP
        self.F_MAX = F_MAX
        self.V_MAX = V_MAX

        self.L_OPT = L_OPT
        self.L_SLACK = L_SLACK
        self.L_MTU = L_MTU

        self.A = A

        # print("initialize: l_ce = %.4f" % self.l_ce)
        self.F_mtu = None
        self.reset()

        self.DEL_T = DEL_T
        self.name = name if name is not None else "MTU"

    def reset(self, ):
        L_OPT, L_SLACK, L_MTU = self.L_OPT, self.L_SLACK, self.L_MTU
        self.l_ce = L_OPT if L_MTU is None else L_MTU - L_SLACK
        self.a = self.A

    def update(self, s, l_mtu):
        from pydart2.muscle.model_geyer_2010 import fn_inv_f_vce0
        from pydart2.muscle.model_geyer_2010 import fn_f_lce0
        from pydart2.muscle.model_geyer_2010 import fn_f_p0
        from pydart2.muscle.model_geyer_2010 import fn_f_p0_ext
        MTU = MusculoTendonUnit

        # ECC
        self.a = self.fn_ECC(s)

        # update muscle state
        self.l_se = l_mtu - self.l_ce
        f_se0 = fn_f_p0(self.l_se / self.L_SLACK, MTU.E_REF)
        f_be0 = fn_f_p0_ext(self.l_ce / self.L_OPT,
                            MTU.E_REF_BE, MTU.E_REF_BE2)
        f_pe0 = fn_f_p0(self.l_ce / self.L_OPT, MTU.E_REF_PE)
        f_lce0 = fn_f_lce0(self.l_ce / self.L_OPT, MTU.W, MTU.C)
        f_vce0 = (f_se0 + f_be0) / (f_pe0 + self.a * f_lce0)
        # f_vce0 = (f_se0 + f_be0 - f_pe0)/(self.A*f_lce0)
        v_ce0 = fn_inv_f_vce0(f_vce0, MTU.K, MTU.N)

        self.v_ce = self.L_OPT * self.V_MAX * v_ce0
        self.l_ce = self.l_ce + self.TIMESTEP * self.v_ce
        self.F_mtu = self.F_MAX * f_se0

        # print("\n")
        # print("\ta = %.4f (s = %.4f)" % (self.a, s))
        # print("\tl_mtu = %.4f" % l_mtu)
        # print("\tl_ce = %.4f l_se = %.4f" % (self.l_ce, self.l_se))
        # print("\tl_se/L_SLACK = %.4f" % (self.l_se / self.L_SLACK))
        # print("\tf_se0 = %.4f" % f_se0)
        # print("\tF_mtu = %.4f" % self.F_mtu)
        # print("\tv_ce = %.4f" % self.v_ce)
        # print("\n")
        return self.F_mtu

    def fn_ECC(self, S):
        MTU = MusculoTendonUnit
        lo, hi = MTU.R_S
        S = np.minimum(hi, np.maximum(lo, S))
        A = self.a
        if S > A:
            tau = MTU.TAU_ACT
        else:
            tau = MTU.TAU_DACT

        dA = (S - A) / tau
        self.a = A + self.TIMESTEP * dA

        return self.a


if __name__ == '__main__':
    mtu = MusculoTendonUnit(F_MAX=15000, L_OPT=0.1, V_MAX=12, L_SLACK=0.4)
    for frame in range(10):
        mtu.update(0.9, 0.5)
        print("A = %.4f F_mtu = %.4f" % (mtu.a, mtu.F_mtu))
