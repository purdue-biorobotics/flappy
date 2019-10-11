# Author(s): Seungmoon Song <ssm0445@gmail.com>
import numpy as np


def fn_inv_f_vce0(f_vce0, K, N):
    if f_vce0 <= 1:
        v_ce0 = (f_vce0 - 1) / (K * f_vce0 + 1)
    elif f_vce0 > 1 and f_vce0 <= N:
        temp = (f_vce0 - N) / (f_vce0 - N + 1)
        v_ce0 = (temp + 1) / (1 - 7.56 * K * temp)
    else:  # elif f_vce0 > N:
        v_ce0 = .01 * (f_vce0 - N) + 1

    return v_ce0


def fn_f_lce0(l_ce0, w, c):
    f_lce0 = np.exp(c * np.abs((l_ce0 - 1) / (w))**3)
    return f_lce0

# f_p0 is for both f_se0 and f_pe0


def fn_f_p0(l0, e_ref):
    if l0 > 1:
        f_p0 = ((l0 - 1) / (e_ref))**2
    else:
        f_p0 = 0

    return f_p0

# for both f_be0


def fn_f_p0_ext(l0, e_ref, e_ref2):
    if l0 < e_ref2:
        f_p0 = ((l0 - e_ref2) / (e_ref))**2
    else:
        f_p0 = 0

    return f_p0
