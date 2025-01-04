import sympy
from sympy import Symbol, Eq, sin, pi, solve


fx = Symbol("Fx")
fy = Symbol("Fy")
omega_dot = Symbol("omega_dot")
r = Symbol("r")
l = Symbol("l")
t1 = Symbol("tau1")
t2 = Symbol("tau2")
t3 = Symbol("tau3")

eq1 = Eq(fx, (-sin(pi / 3) * t1 + sin(pi / 3) * t3) / r)
eq2 = Eq(fy, (t1 / 2 - t2 + t3 / 2) / r)
eq3 = Eq(omega_dot, l / r * (t1 + t2 + t3))

ret = solve([eq1, eq2, eq3], (t1, t2, t3))
print(ret)
