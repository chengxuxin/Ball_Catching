import numpy as np
import pyomo.environ as pyo
import matplotlib.pyplot as plt
l = 0.23
d = 0.07
delta_T = 0.05
rotation_boundary = 80
acc_boundary = 12
ball_radius = 0.1
bot_radius = 0.176
# u[0] right; u[1] left


def Model(x, u):
    x_dot = np.zeros(3)
    u = (d/2)*np.array(u)
    x_dot[0] = 0.5*(u[0]+u[1])*np.cos(x[2])
    x_dot[1] = 0.5*(u[0]+u[1])*np.sin(x[2])
    x_dot[2] = (1/l)*(u[0]-u[1])
    return x_dot


def ModelUpdate(x, u, T_s):
    x_next = x + T_s*Model(x, u)
    return x_next

# <cylinder length="0.10938" radius="0.176"/>


def Optimalcontrol(z0, upre, xd, yd, T_lower, T_upper):
    '''
    '''
    # delta_T = 0.02
    # acc_boundary = acc_boundary / 2
    N = int(T_upper/delta_T)
    M = int(T_lower/delta_T)
    print(f'N: {N}')
    print(f'M: {M}')

    nx = 3         # number of states
    nu = 2         # number of inputs
    model = pyo.ConcreteModel()
    # length of finite optimization problem
    model.tidx = pyo.Set(initialize=range(0, N+1))
    model.xidx = pyo.Set(initialize=range(0, nx))
    model.uidx = pyo.Set(initialize=range(0, nu))

    # Create state and input variables trajectory:
    model.z = pyo.Var(model.xidx, model.tidx)
    model.u = pyo.Var(model.uidx, model.tidx)
    # object
    model.cost = pyo.Objective(expr=sum(500*(model.z[0, t]-xd)**2 + 500*(model.z[1, t]-yd)**2 + (
        model.u[0, t]**2) + (model.u[1, t]**2) for t in model.tidx if t < N), sense=pyo.minimize)
    # init
    model.constraintinit1 = pyo.Constraint(
        model.xidx, rule=lambda model, i: model.z[i, 0] == z0[i])
    model.constraintinit2 = pyo.Constraint(
        model.uidx, rule=lambda model, i: model.u[i, 0]-upre[i] <= acc_boundary)
    model.constraintinit3 = pyo.Constraint(
        model.uidx, rule=lambda model, i: model.u[i, 0]-upre[i] >= -acc_boundary)
    # final
    model.constraintfinal1 = pyo.Constraint(
        model.xidx, rule=lambda model, i: model.z[0, N] == xd)
    model.constraintfinal2 = pyo.Constraint(
        model.xidx, rule=lambda model, i: model.z[1, N] == yd)
    # model
    model.constraintmodel1 = pyo.Constraint(model.tidx, rule=lambda model, t: model.z[0, t+1] == model.z[0, t]+delta_T*(
        (d/2)*0.5*(model.u[0, t]+model.u[1, t])*pyo.cos(model.z[2, t])) if t < N else pyo.Constraint.Skip)
    model.constraintmodel2 = pyo.Constraint(model.tidx, rule=lambda model, t: model.z[1, t+1] == model.z[1, t]+delta_T*(
        (d/2)*0.5*(model.u[0, t]+model.u[1, t])*pyo.sin(model.z[2, t])) if t < N else pyo.Constraint.Skip)
    model.constraintmodel3 = pyo.Constraint(model.tidx, rule=lambda model, t: model.z[2, t+1] == model.z[2, t]+delta_T*(
        (d/2)*(1/l)*(model.u[0, t]-model.u[1, t])) if t < N else pyo.Constraint.Skip)
    # boundary
    model.constraintboundary1 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t] <= rotation_boundary
                                                                                       if t <= N-1 else pyo.Constraint.Skip)
    model.constraintboundary2 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t] >= -rotation_boundary
                                                                                       if t <= N-1 else pyo.Constraint.Skip)
    model.constraintboundary3 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t] <= rotation_boundary
                                                                                       if t <= N-1 else pyo.Constraint.Skip)
    model.constraintboundary4 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t] >= -rotation_boundary
                                                                                       if t <= N-1 else pyo.Constraint.Skip)
    # acc boundary
    model.constraintboundary5 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t+1]-model.u[0, t] <= acc_boundary
                                               if t < N-1 else pyo.Constraint.Skip)
    model.constraintboundary6 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t+1]-model.u[0, t] >= -acc_boundary
                                               if t < N-1 else pyo.Constraint.Skip)
    model.constraintboundary7 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t+1]-model.u[1, t] <= acc_boundary
                                               if t < N-1 else pyo.Constraint.Skip)
    model.constraintboundary8 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t+1]-model.u[1, t] >= -acc_boundary
                                               if t < N-1 else pyo.Constraint.Skip)
    model.constraint16 = pyo.Constraint(
        model.uidx, rule=lambda model, i: model.u[0, N-1] == 0)
    model.constraint17 = pyo.Constraint(
        model.uidx, rule=lambda model, i: model.u[1, N-1] == 0)
    # new constraint for detecting catchable
    model.contstrain18 = pyo.Constraint(model.tidx, rule=lambda model, t: (model.z[0, t] - xd)**2 + (
        model.z[1, t] - yd)**2 <= (ball_radius + bot_radius)**2 if t > 0 and t <= M else pyo.Constraint.Skip)
    solver = pyo.SolverFactory('ipopt')
    results = solver.solve(model, tee=False)
    z1 = [pyo.value(model.z[0, 0])]
    z2 = [pyo.value(model.z[1, 0])]
    z3 = [pyo.value(model.z[2, 0])]
    u1 = [pyo.value(model.u[0, 0])]
    u2 = [pyo.value(model.u[1, 0])]

    for t in model.tidx:
        if t < N:
            z1.append(pyo.value(model.z[0, t+1]))
            z2.append(pyo.value(model.z[1, t+1]))
            z3.append(pyo.value(model.z[2, t+1]))
        if t < N-1:
            u1.append(pyo.value(model.u[0, t+1]))
            u2.append(pyo.value(model.u[1, t+1]))

    # plt.figure(1)
    # plt.plot(z1, z2,'b')
    # plt.figure(2)
    # plt.plot( z3,'k')
    # plt.plot(u1,'r')
    # plt.plot(u2,'b')
    # plt.figure(3)
    # plt.plot(z1,'k',label='z1')
    # plt.plot(z2,'r',label='z2')
    # plt.legend()
    # plt.show()

    if str(results.solver.termination_condition) == "optimal":
        feas = True
        return feas, u1, u2
    else:
        feas = False
        return feas, u1, u2

# v2.0


def Optimalcontrol_approach(z0, upre, xd, yd, horizon):
    N = int(horizon/delta_T)
    nx = 3         # number of states
    nu = 2         # number of inputs
    model = pyo.ConcreteModel()
    # length of finite optimization problem
    model.tidx = pyo.Set(initialize=range(0, N+1))
    model.xidx = pyo.Set(initialize=range(0, nx))
    model.uidx = pyo.Set(initialize=range(0, nu))

    # Create state and input variables trajectory:
    model.z = pyo.Var(model.xidx, model.tidx)
    model.u = pyo.Var(model.uidx, model.tidx)
    # object
    model.cost = pyo.Objective(expr=sum(500*(model.z[0, t]-xd)**2 + 500*(
        model.z[1, t]-yd)**2 for t in model.tidx if t < N), sense=pyo.minimize)
    # init
    model.constraintinit1 = pyo.Constraint(
        model.xidx, rule=lambda model, i: model.z[i, 0] == z0[i])
    model.constraintinit2 = pyo.Constraint(
        model.uidx, rule=lambda model, i: model.u[i, 0]-upre[i] <= acc_boundary)
    model.constraintinit3 = pyo.Constraint(
        model.uidx, rule=lambda model, i: model.u[i, 0]-upre[i] >= -acc_boundary)
    # final
    # final state has to be on the desired coordinate
    # model.constraintfinal1 = pyo.Constraint(
    #     model.xidx, rule=lambda model, i: model.z[0, N] == xd)
    # model.constraintfinal2 = pyo.Constraint(
    #     model.xidx, rule=lambda model, i: model.z[1, N] == yd)
    # model
    model.constraintmodel1 = pyo.Constraint(model.tidx, rule=lambda model, t: model.z[0, t+1] == model.z[0, t]+delta_T*(
        (d/2)*0.5*(model.u[0, t]+model.u[1, t])*pyo.cos(model.z[2, t])) if t < N else pyo.Constraint.Skip)
    model.constraintmodel2 = pyo.Constraint(model.tidx, rule=lambda model, t: model.z[1, t+1] == model.z[1, t]+delta_T*(
        (d/2)*0.5*(model.u[0, t]+model.u[1, t])*pyo.sin(model.z[2, t])) if t < N else pyo.Constraint.Skip)
    model.constraintmodel3 = pyo.Constraint(model.tidx, rule=lambda model, t: model.z[2, t+1] == model.z[2, t]+delta_T*(
        (d/2)*(1/l)*(model.u[0, t]-model.u[1, t])) if t < N else pyo.Constraint.Skip)
    # boundary
    model.constraintboundary1 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t] <= rotation_boundary
                                               if t <= N-1 else pyo.Constraint.Skip)
    model.constraintboundary2 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t] >= -rotation_boundary
                                               if t <= N-1 else pyo.Constraint.Skip)
    model.constraintboundary3 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t] <= rotation_boundary
                                               if t <= N-1 else pyo.Constraint.Skip)
    model.constraintboundary4 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t] >= -rotation_boundary
                                               if t <= N-1 else pyo.Constraint.Skip)
    # acc boundary
    model.constraintboundary5 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t+1]-model.u[0, t] <= acc_boundary
                                               if t < N-1 else pyo.Constraint.Skip)
    model.constraintboundary6 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t+1]-model.u[0, t] >= -acc_boundary
                                               if t < N-1 else pyo.Constraint.Skip)
    model.constraintboundary7 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t+1]-model.u[1, t] <= acc_boundary
                                               if t < N-1 else pyo.Constraint.Skip)
    model.constraintboundary8 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[1, t+1]-model.u[1, t] >= -acc_boundary
                                               if t < N-1 else pyo.Constraint.Skip)
    # final velocity has to be zero
    # model.constraint16 = pyo.Constraint(
    #     model.uidx, rule=lambda model, i: model.u[0, N-1] == 0)
    # model.constraint17 = pyo.Constraint(
    #     model.uidx, rule=lambda model, i: model.u[1, N-1] == 0)
    solver = pyo.SolverFactory('ipopt')
    results = solver.solve(model)
    z1 = [pyo.value(model.z[0, 0])]
    z2 = [pyo.value(model.z[1, 0])]
    z3 = [pyo.value(model.z[2, 0])]
    u1 = [pyo.value(model.u[0, 0])]
    u2 = [pyo.value(model.u[1, 0])]

    for t in model.tidx:
        if t < N:
            z1.append(pyo.value(model.z[0, t+1]))
            z2.append(pyo.value(model.z[1, t+1]))
            z3.append(pyo.value(model.z[2, t+1]))
        if t < N-1:
            u1.append(pyo.value(model.u[0, t+1]))
            u2.append(pyo.value(model.u[1, t+1]))

    # plt.figure(1)
    # plt.plot(z1, z2,'b')
    # plt.figure(2)
    # plt.plot( z3,'k')
    # plt.plot(u1,'r')
    # plt.plot(u2,'b')
    # plt.figure(3)
    # plt.plot(z1,'k',label='z1')
    # plt.plot(z2,'r',label='z2')
    # plt.legend()
    # plt.show()

    if str(results.solver.termination_condition) == "optimal":
        feas = True
        return feas, u1, u2
    else:
        feas = False
        return feas, u1, u2
