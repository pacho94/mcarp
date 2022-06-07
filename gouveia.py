from gurobipy import *
from numpy import split

def F1(m, P, N, W, DEPOT, DUMPING_COST, R, A, AR, ER, All=False):
    x, y, f = {}, {}, {}  #intialize the decision variables

    #binary variables if arc i,j served at trip p
    for (i, j) in R:
        for p in range(P):
            x[i, j, p] = m.addVar(vtype=GRB.BINARY, name="x%d,%d,%d" % (i, j, p))

    #integer variables y that denotes how many time arc traversed in trip p
    #continuos variables f are flow variables
    for (i, j) in A:
        for p in range(P):
            y[i, j, p] = m.addVar(vtype=GRB.INTEGER, name="y%d,%d,%d" % (i, j, p))
            f[i, j, p] = m.addVar(vtype=GRB.CONTINUOUS, name="f%d,%d,%d" % (i, j, p))
            m.addConstr(y[i, j, p] >= 0) #positivity constraints
            m.addConstr(f[i, j, p] >= 0)
    m.update()

    # (2) constraints that impose the continuity of trips at each node
    for ir in range(N):
        for p in range(P):
            m.addConstr((quicksum(y[i, j, p] for (i, j) in A if i == ir) +
                         quicksum(x[i, j, p] for (i, j) in R if i == ir)) ==
                        (quicksum(y[j, i, p] for (j, i) in A if i == ir) +
                         quicksum(x[j, i, p] for (j, i) in R if i == ir)))
    m.update()

    # (3) required arcs servicing constraints
    for (i, j) in AR:
        m.addConstr(quicksum(x[i, j, p] for p in range(P)) == 1)
    m.update()

    # (4) required edges servicing constraints
    for (i, j) in ER:
        m.addConstr(quicksum((x[i, j, p] + x[j, i, p]) for p in range(P)) == 1)
    m.update()

    # (5) constraints that implies that the dump cost is adequately charged in the objective function
    for p in range(P):
        m.addConstr((quicksum((x[i, j, p]) for (i, j) in R if i == DEPOT) +
                     quicksum((y[i, j, p]) for (i, j) in A if i == DEPOT)) <= 1)
    m.update()

    # (6) flow conservation constraints
    for ir in range(N):
        for p in range(P):
            m.addConstr((quicksum((f[j, i, p]) for (j, i) in A if i == ir and ir != DEPOT) -
                         quicksum((f[i, j, p]) for (i, j) in A if i == ir and ir != DEPOT)) ==
                        quicksum((R[(j, i)]['demand']*x[j, i, p]) for (j, i) in R if i == ir and ir != DEPOT))
    m.update()

    # (7) flow conservation constraints
    for p in range(P):
        m.addConstr(quicksum((f[i, j, p]) for (i, j) in A if i == DEPOT) ==
                    quicksum((R[(i, j)]['demand']*x[i, j, p]) for (i, j) in R))
    m.update()

    # (8) flow conservation constraints
    for p in range(P):
        m.addConstr(quicksum((f[i, j, p]) for (i, j) in A if j == DEPOT) ==
                    quicksum((R[(i, j)]['demand']*x[i, j, p]) for (i, j) in R if j == DEPOT))
    m.update()

    # (9)  linking constraints
    for (i, j) in A:
        for p in range(P):
            m.addConstr(f[i, j, p] <= W*(x[i, j, p] + y[i, j, p]))
    m.update()

    m.setObjective(quicksum(quicksum(x[i, j, p]*R[(i, j)]['serv_cost'] for (i, j) in R) +
                            quicksum(y[i, j, p]*R[(i, j)]['trav_cost'] for (i, j) in A) +
                            quicksum(y[i, j, p]*R[(i, j)]['trav_cost'] for (i, j) in A if j == DEPOT)*DUMPING_COST +
                            quicksum(x[i, j, p]*R[(i, j)]['trav_cost'] for (i, j) in R if j == DEPOT)*DUMPING_COST
                            for p in range(P)), GRB.MINIMIZE)
    m.update()
    if All:
        return m, x, y, f
    else:
        return m

def F1R(m, P, N, W, QT, DEPOT, DUMPING_COST, R, A, AR, ER, All=False):
    m, x, y, f = F1(m, P, N, W, DEPOT, DUMPING_COST, R, A, AR, ER, All=True)

    m.addConstr(quicksum(quicksum(y[i, j, p] for (i, j) in A if i == DEPOT) +
                         quicksum(x[i, j, p] for (i, j) in R if i == DEPOT) for p in range(P)) >= int(QT/W))
    m.update()

    for (i, j) in R:
        for p in range(P):
            m.addConstr(f[i, j, p] >= R[(i, j)]['demand']*x[i, j, p])
    m.update()

    for (i, j) in dict(set(A).difference(set(R))):
        for p in range(P):
            m.addConstr(f[i, j, p] >= y[i, j, p] - 1)
    m.update()

    for p in range(P - 1):
        m.addConstr((quicksum(y[i, j, p] for (i, j) in A if i == DEPOT) +
                     quicksum(x[i, j, p] for (i, j) in R if i == DEPOT)) >=
                    (quicksum(y[i, j, p + 1] for (i, j) in A if i == DEPOT) +
                     quicksum(x[i, j, p + 1] for (i, j) in R if i == DEPOT)))
    m.update()

    if All:
        return m, x, y, f
    else:
        return m

def F2(P, N, W, QT, DEPOT, DUMPING_COST, R, A, AR, ER, All=False):
    m = Model("MCARP")
    x, y, f = {}, {}, {}  #initialize decision variables
    x_a, y_a, f_a = {}, {}, {}  #initialize aggregated decision variables

    #binary variables if arc i,j served at trip p
    for (i, j) in R:
        for p in range(P):
            x[i, j, p] = m.addVar(vtype=GRB.BINARY, name="x%d,%d,%d" % (i, j, p))
        x_a[i, j] = quicksum(x[i, j, p] for p in range(P))
    m.update()

    #integer variables y that denotes how many time arc traversed in trip p
    #continuos variables f are flow variables
    for (i, j) in A:
        for p in range(P):
            y[i, j, p] = m.addVar(vtype=GRB.INTEGER, name="y%d,%d,%d" % (i, j, p))
            f[i, j, p] = m.addVar(vtype=GRB.CONTINUOUS, name="f%d,%d,%d" % (i, j, p))
        y_a[i, j] = quicksum(y[i, j, p] for p in range(P))
        f_a[i, j] = quicksum(f[i, j, p] for p in range(P))
        m.addConstr(y_a[i, j] >= 0) #positivity constraints
        m.addConstr(f_a[i, j] >= 0)
    m.update()

    # (2) constraints that impose the continuity of trips at each node
    for ir in range(N):
        m.addConstr((quicksum(y_a[i, j] for (i, j) in A if i == ir) +
                     quicksum(x_a[i, j] for (i, j) in R if i == ir)) ==
                    (quicksum(y_a[j, i] for (j, i) in A if i == ir) +
                     quicksum(x_a[j, i] for (j, i) in R if i == ir)))
    m.update()

    # (3) required arcs servicing constraints
    for (i, j) in AR:
        m.addConstr(x_a[i, j] == 1)
    m.update()

    # (4) required edges servicing constraints
    for (i, j) in ER:
        m.addConstr((x_a[i, j] + x_a[j, i]) == 1)

    # (5) constraints that implies that the dump cost is adequately charged in the objective function
    m.addConstr((quicksum((x_a[i, j]) for (i, j) in R if i == DEPOT) +
                 quicksum((y_a[i, j]) for (i, j) in A if i == DEPOT)) <= P)
    m.update()

    # (6) flow conservation constraints
    for ir in range(1, N):
        m.addConstr((quicksum((f_a[j, i]) for (j, i) in A if i == ir) -
                     quicksum((f_a[i, j]) for (i, j) in A if i == ir)) ==
                    quicksum((R[(j, i)]['demand']*x_a[j, i]) for (j, i) in R if i == ir))
    m.update()

    # (7) flow conservation constraints
    m.addConstr(quicksum((f_a[i, j]) for (i, j) in A if i == DEPOT) == QT)
    m.update()

    # (8) flow conservation constraints
    m.addConstr(quicksum((f_a[i, j]) for (i, j) in A if j == DEPOT) ==
                quicksum((R[(i, j)]['demand']*x_a[i, j]) for (i, j) in R if j == DEPOT))
    m.update()

    # (9)  linking constraints
    for (i, j) in A:
        m.addConstr(f_a[i, j] <= W*(x_a[i, j] + y_a[i, j]))
    m.update()

    m.setObjective(quicksum(x_a[i, j]*R[(i, j)]['serv_cost'] for (i, j) in R) +
                   quicksum(y_a[i, j]*R[(i, j)]['trav_cost'] for (i, j) in A) +
                   quicksum(y_a[i, j]*R[(i, j)]['trav_cost'] for (i, j) in A if j == DEPOT)*DUMPING_COST +
                   quicksum(x_a[i, j]*R[(i, j)]['trav_cost'] for (i, j) in R if j == DEPOT)*DUMPING_COST, GRB.MINIMIZE)
    m.update()
    if All:
        return m, x, y, f, x_a, y_a, f_a
    else:
        return m

def F2R(P, N, W, QT, DEPOT, DUMPING_COST, R, A, AR, ER):
    m, x, y, f, x_a, y_a, f_a = F2(P, N, W, QT, DEPOT, DUMPING_COST, R, A, AR, ER, All=True)

    m.addConstr(quicksum(y_a[i, j] for (i, j) in A if i == DEPOT) +
                quicksum(x_a[i, j] for (i, j) in R if i == DEPOT) >= int(QT/W))
    m.update()

    for (i, j) in R:
        m.addConstr(f_a[i, j] >= R[(i, j)]['demand']*x_a[i, j])
    m.update()

    for (i, j) in dict(set(A).difference(set(R))):
        m.addConstr(f_a[i, j] >= y_a[i, j] - P)
    m.update()

    return m
