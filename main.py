import pandas as pd
import numpy as np
import gouveia
import belenguefread
from matplotlib import pyplot as plt
from gurobipy import *
from numpy import split
from tkinter import Tk
from tkinter.filedialog import askopenfilename

#popup dialog for file
Tk().withdraw()
filename = askopenfilename()

#read modelData from https://www.uv.es/~belengue/mcarp/
modelData = belenguefread.readFile(filename)

#translate to shoter varibles names
P = modelData['VEHICLES']
N = modelData['NODES']
W = modelData['CAPACITY']
DEPOT = modelData['DEPOT'] - 1
DUMPING_COST = modelData['DUMPING_COST']
ER = modelData['REQ_EDGES'] #required edges
if modelData['REQ_EDGES'] != 0:
    ER = modelData['REQ_EDGES']
else:
    ER = {}
ER2AR = dict() #dictionary to translates ER to required arcs
if modelData['NOREQ_EDGES'] != 0:
    NER = modelData['NOREQ_EDGES']
else:
    NER = {}
NER2NAR = dict() #dictionary to translates NER to non required arcs
if modelData['REQ_ARCS'] != 0:
    AR = modelData['REQ_ARCS']
else:
    AR = {}

if modelData['NOREQ_ARCS'] != 0:
    NAR = modelData['NOREQ_ARCS']
else:
    NAR = {}

#edges becomes 2 opposite arcs
for (i, j) in ER:
    ER2AR[(i, j)] = ER[(i, j)]
    ER2AR[(j, i)] = ER[(i, j)]

#edges becomes 2 opposite arcs
for (i, j) in NER:
    NER2NAR[(i, j)] = NER[(i, j)]
    NER2NAR[(j, i)] = NER[(i, j)]

R = AR.copy()
R.update(ER2AR.copy()) #all required arcs
NR = NAR.copy()
NR.update(NER2NAR.copy()) #all non required arcs
A = R.copy()
A.update(NR.copy()) #all arcs

QT = 0 #total demand cost
for (i, j) in AR:
    QT = QT + AR[(i, j)]['demand']
for (i, j) in ER:
    QT = QT + ER[(i, j)]['demand']

m, x, y, f = gouveia.F1R(P, N, W, QT, DEPOT, DUMPING_COST, R, A, AR, ER)
m.optimize()