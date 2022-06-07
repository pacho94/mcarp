from datetime import datetime
import json

import gurobipy
from gurobipy import *
import numpy as np

from jenksnatural import *


def valueRCSorting(item):
    return item['Value'], -math.fabs(item['RC'])

def buildBuckets(bucketNum, items):
    buckets = [None] * bucketNum
    rc_array = [item['RC'] for item in items]
    centroids = get_jenks_breaks(rc_array, bucketNum)
    for item in items:
        for i in range(bucketNum):
            if centroids[i] <= item['RC'] < centroids[i + 1]:
                if not buckets[i]:
                    buckets[i] = []
                buckets[i].append(item)
    return buckets

def solveKernel(m, kernel, items, bestSolution, cutoffThreshold, kernelTimeLimit):
    # If a solution exist
    if bestSolution:
        # Set cut off constraint to make solution better
        if m.getAttr('ModelSense') == GRB.MINIMIZE:
            m.setParam(GRB.Param.Cutoff, bestSolution['ObjVal'] - cutoffThreshold)
        else:
            m.setParam(GRB.Param.Cutoff, bestSolution['ObjVal'] + cutoffThreshold)

    # Fix out of kernel variables
    for item in items:
        if item not in kernel:
            m.addConstr(m.getVarByName(item['Name']) == 0)
    m.update()
    # Set model time limit
    m.setParam(GRB.Param.TimeLimit, kernelTimeLimit)

    m.optimize()

    if json.loads(m.getJSONSolution())['SolutionInfo']['SolCount'] > 0:
        m.Params.SolutionNumber = 0
        m.write("bestSolution.sol")
        bestSolution = {'ObjVal': m.objVal, 'Vars': [{'Name': var.VarName, 'Value': var.X} for var in m.getVars()]}

    return bestSolution

def solveBucket(m, bucket, kernel, items, positiveThreshold, cutoffThreshold, bestSolution, bucketTimeLimit):
    # Init as empty list
    selectedItems = []

    # Fix out of kernel+bucket variables
    for item in items:
        if item not in kernel and item not in bucket:
            m.addConstr(m.getVarByName(item['Name']) == 0)

    # Select at least one element from the bucket
    m.addConstr(quicksum(m.getVarByName(item['Name']) for item in bucket) >= 1)

    # If a solution exist
    if bestSolution:
        # Set cut off constraint to make solution better
        if m.getAttr('ModelSense') == GRB.MINIMIZE:
            m.setParam(GRB.Param.Cutoff, bestSolution['ObjVal'] - cutoffThreshold)
        else:
            m.setParam(GRB.Param.Cutoff, bestSolution['ObjVal'] + cutoffThreshold)
        for item in bestSolution['Vars']:
            # Set up variable start value as best solution value
            m.getVarByName(item['Name']).Start = item['Value']

    # Set model time limit
    m.setParam(GRB.Param.TimeLimit, bucketTimeLimit)
    m.update()
    m.optimize()

    if json.loads(m.getJSONSolution())['SolutionInfo']['SolCount'] > 0:
        newSolution = True
        m.Params.SolutionNumber = 0
        m.write("bestSolution.sol")
        bestSolution = {'ObjVal': m.objVal, 'Vars': [{'Name': var.VarName, 'Value': var.X} for var in m.getVars()]}
        selectedItems = [item for item in bucket if m.getVarByName(item['Name']).X > positiveThreshold]

    return bestSolution, selectedItems, m.status

def startKernelSearch(m, varToUse, bucketInverseApprox, numIterations, positiveThreshold, cutoffThreshold,
                      timeLimit, kernelTimeLimit, bucketTimeLimit):
    # Save start time to impose computational time limit
    startTime = datetime.now()

    # Solve LP relaxation
    relaxedModel = m.copy().relax()
    relaxedModel.setParam(GRB.Param.TimeLimit, timeLimit)
    relaxedModel.optimize()

    bestSolution = []

    # Get variables value and RC
    items = [{'Name': var.VarName, 'Value': var.X, 'RC': var.RC, 'ImprRank': 0, 'InfRank': 0, 'DImprRank': 0} for var in relaxedModel.getVars() if var.VarName.__contains__(varToUse)]

    # Sort variables by means of their values (non-increasing) and RC (non-decreasing)
    items.sort(key=valueRCSorting, reverse=True)

    # Add to kernel the positive variables
    kernel = [item for item in items if item['Value'] > 0]

    bucketNum = bucketInverseApprox/np.var([item['RC'] for item in items if item not in kernel])
    # Build buckets with predefined bucket size
    buckets = buildBuckets(int(bucketNum), [item for item in items if item not in kernel])

    # Init maxBucketSize
    bucketMaxSize = len([item for item in items if item not in kernel])

    # How many times a new solution has been found
    improvementNumber = 0

    oldKernelSize = 0

    # Solve buckets
    for i in range(numIterations):
        # Solve kernel
        bestSolution = solveKernel(m.copy(), kernel, items, bestSolution, cutoffThreshold,
                                   np.min([timeLimit - (datetime.now() - startTime).total_seconds(), kernelTimeLimit]))
        if oldKernelSize == len(kernel):
            return
        oldKernelSize = len(kernel)
        currentAggregation = []
        for j in range(len(buckets)):
            if buckets[j]:
                currentAggregation = buckets[j].copy()
                for z in range(j, len(buckets)):
                    if buckets[z]:
                        if (datetime.now() - startTime).total_seconds() > timeLimit:
                            return
                        else:
                            bestSolution, selectedItems, modelStatus = solveBucket(m.copy(), currentAggregation, kernel, items,
                                                                                   positiveThreshold, cutoffThreshold, bestSolution,
                                                                                   np.min([timeLimit - (datetime.now() - startTime).total_seconds(), bucketTimeLimit]))
                        if selectedItems:
                            bucketMaxSize = (improvementNumber*bucketMaxSize + len(currentAggregation))/(improvementNumber + 1)
                            improvementNumber += 1
                            for item in selectedItems:
                                kernel.append(item)
                                for bucket in buckets:
                                    if bucket:
                                        if item in bucket:
                                            bucket.remove(item)
                            print("Improvement n° "+str(improvementNumber) + " Iteration " + str(i) + " Bucket " + str(j) +
                                  " CurrentAggr " + str(len(currentAggregation)) + " BucketMaxSize " + str(bucketMaxSize) +
                                  " KernelSize " + str(len(kernel)) + " ObjVal " + str(bestSolution['ObjVal']))
                            break

                        if len(currentAggregation) + len(buckets[z]) <= bucketMaxSize:
                            currentAggregation = currentAggregation + buckets[z].copy()
        # Sort variables by means of their values (non-increasing) and RC (non-decreasing)
        items.sort(key=valueRCSorting, reverse=True)
        # Build buckets with predefined bucket size
        bucketNum = bucketInverseApprox/np.var([item['RC'] for item in items if item not in kernel])
        buckets = buildBuckets(int(bucketNum), [item for item in items if item not in kernel])

    return datetime.now() - startTime
