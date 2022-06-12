
#retrieve tuple from (i,j) string
def fromParenthesisToIndex(indexedPar):
    indexedPar = indexedPar.replace("(", "")
    indexedPar = indexedPar.replace(")", "")
    indexedPar = indexedPar.split(",")
    i = int(indexedPar[0])
    j = int(indexedPar[1])
    return i, j

#read file from https://www.uv.es/~belengue/mcarp/
def readFile(filename):
    f = open(filename, "r")
    lines = f.readlines()
    data = dict()
    stop = False
    for i in range(len(lines)):
        if lines[i].find(":") != -1:
            splittedLine = lines[i].split(":")
            if splittedLine[1].isspace(): #if after : there is nothing is arc/edge definitin afterwards
                tag = splittedLine[0].replace("LIST_", "").strip() #delete LIST_ prefix
                datalen = data.get(tag) #get how many lines to read
                arcs = dict()
                for j in range(datalen):
                    splittedArcLine = lines[i+1+j].split() #split line over spaces
                    costs = dict()
                    for n in range(1, len(splittedArcLine), 2): #create dictionary with cost_name:value
                        if not splittedArcLine[n].isspace():
                            if splittedArcLine[n].strip() == 'trav_cost':
                                costs['cost'] = int(splittedArcLine[n+1].strip())
                            else:
                                costs[splittedArcLine[n].strip()] = int(splittedArcLine[n+1].strip())
                    ii, jj = fromParenthesisToIndex(splittedArcLine[0].strip())
                    arcs[(ii-1, jj-1)] = costs #add costs to arc in tuples fashion
                data[tag] = arcs #add all to tag name ex: REQ_ARCS
            else: #add to dictionary tag:value
                tag = splittedLine[0]
                tag = tag.strip()
                value = splittedLine[1].strip()
                if value.isdigit():
                    data[tag] = int(value)
    return data

