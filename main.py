import sys
import pickle
from copy import deepcopy
from math import sqrt
import time

# different identifiers
FLY = "fly"
LOAD = "load"
DELIVER = "deliver"

POSITION = "position"
ORDER = "order"
WAREHOUSE = "warehouse"
EMPTY = "empty"
HAS_PRODUCT = "hasProduct"
CARRIES = "carries"
CLIENT = "client"
NOT_EMPTY = "not_empty"

LP = "listaPreconditiilor"
LA = "listaAdaugarilor"
LE = "listaEliminarilor"

# STRIPS representation sets
operatorDefinition = {\
	FLY : {\
		LP : (\
			(POSITION, 0),\
		),\
		LA : (\
			(POSITION, 1),\
		),\
		LE : (\
			(POSITION, 0),\
		)\
	},\
	LOAD : {\
		LP : (\
			(WAREHOUSE, 1),\
			(EMPTY,),\
			(HAS_PRODUCT, 1, 0),\
		),\
		LA : (\
			(CARRIES, 0),\
			(NOT_EMPTY,),\
		),\
		LE : (\
			(EMPTY,),\
		)\
	},\
	DELIVER : {\
		LP : (\
			(CARRIES, 0),\
			(CLIENT, 1),\
			(ORDER, 1, 0),\
			(NOT_EMPTY,),\
		),\
		LA : (\
			(EMPTY,),\
		),\
		LE : (\
			(CARRIES, 0),\
			(ORDER, 1, 0),\
			(NOT_EMPTY,),\
		)\
	}\
}


def checkFormulas(arguments, formulaTuple, state):
	"""
	The arguments are checked against the conditions in "formulaTuple" in
	context "state".
	"""
	for formula in formulaTuple:
		if len(formula) == 1 and formula[0] not in state:
			return False
		if len(formula) == 2\
			and (formula[0] not in state or arguments[formula[1]] not in state[formula[0]]):
			return False
		if len(formula) == 3\
			and (formula[0] not in state or (arguments[formula[1]], arguments[formula[2]]) not in state[formula[0]]):
			return False
	return True

def isEffectOf(operatorName, arguments, currentState):
	"""
	The set of post-conditions is built for the operator "operatorName" and
	it is verified that they are met in the current state "currentState"
	for "arguments".
	"""
	global operatorDefinition
	postConditions =\
		tuple(filter(lambda el: el not in operatorDefinition[operatorName][LE],\
			operatorDefinition[operatorName][LP])) + operatorDefinition[operatorName][LA]
	return checkFormulas(arguments, postConditions, currentState)

def applyReverseOperation(operatorName, arguments, currentState):
	"""
	The inverse of "operatorName" is applied on
	"currentState" with "arguments".
	"""
	newState = deepcopy(currentState)
	for t in operatorDefinition[operatorName][LA]:
		del newState[t[0]]
	for t in operatorDefinition[operatorName][LE]:
		newState[t[0]] = None if len(t) == 1 else [arguments[t[1]]] if len(t) == 2 else [(arguments[t[1]], arguments[t[2]])]
	return newState

def regression(currentState, path, currentCost, orders):
	"""
	The regression algorithm is applied and the drone's path is
	built. The function returns false if either there is no valid
	route or if the current time is greater than the best time
	found so far. Otherwise, it returns the actual path of the
	drone.
	"""
	global bestCost, initialPos, interval, starttime

	if currentCost >= bestCost or time.time() - starttime > interval:
		return False

	operations = []
	if EMPTY in currentState and currentState[POSITION][0] in currentState[WAREHOUSE]:
		# The drone has finished a delivery.
		for (clientCell, orderList) in orders.items():
			if orderList:
				operations.append( (FLY, (clientCell, currentState[POSITION][0])) )
	elif NOT_EMPTY in currentState and currentState[POSITION][0] in currentState[CLIENT]:
		# The drone has arrived at a client with a product.
		for a in filter(lambda el: (el, currentState[CARRIES][0]) in currentState[HAS_PRODUCT],\
			currentState[WAREHOUSE]):
			operations.append((FLY, (a, currentState[POSITION][0])))
	elif isEffectOf(DELIVER, (None, currentState[POSITION][0]), currentState):
		operations.append( (DELIVER, (None, currentState[POSITION][0])) )
	elif isEffectOf(LOAD, (currentState[CARRIES][0], currentState[POSITION][0]), currentState):
		operations.append( (LOAD, (currentState[CARRIES][0], currentState[POSITION][0])) )

	if not operations:
		return False

	if operations[0][0] == DELIVER:
		operations[0] = (DELIVER,(orders[currentState[POSITION][0]][0],currentState[POSITION][0]))
		orders[currentState[POSITION][0]].pop(0)

	# The first delivery is done.
	if not tuple(filter(lambda el: el[1], orders.items())):
		path.append(operations[0])
		order = operations[0][1][0]
		newState = applyReverseOperation(operations[0][0], operations[0][1], currentState)

		# Best route is computed.
		bestDist = 4294967295
		bestChoice = None
		for wId in filter(lambda el: (el, order) in newState[HAS_PRODUCT],\
			newState[WAREHOUSE]):
			d = sqrt(\
					pow(newState[POSITION][0][0] - wId[0], 2)\
					+ pow(newState[POSITION][0][1] - wId[1], 2))\
				+ sqrt(\
					pow(initialPos[0] - wId[0], 2)\
					+ pow(initialPos[1] - wId[1], 2)\
				)
			if d < bestDist:
				bestDist = d
				bestChoice = wId
		currentCost = currentCost + bestDist

		if currentCost >= bestCost:
			return False

		bestCost = currentCost
		path.append((FLY, (bestChoice, newState[POSITION][0])))
		path.append((LOAD,(order, bestChoice)))
		path.append((FLY, (initialPos, bestChoice)))
		return path

	# FLY operations are sorted.
	if len(operations) > 1:
		operations.sort(key=lambda op: sqrt( pow(op[1][0][0] - op[1][1][0], 2)\
			+  pow(op[1][0][1] - op[1][1][1], 2) ))

	# Iterates through all possible operators.
	bestPath = None
	for operation in operations:
		pathCopy = deepcopy(path)
		pathCopy.append(operation)
		rezPath = regression(applyReverseOperation(operation[0], operation[1], currentState),\
			pathCopy, currentCost if operation[0] != FLY else currentCost + sqrt(\
			pow(operation[1][0][0] - operation[1][1][0], 2) + pow(operation[1][0][1] - operation[1][1][1], 2)),\
			deepcopy(orders))
		if rezPath:
			bestPath = rezPath

	return bestPath

def make_plan(scenario):
    currentState = {\
    	HAS_PRODUCT	: scenario["available_products"],\
    	WAREHOUSE : scenario["warehouses"],\
    	CLIENT : scenario["clients"],\
    	EMPTY : None\
    }
    orders = {}
    for t in scenario["orders"]:
    	if t[0] in orders:
    		orders[t[0]].append(t[1])
    	else:
    		orders[t[0]] = [t[1]]

    global bestCost, initialPos, interval, starttime
    bestCost = 4294967295
    initialPos = scenario["initial_position"]
    starttime = time.time()
    interval = 30

    rez = None
    for order in scenario["orders"]:
    	newState = deepcopy(currentState)
    	newState[POSITION] =[order[0]]
    	newRez = regression(newState, [], 0, deepcopy(orders))
    	if newRez:
    		rez = newRez

    return list(\
    	map(\
    		lambda el: ("Fly"+str(el[1])).replace(" ","") if el[0] == FLY else\
    			"Load("+str(el[1][0])+")" if el[0] == LOAD else\
    			"Deliver("+str(el[1][0])+")",\
    		rez\
    	)\
    )

def main(args):
    # scenario = pickle.load(open('example.pkl', "rb"))
    # scenario = pickle.load(open('scenario1.pkl', "rb"))
    # scenario = pickle.load(open('scenario2.pkl', "rb"))
    # scenario = pickle.load(open('scenario3.pkl', "rb"))
    # scenario = pickle.load(open('scenario4.pkl', "rb"))
    # scenario = pickle.load(open('scenario5.pkl', "rb"))
    scenario = pickle.load(open('scenario6.pkl', "rb"))

    plan = make_plan(scenario)

    print(plan)

    # Se decomenteaza pentru a vedea si costul.
    # global bestCost
    # print(bestCost)

if __name__ == '__main__':
    main(sys.argv)
