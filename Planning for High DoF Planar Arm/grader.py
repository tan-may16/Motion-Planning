import numpy as np
import pdb
import argparse
import subprocess # For executing c++ executable
import pandas as pd
from timeit import default_timer as timer

###############################################################
################### Util Functions Below ######################

def convertPIs(aString):
    """ Input: A comma seperated string like "pi/2,pi/4,pi/2,pi/4,pi/2,"
            or 1.2,4,5.3 etc
    Output: string replacing the pis 1.57079,...,...
    """
    if aString[-1] == ",":  # Remove training comma if there is one
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592") # Replace pi with 3.14... if needed
    vecOfStrings = aString.split(",")
    ans = []
    for anExpression in vecOfStrings:
        ans.append(str(eval(anExpression))) # Evaluate expressions if needed
    return ans

###############################################################
################### Main Functions Below ######################

# ["./map2.txt", "0.392699,2.356194,3.141592",
#                                 "1.570796,0.785398,1.570796"]
def graderMain(executablePath, gradingCSV):
    problems = [["./map2.txt", "1.61198,0.0667933,6.15022",
                                         "pi/2,pi*2/3,pi"],
                # ["./map2.txt", "1.665762, 2.122711, 2.159336, 5.825278, 0.346115",
                #  "0.442375, 2.669400, 1.684554, 3.232004, 2.894518"],  
                # ["./map2.txt", "1.426454, 0.953590, 0.357237, 3.524812, 0.743812",
                #  "1.010924, 2.990779, 0.700475, 3.774283, 0.197890"], 
                # ["./map2.txt", "1.509483, 2.712544, 5.506200, 1.697402, 0.324063",
                #  "0.316585, 2.262115, 1.549368, 6.278967, 2.685507"], 
                # ["./map2.txt", "1.510058, 1.380625, 2.230860, 1.150521, 0.835278",
                #  "1.691074, 2.297973, 5.233143, 1.019553, 2.999407"], 
                
            ]
    
    # [["./map2.txt", "1.715043, 0.760686, 1.567393, 2.612641, 0.331542",
    #              "0.896447, 2.717722, 0.362222, 4.715409, 1.9524345"],
    #             ["./map2.txt", "1.193665, 2.783110, 0.114285, 0.298177, 5.796323",
    #              "0.699517, 2.209383, 1.836998, 1.058671, 5.628539"],  
    #             ["./map2.txt", "1.426454, 0.953590, 0.357237, 3.524812, 0.743812",
    #              "1.010924, 2.990779, 0.700475, 3.774283, 0.197890"], 
    #             ["./map2.txt", "1.509483, 2.712544, 5.506200, 1.697402, 0.324063",
    #              "0.316585, 2.262115, 1.549368, 6.278967, 2.685507"], 
    #             ["./map2.txt", "1.510058, 1.380625, 2.230860, 1.150521, 0.835278",
    #              "1.691074, 2.297973, 5.233143, 1.019553, 2.999407"], 
    scores = []
    for repeat in range(4):
        for aPlanner in [2]:
            for i, data in enumerate(problems):
                inputMap, startPos, goalPos = [*data]
                numDOFs = len(startPos.split(","))
                outputSolutionFile = "tmp.txt"
                startPosString = ",".join(convertPIs(startPos))
                goalPosString = ",".join(convertPIs(goalPos))
                commandPlan = "{} {} {} {} {} {} {}".format(
                    executablePath,
                    inputMap, numDOFs, startPosString, goalPosString,
                    aPlanner, outputSolutionFile)
                commandVerify = "./verifier.out {} {} {} {} {}".format(
                    inputMap, numDOFs, startPosString, goalPosString,
                    outputSolutionFile)
                print("Problem Number {}".format(i))
                try:
                    start = timer()
                    subprocess.run(commandPlan.split(" "), check=True) # True if want to see failure errors
                    timespent = timer() - start
                    returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                    if returncode != 0:
                        print("Returned an invalid solution")
                    
                    ### Calculate the cost from their solution
                    with open(outputSolutionFile) as f:
                        line = f.readline().rstrip()  # filepath of the map
                        solution = []
                        for line in f:
                            solution.append(line.split(",")[:-1]) # :-1 to drop trailing comma
                        solution = np.asarray(solution).astype(float)
                        numSteps = solution.shape[0]

                        ## Cost is sum of all joint angle movements
                        difsPos = np.abs(solution[1:,]-solution[:-1,])
                        cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()

                        success = returncode == 0
                        scores.append([aPlanner, inputMap, i, numSteps, cost, timespent, success, repeat])
                    ### Visualize their results
                    if(repeat == 0):
                        commandViz = "python visualizer.py tmp.txt --gifFilepath=grades|planner_{}|prob_{}|Itr_{}.gif".format(aPlanner,i,repeat)
                        commandViz += " --incPrev=1"
                        subprocess.run(commandViz.split(" "), check=True) # True if want to see failure errors
                except Exception as exc:
                    print("Failed: {} !!".format(exc))
                    scores.append([aPlanner, inputMap, i, -1, -1, timespent, False, repeat])

    ### Save all the scores into a csv to compute overall grades
    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numSteps", "cost", "timespent", "success","Iteration"])
    df.to_csv(gradingCSV, index=False)
            

if __name__ == "__main__":
    graderMain("./planner.out", "test.csv")