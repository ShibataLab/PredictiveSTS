/*
This code is used to
1) Access the progress of optimization by replaying using log file
2) Analyze optimization progress with costEvolution.py
*/

#include <iostream>
#include <string>
#include <future>
#include <mutex>

#include "OpenSim/OpenSim.h"
#include "parameterization.h"
#include "modelFuns.h"
#include "universalConsts.h"
#include "utility.h"

std::mutex threadLock;

const ParameterizationType parameterization = ParameterizationType::PWLinearFixedDt;

int main(int argc, char *argv[]){

    if (argc!=6){
        std::cout << "Inappropriate Number of Arguments" << std::endl;
        std::cout << "Correct Format:\n funcName logFileName.txt startGen endGen"
                    "visualize(true/false) saveData(true/false)" 
                    << std::endl;
        exit(1);
    }
    int numVarsPerComp = mapNumVarsPerComp[parameterization];

    const char *fileName = argv[1];
    int genStart = std::stoi(argv[2]);
    int genEnd = std::stoi(argv[3]);

    bool visualizeResults= false;
    if ((std::string)argv[4] == "true") visualizeResults = true;

    bool saveResults= false;
    if ((std::string)argv[5] == "true") saveResults = true;

    #ifdef Standing
        const std::vector<double> costWeights{800, 6, 0.3, 1.2, 175, 70, 40, 1000, 0.1, 5.0};
    #else
        const std::vector<double> costWeights{5, 4, 175, 70, 40, 1000, 0.1, 5.0};
    #endif

    OpenSim::ModelVisualizer::addDirToGeometrySearchPaths("../geometry");
    OpenSim::ModelVisualizer::addDirToGeometrySearchPaths("../geometry/LumbarGeometry");
    addComponentsToModel(BASE_MODEL_NAME, REPLAY_MODEL_NAME);

    const int numTotalVars = numVarsPerComp*NUM_COMPS*NUM_ACTUATORS+1;
    std::vector<int> gens(genEnd-genStart+1);
    std::iota(gens.begin(), gens.end(), 0);

    std::vector<std::future<void>> futuresVec;
    std::launch policy = std::launch::async;

    if (saveResults || visualizeResults) policy = std::launch::deferred;

    for (int gen : gens){
        futuresVec.push_back(std::async(policy, [&](int i){
            std::vector<double> inVec(numTotalVars+2);

            std::unique_lock<std::mutex> locker(threadLock);
                OpenSim::Model osimModel(REPLAY_MODEL_NAME);
                readVector(inVec, fileName, i+genStart, i+genStart, 0);
            locker.unlock();

            const std::string fileNamePrefix = std::to_string(int(inVec[0]));
            const std::vector<double> costs = runSimulation(osimModel, parameterization, 
                                                    numTotalVars, inVec.data()+2, visualizeResults, 
                                                    saveResults, fileNamePrefix);

            std::vector<double> relativeCosts(costs.size());
            std::transform(costs.begin(), costs.end(), costWeights.begin(), relativeCosts.begin(), 
                            std::multiplies<double>());

            const double totalCost = std::accumulate(relativeCosts.begin(), relativeCosts.end(), 0.0); 

            if(visualizeResults){
                locker.lock();
                    std::cout << int(inVec[0]) << " : " << totalCost << std::endl;
                    for(int i=0; i<relativeCosts.size(); ++i){
                        std::cout << relativeCosts[i] << " : " << costs[i] << std::endl;
                    }
                    std::cout <<std::endl;
                locker.unlock();
            }
            else{
                std::vector<double> vecOut;
                vecOut.push_back(inVec[0]);
                vecOut.push_back(totalCost);
                //vecOut.insert(vecOut.end(), inVec.begin(), inVec.begin()+2);
                vecOut.insert(vecOut.end(), costs.begin(), costs.end());
                locker.lock();
                    writeVector(vecOut, "costsData.csv", std::ios::app);
                locker.unlock();
            }
          }, gen
        )
        );
    }
    for(auto &e : futuresVec){
        e.get();
    }
    // Bye bye
    std::cout << "C++: Done!" << std::endl;
    return 0;
}