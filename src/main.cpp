/* -------------------------------------------------------------------------- *
Used to synthesize assisted/Unassisted Sit to Stand and Sitting motions
Look universalConsts.h and weights__.txt files for hyper parameters

To Do :
1) Automate seat constraint index.
2) Patch forcestorage for seat contraint release
Until 1) and 2) make sure that seat constrainst is the last foce added to the model
and correct index is passed to seatConstraintHandler
*/

#include "OpenSim/OpenSim.h"
#include "modelFuns.h"
#include "cmaes.h"
#include "universalConsts.h"
#include "utility.h"

#include <iostream>
#include <mutex>
#include <math.h>
#include <vector>
#include <algorithm>

using namespace libcmaes;

const ParameterizationType parameterization = ParameterizationType::PWLinearFixedDt;
std::mutex threadLock;

/*
Performs N_RESTART independent optimizations and saves the optimal trajectory with
the lowest cost among them.

Logging is done in a time stamp based unique folder created inside results directory
The same directory can be used to resume optimization.

If resume directory is provided the guess inside initialMeanFile.txt is overwritten
*/
int main(int argc, const char *argv[])
{
    try {
        if (argc != 4){
            std::cout << "Inappropriate number of function arguments" << std::endl;
            std::cout << "Correct Format: funcName.exe initialMeanFile.txt"
                        "weightsFile.txt resumeDirectory/null" << std::endl;
            exit(1);
        }

        const std::string initMeanFileName = argv[1]; 
        const std::string weightsFileName = argv[2];

        bool resumeFlag = false;
        std::string resumeDir = argv[3];
        if(resumeDir != "null"){
            resumeFlag = true;
        }

        std::vector<double> weightsVec(NUM_WEIGHTS);
        readVector(weightsVec, weightsFileName);

        const int numVarsPerComp = mapNumVarsPerComp[parameterization];
        const int numDecisionVars = numVarsPerComp * NUM_COMPS * NUM_ACTUATORS + 1;
        std::vector<double> initMean(numDecisionVars);
        #ifdef Standing
            readVector(initMean, initMeanFileName, 1, 1, 2);
        #else
            readVector(initMean, initMeanFileName);
        #endif
        std::vector<double> initStepSize(numDecisionVars), upperBound(numDecisionVars), 
                            lowerBound(numDecisionVars);
        readVector(initStepSize, STD_DEV_FILE);
        readVector(lowerBound, LOWER_BOUNDS_FILE);
        readVector(upperBound, UPPER_BOUNDS_FILE);

        const std::string logFolder = createUniqueFolder("results/");
        const std::string plotFile = logFolder + "plotting.dat";

        OpenSim::ModelVisualizer::addDirToGeometrySearchPaths("../geometry");
        OpenSim::ModelVisualizer::addDirToGeometrySearchPaths("../geometry/LumbarGeometry");
        addComponentsToModel(BASE_MODEL_NAME, OPTIM_MODEL_NAME);

        FitFunc objectiveFunc = [&weightsVec](const double* newControls, const int numDecisionVars){
            std::unique_lock<std::mutex> locker(threadLock);
                OpenSim::Model osimModel(OPTIM_MODEL_NAME);
            locker.unlock();

            const std::vector<double> costs = runSimulation(osimModel, parameterization, 
                                                            numDecisionVars, newControls);
            const double value = std::inner_product(weightsVec.begin(), weightsVec.end(), costs.begin(), 0.0);
            return value;
        };

        ProgressFunc<CMAParameters<GenoPhenoType>,CMASolutions> writeLogs = [&logFolder]
            (const CMAParameters<GenoPhenoType> &cmaparams, const CMASolutions &cmasols){
                progressFunc(cmaparams, cmasols, logFolder);
                return 0;
        };

        CMAParameters<GenoPhenoType> cmaparams(initMean, initStepSize, -1, lowerBound, upperBound);
        cmaparams.set_algo(aCMAES);
        cmaparams.set_fplot(plotFile);
        cmaparams.set_max_hist(HIST_SIZE);
        cmaparams.set_ftolerance(F_TOLERANCE);
        cmaparams.set_max_iter(MAX_ITER);
        cmaparams.set_noisy();
        cmaparams.set_mt_feval(true);

        CMASolutions cmasols;
        for(int i=0; i<N_RESTARTS; ++i){
            auto seed = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>
                            (std::chrono::high_resolution_clock::now().time_since_epoch()).count());
            cmaparams.set_seed(seed);
            cmaparams.initialize_parameters();
            CMASolutions cmasols_temp(cmaparams);

            if(i==0 && resumeFlag){
                cmasols_temp = resumeDistribution(resumeDir, cmaparams);
            }
            if(i>0){
                cmasols_temp.set_xmean(cmasols.get_best_seen_candidate().get_x_dvec());
            }

            cmasols_temp = cmaes<GenoPhenoType>(objectiveFunc, cmaparams, writeLogs, nullptr, 
                                                    cmasols_temp);

            if(cmasols_temp.get_best_seen_candidate().get_fvalue() < 
                cmasols.get_best_seen_candidate().get_fvalue() ||  cmasols.candidates().empty()){
                cmasols = cmasols_temp;
            }
        }

        std::cout << "Optimization Finished" << std::endl;

        Eigen::VectorXd optimParams = cmaparams.get_gp().pheno(cmasols.get_best_seen_candidate().get_x_dvec());
        OpenSim::Model osimModel(OPTIM_MODEL_NAME);
        std::vector<double> finalCosts = runSimulation(osimModel, parameterization, numDecisionVars, 
                                                        optimParams.data(), false, true, logFolder+"optimTraj");

        const double totalCost = std::inner_product(finalCosts.begin(), finalCosts.end(), weightsVec.begin(), 0.0); 
        std::cout << "Optimized Cost : " << totalCost << std::endl;
    }
    catch(const std::string& ex){
        std::cout << ex << std::endl;
        return 1;
    }
    catch (const std::exception& ex){
        std::cout << ex.what() << std::endl;
        return 1;
    }

    return 0;
}