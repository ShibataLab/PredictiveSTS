#pragma once
#include <iostream>
#include <string>
#include "cmaes.h"
#include "OpenSim.h"

#define Standing                            // Commenting will switch to Sitting
//#define Assisted                          // Commenting will switch to unAssisted

// Sets up model and number of variables
#ifdef Assisted
    const std::string BASE_MODEL_NAME = "vin_80pctAssisted.osim";
    const std::string OPTIM_MODEL_NAME = "vin_80pctAssisted.osim.mod";
    const std::string REPLAY_MODEL_NAME = "vin_80pctAssisted_replay.osim.mod";
    const int NUM_ACTUATORS = 12;                  // Number of unique excitation functions
#ifdef Standing
    const size_t NUM_WEIGHTS = 10;
#else
    const size_t NUM_WEIGHTS = 8;
#endif

#else
    const std::string BASE_MODEL_NAME = "vin_lumbarN2.osim";
    const std::string OPTIM_MODEL_NAME = "vin_lumbarN2.osim.mod";
    const std::string REPLAY_MODEL_NAME = "vin_lumbarN2_replay.osim.mod";
    //const std::string BASE_MODEL_NAME = "vin_20pct.osim";
    //const std::string OPTIM_MODEL_NAME = "vin_20pct.osim.mod";
    //const std::string REPLAY_MODEL_NAME = "vin_20pct_replay.osim.mod";
    //const std::string BASE_MODEL_NAME = "vin_40pct.osim";
    //const std::string OPTIM_MODEL_NAME = "vin_40pct.osim.mod";
    //const std::string REPLAY_MODEL_NAME = "vin_40pct_replay.osim.mod";
    //const std::string BASE_MODEL_NAME = "vin_60pct.osim";
    //const std::string OPTIM_MODEL_NAME = "vin_60pct.osim.mod";
    //const std::string REPLAY_MODEL_NAME = "vin_60pct_replay.osim.mod";
    //const std::string BASE_MODEL_NAME = "vin_80pct.osim";
    //const std::string OPTIM_MODEL_NAME = "vin_80pct.osim.mod";
    //const std::string REPLAY_MODEL_NAME = "vin_80pct_replay.osim.mod";

    const int NUM_ACTUATORS = 10; 
#ifdef Standing
    const size_t NUM_WEIGHTS = 9;
#else
    const size_t NUM_WEIGHTS = 7;
#endif
#endif

// Simulation Setup
const double T0 = 0.0;             // Simulation starting time
const double T_MAX = 1.6;

const double MU_STATIC = 0.8;               // coefficient of static friction
const double SAMPLING_DT = 0.01;             // The rate at which excitation signal is sampled(sec) from the parameterization function
const double REPORT_INTERVAL = 0.001;         // Reporting Interval of table reporters(secs). May be made noisy to avoid peculiarities
const double INTEGRATOR_ACCURACY = 1.0e-4;   // Desired Integration Accuracy
const double DEFAULT_EXCITATION = 0.05;

// Optimization Setup 

const SimTK::Vec3 COM_TARGET{0.0508939, 0.917116, -0.054666};
const int NUM_COMPS = 16;                   // Number of parameterization componenets each excitation trajectory is made up of
const double TAU_CHAIR_FORCE = T_MAX/8.0;
const int N_RESTARTS = 5;
const double F_TOLERANCE = 1.0;
const int HIST_SIZE = 250;
const int MAX_ITER = 4000;

#ifdef Assisted
    const std::string LOWER_BOUNDS_FILE = "lowerBoundAssisted.txt";
    const std::string UPPER_BOUNDS_FILE = "upperBoundAssisted.txt";
    const std::string STD_DEV_FILE = "stdDevAssisted.txt";
#else
    const std::string LOWER_BOUNDS_FILE = "lowerBoundUnAssisted.txt";
    const std::string UPPER_BOUNDS_FILE = "upperBoundUnAssisted.txt";
    const std::string STD_DEV_FILE = "stdDevUnAssisted.txt";
#endif


// Log file Names
const std::string LOG_BEST_FILE = "logBest.txt";
const std::string LOG_MEAN_FILE = "logMean.txt";
const std::string RESUME_MEAN_FILE = "resumeMean.dat";
const std::string RESUME_COV_FILE = "resumeCov.dat";
const std::string RESUME_SIGMA_FILE = "resumeSigma.dat";

typedef libcmaes::GenoPheno<libcmaes::pwqBoundStrategy,libcmaes::linScalingStrategy> GenoPhenoType;