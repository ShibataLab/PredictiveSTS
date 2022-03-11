#include "modelFuns.h"

void addComponentsToModel(const std::string modelName, const std::string newModelName){
    OpenSim::Model osimModel(modelName);
    addController(osimModel);
    osimModel.finalizeConnections();
    osimModel.print(newModelName);
};

void addController(OpenSim::Model &osimModel){
    const OpenSim::Set<OpenSim::Actuator> &actuators = osimModel.getActuators();
    OpenSim::PrescribedController *openLoopController = new OpenSim::PrescribedController();
    openLoopController->setName("openLoopController");
    openLoopController->setActuators(actuators);
    const int nSamples = int(T_MAX / SAMPLING_DT) + 1;
    SimTK::Vector vecExt(nSamples, DEFAULT_EXCITATION);
    SimTK::Vector vecTime(nSamples);
    std::iota(vecTime.begin(), vecTime.end(), 0);
    std::for_each(vecTime.begin(), vecTime.end(), [](double &value){ value *= SAMPLING_DT;});
    for(size_t i=0; i<actuators.getSize(); ++i){
      OpenSim::PiecewiseLinearFunction *func = new OpenSim::PiecewiseLinearFunction(nSamples,
                                                    vecTime.getContiguousScalarData(),
                                                    vecExt.getContiguousScalarData(),
                                                    "ExcitationSignal");

      openLoopController->prescribeControlForActuator( actuators[i].getName(), func);
    }
    osimModel.addController(openLoopController);
};

void addReporters(OpenSim::Model &osimModel){
    // Reporter of the applied on the ground by the feet
    OpenSim::Joint &groundFeetJoint = osimModel.updJointSet().get("ground_calcn_r");
    OpenSim::TableReporter_<SimTK::SpatialVec> *feetForceReporter = new OpenSim::TableReporter_<SimTK::SpatialVec>();
    feetForceReporter->set_report_time_interval(REPORT_INTERVAL);
    feetForceReporter->addToReport(groundFeetJoint.getOutput("reaction_on_parent"));
    feetForceReporter->setName("feetForceReporter");
    osimModel.addComponent(feetForceReporter);
    OpenSim::TableReporter *actuatorActivReporter = new OpenSim::TableReporter();
    actuatorActivReporter->set_report_time_interval(REPORT_INTERVAL);
    actuatorActivReporter->setName("actuatorActivReporter");
    OpenSim::Set<OpenSim::Muscle> &muscleSet = osimModel.updMuscles();
    for(size_t i=0; i<muscleSet.getSize(); ++i){
        actuatorActivReporter->addToReport(muscleSet[i].getOutput("activation"));
    }
    #ifdef Assisted
        OpenSim::ActivationPointActuator *assistFx = dynamic_cast<OpenSim::ActivationPointActuator*>
                                                    (&osimModel.updForceSet().get("assistFx"));
        OpenSim::ActivationPointActuator *assistFy = dynamic_cast<OpenSim::ActivationPointActuator*>
                                                    (&osimModel.updForceSet().get("assistFy"));

        actuatorActivReporter->addToReport(assistFx->getOutput("activation"));
        actuatorActivReporter->addToReport(assistFy->getOutput("activation"));
    #endif
    osimModel.addComponent(actuatorActivReporter);
    #ifndef Standing
        const OpenSim::CoordinateSet &coordSet = osimModel.getCoordinateSet();
        const std::vector<int> coordinateIndices{coordSet.getIndex("hip_flexion"),
                                                coordSet.getIndex("knee_angle"),
                                                coordSet.getIndex("ankle_angle")};

        OpenSim::TableReporter *coordReporter = new OpenSim::TableReporter();
        coordReporter->set_report_time_interval(REPORT_INTERVAL);
        coordReporter->setName("coordReporter");
        for(auto ind : coordinateIndices){
            coordReporter->addToReport(coordSet[ind].getOutput("value"));
            coordReporter->addToReport(coordSet[ind].getOutput("speed"));
        }
        osimModel.addComponent(coordReporter);
    #endif
};

void setExcitations(OpenSim::Model &osimModel, SimTK::State &si0, const ParameterizationType parameterization,
                    const double *compValues){

    OpenSim::PrescribedController *openLoopController = dynamic_cast<OpenSim::PrescribedController*>
                                      (&osimModel.updComponent("/controllerset/openLoopController"));
    OpenSim::FunctionSet &actuatorExtFuncs = openLoopController->upd_ControlFunctions();
    const int numActuators = actuatorExtFuncs.getSize();
    const int numVarsPerComp = mapNumVarsPerComp[parameterization];
    OpenSim::Set<OpenSim::Actuator> &actuatorSet = osimModel.updActuators();
    for(size_t i=0; i<numActuators; ++i){
        OpenSim::PiecewiseLinearFunction *actuatorExtFuncPtr = dynamic_cast<OpenSim::PiecewiseLinearFunction*>
                                                                (&actuatorExtFuncs[i]);
        const SimTK::Vector params(numVarsPerComp * NUM_COMPS, &compValues[numVarsPerComp * NUM_COMPS * i]);
        const std::unique_ptr<CustomFunction> parameterizationExtFuncPtr = mapExtFuncPtr[parameterization](params);
        OpenSim::ScalarActuator *actuator = dynamic_cast<OpenSim::ScalarActuator*>(&actuatorSet[i]);
        if(actuator == NULL){
            std::cout << "Could not convert to a scalar actuator" << std::endl;
        }
        const double lowerControlLimit = actuator->get_min_control();
        const double upperControlLimit = actuator->get_max_control();
        for(size_t j=0; j<actuatorExtFuncPtr->getNumberOfPoints(); ++j){
            double excitation = parameterizationExtFuncPtr->getValue(actuatorExtFuncPtr->getX(j));

            // Clamping is required for correct bheavior of first order dynamics
            excitation = SimTK::clamp(lowerControlLimit, excitation, upperControlLimit);
            actuatorExtFuncPtr->setY(j, excitation);
        }
        OpenSim::Muscle *muscle = dynamic_cast<OpenSim::Muscle*>(&actuatorSet[i]);
        if(muscle != NULL){
            muscle->setActivation(si0, actuatorExtFuncPtr->getY(0));
        }
    }
};

std::vector<double> runSimulation(OpenSim::Model &osimModel, const ParameterizationType &parameterization,
                                const int numDecisionVars, const double *controls,
                                const bool visualizeResults, const bool saveResults,
                                const std::string outputPrefix){

    addReporters(osimModel);
    osimModel.setUseVisualizer(visualizeResults);
    auto frcReporter{std::make_unique<OpenSim::ForceReporter>()};
    frcReporter->setName("forceReporter");
    osimModel.addAnalysis(frcReporter.get());

    #ifdef Assisted
        OpenSim::AssistanceForceVisualization *forceVisualizer = new OpenSim::AssistanceForceVisualization();
        osimModel.addComponent(forceVisualizer);
    #endif

    osimModel.finalizeConnections();
    osimModel.buildSystem();

    if(visualizeResults) osimModel.updVisualizer().updSimbodyVisualizer().setShowSimTime(true);

    ReleaseSeatConstraint *releaseSeatConstraint = new ReleaseSeatConstraint(osimModel, 1e-1);
    osimModel.updMultibodySystem().addEventHandler(releaseSeatConstraint);

    SimTK::State &si = osimModel.initializeState();
    si.setTime(T0);

    std::vector<double> costs(NUM_WEIGHTS);
    std::fill(costs.begin(), costs.end(), 1e6);

    try{
        setExcitations(osimModel, si, parameterization, controls);
        osimModel.equilibrateMuscles(si);
        const SimTK::State si0(si);

        OpenSim::Manager manager(osimModel);
        manager.setIntegratorAccuracy(INTEGRATOR_ACCURACY);
        manager.initialize(si);

        const double simT = controls[numDecisionVars - 1];
        const SimTK::State &siF = manager.integrate(simT);

        double seatOffTime = releaseSeatConstraint->getSeatRleaseTime();
        if (seatOffTime < 0.0){
            seatOffTime = siF.getTime();
        }

        // The computingCosts function clears the reporter memories. Therefore, exports needs to be done
        //before the cost is computed
        if(saveResults){
            OpenSim::STOFileAdapter_<double>::write(manager.getStatesTable(), outputPrefix + ".sto");
            frcReporter->getForceStorage().print(outputPrefix + "_force.mot");
            OpenSim::TableReporter_<SimTK::SpatialVec> *feetForceReporter = 
                                            dynamic_cast<OpenSim::TableReporter_<SimTK::SpatialVec>*>
                                                (&osimModel.updComponent("/feetForceReporter"));

            OpenSim::STOFileAdapter_<SimTK::SpatialVec>::write(feetForceReporter->getTable(), 
                                                            outputPrefix + "_feetForces.mot");
        }
        #ifdef Standing
            computeCostsStanding(costs, osimModel, si0, siF, seatOffTime);
        #else
            computeCostsSitting(costs, osimModel, si0);
        #endif
    }
    catch(const std::string& ex){
        std::cout << ex << std::endl;
        std::cout << "Sending Default Costs" << std::endl;
        std::cout << std::endl;
    }
    catch(const std::exception& ex){
        std::cout << ex.what() << std::endl;
        std::cout << "Sending Default Costs" << std::endl;
    }
    return costs;
};

double computeCostLimitTorque(const OpenSim::Storage &forceStorage){
    double result{0.0};
    OpenSim::Array<double> timeArray;
    forceStorage.getTimeColumn(timeArray);

    const std::vector<double> tVec(timeArray.get(), timeArray.get() + timeArray.size());
    const std::vector<double> dtVec = dVector(tVec);
    const std::vector<std::string> limitTorqueLabels{"hip_flexion_LimitForce", "knee_angle_LimitForce",
                                                    "ankle_angle_LimitForce"};

    for(auto limitTorqueLabel: limitTorqueLabels){
          std::vector<double> limitTorqueVec(dtVec.size(), 0.0);
          double *limitTorqueVecPtr = limitTorqueVec.data();
          forceStorage.getDataColumn(limitTorqueLabel, limitTorqueVecPtr);
          result += std::inner_product(limitTorqueVec.begin(), limitTorqueVec.end(), dtVec.begin(), 0.0,
                                        std::plus<double>(), [](const double &limitTorqueT, const double &dt){
                                            return fabs(limitTorqueT) * dt;
                                      });
    }
    result = result / (limitTorqueLabels.size());
    return result;
}

double computeCostActivation(const OpenSim::TimeSeriesTable &activationTimeSeries){
    double result{0.0};
    for(size_t i=0; i<activationTimeSeries.getNumColumns(); ++i){
        SimTK::VectorView activVec = activationTimeSeries.getDependentColumnAtIndex(i);
        result += std::inner_product(activVec.begin(), activVec.end(), activVec.begin(), 0.0);
    }

    result = sqrt(result * REPORT_INTERVAL / activationTimeSeries.getNumColumns());
    return result;
}

double computeCostDiffActivation(const OpenSim::TimeSeriesTable &activationTimeSeries){
    double result{0.0};
    for(size_t i=0; i<activationTimeSeries.getNumColumns(); ++i){
        SimTK::VectorView activVec = activationTimeSeries.getDependentColumnAtIndex(i);
        const std::vector<double> dActivVec = dVector(activVec);
        result += std::inner_product(dActivVec.begin(), dActivVec.end(), dActivVec.begin(), 0.0);
    }

    result = sqrt(result / (REPORT_INTERVAL * activationTimeSeries.getNumColumns()));
    return result;
}

SimTK::Vec3 computeCostFeet(OpenSim::Model &model, const SimTK::State &si0, const double seatOffTime){
    using namespace Eigen;
    OpenSim::TableReporter_<SimTK::SpatialVec> *feetForceReporter = dynamic_cast<OpenSim::TableReporter_<SimTK::SpatialVec>*>(
                                                      &model.updComponent("/feetForceReporter"));
    model.realizePosition(si0);
    const OpenSim::PhysicalOffsetFrame *heelCnctFrame = dynamic_cast<const OpenSim::PhysicalOffsetFrame*>
                                                        (&model.updComponent("/bodyset/calcn_r/heel_cnctFrame"));
    const OpenSim::PhysicalOffsetFrame *toesCnctFrame = dynamic_cast<const OpenSim::PhysicalOffsetFrame*>
                                                        (&model.getComponent("/bodyset/toes_r/toes_cnctFrame"));

    const SimTK::Vec3 heelPos = heelCnctFrame->getPositionInGround(si0);
    const SimTK::Vec3 toesPos = toesCnctFrame->getPositionInGround(si0);
    const double bodyWeight = model.getTotalMass(si0) * (model.getGravity()[1]);

    //// Forces applied by the feet on the ground
    const auto &feetWrenchTimeSeries = feetForceReporter->getTable();
    const int indfeetWrench = feetWrenchTimeSeries.getColumnIndex("/jointset/ground_calcn_r|reaction_on_parent");
    auto feetWrenchVec = feetWrenchTimeSeries.getDependentColumnAtIndex(indfeetWrench);
    Map<const Matrix<double,Dynamic,Dynamic,RowMajor>> feetWrenchMat(feetWrenchVec.getContiguousScalarData(),
                                                                      feetWrenchVec.nrow(), 6);
    auto torqueZ = feetWrenchMat(Eigen::all,2);
    auto forceX = feetWrenchMat(Eigen::all,3);
    auto forceY = feetWrenchMat(Eigen::all,4);
    auto slip = forceX.cwiseAbs() + MU_STATIC * forceY;
    double costSlip = std::max(0.0, slip.maxCoeff());
    auto zmp = torqueZ.array()/forceY.array();
    double costZMP = std::max(fabs(zmp.maxCoeff() - (heelPos[0] + toesPos[0]) / 2.0),
                              fabs(zmp.minCoeff() - (heelPos[0] + toesPos[0]) / 2.0));
    const double tF = feetWrenchTimeSeries.getIndependentColumn().back();
    double costAcc = 0.0;

    if(seatOffTime<tF){
        const size_t seatOffInd = feetWrenchTimeSeries.getNearestRowIndexForTime(seatOffTime);
        const double maxForce = *std::min_element(forceY.begin() + seatOffInd, forceY.end());
        const double minForce = *std::max_element(forceY.begin() + seatOffInd, forceY.end());

        costAcc = fabs(maxForce - bodyWeight) + fabs(minForce - bodyWeight) + fabs(forceY[forceY.size()-1] - bodyWeight);
        //std::cout << fabs(maxForce-bodyWeight) << ", " << fabs(minForce-bodyWeight)<< ", "
        //            << fabs(forceY[forceY.size()-1]-bodyWeight) << std::endl;
    }


    feetForceReporter->clearTable();
    return SimTK::Vec3(costZMP, costSlip, costAcc);
}

#ifdef Assisted
double computeCostAssistance(const OpenSim::Storage &forceStorage){
    OpenSim::Array<double> forceTimeArray;
    forceStorage.getTimeColumn(forceTimeArray);
    const std::vector<double> tVec(forceTimeArray.get(), forceTimeArray.get() + forceTimeArray.getSize());
    const std::vector<double> dtVec = dVector(tVec);
    const int indAssistFx = forceStorage.getStateIndex("assistFx");
    const int indAssistFy = forceStorage.getStateIndex("assistFy");
    std::vector<double> assistFxTVec(tVec.size(), 0.0);
    double *assistFxVecPtr = assistFxTVec.data();
    forceStorage.getDataColumn(indAssistFx, assistFxVecPtr);
    std::vector<double> assistFyTVec(tVec.size(), 0.0);
    double *assistFyVecPtr = assistFyTVec.data();
    forceStorage.getDataColumn(indAssistFy, assistFyVecPtr);
    std::vector<double> assistForceTVec(tVec.size(), 0.0);
    std::transform(assistFxTVec.begin(), assistFxTVec.end(), assistFyTVec.begin(), assistForceTVec.begin(),
                    [](const double fX, const double fY){
                        return std::sqrt(fX * fX + fY * fY);
                    });
    const double result = std::inner_product(assistForceTVec.begin(), assistForceTVec.end(), dtVec.begin(), 0.0);
    return result;
}
#endif