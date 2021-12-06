#include "modelFuns.h"

#ifdef Standing
/*
Computes the cost for standing simulation
*/
void computeCostsStanding(std::vector<double> &costs, OpenSim::Model &osimModel, const SimTK::State si0, 
                          const double seatReleaseTime, const double tF){

  OpenSim::TableReporterVec3 *comReporter = dynamic_cast<OpenSim::TableReporterVec3*>(
                                                    &osimModel.updComponent("comReporter"));
  OpenSim::TableReporter *muscleActivReporter = dynamic_cast<OpenSim::TableReporter*>(
                                                    &osimModel.updComponent("/muscleActivReporter"));
  OpenSim::TableReporter_<SimTK::SpatialVec> *feetForceReporter = dynamic_cast<OpenSim::TableReporter_<SimTK::SpatialVec>*>(
                                                    &osimModel.updComponent("/feetForceReporter"));
  OpenSim::ForceReporter *frcReporter = dynamic_cast<OpenSim::ForceReporter*>(
                                                    &osimModel.updAnalysisSet().get("forceReporter"));
  OpenSim::TableReporter *coordReporter = dynamic_cast<OpenSim::TableReporter*>(
                                                    &osimModel.updComponent("/coordReporter"));

  osimModel.realizePosition(si0);
  const OpenSim::PhysicalOffsetFrame *heelCnctFrame = dynamic_cast<const OpenSim::PhysicalOffsetFrame*>
                                                      (&osimModel.updComponent("/bodyset/calcn_r/heel_cnctFrame"));
  const OpenSim::PhysicalOffsetFrame *toesCnctFrame = dynamic_cast<const OpenSim::PhysicalOffsetFrame *>
                                                      (&osimModel.getComponent("/bodyset/toes_r/toes_cnctFrame"));

  const OpenSim::Body &talus = osimModel.getBodySet().get("talus_r");
  const SimTK::Vec3 feetPos = talus.getPositionInGround(si0);
  const SimTK::Vec3 heelPos = heelCnctFrame->getPositionInGround(si0);                                                    
  const SimTK::Vec3 toesPos = toesCnctFrame->getPositionInGround(si0);                                                    

  const auto &comTimeSeries = comReporter->getTable();
  const auto &activationTimeSeries = muscleActivReporter->getTable();
  const auto &feetWrenchTimesSeries = feetForceReporter->getTable();
  const auto &coordTimeSeries = coordReporter->getTable();

  //// Can not convert to timeseries as it will break the storage due to constraint being disabled
  const auto &forceStorage = frcReporter->getForceStorage(); 
  OpenSim::Array<double> forceTimeArray;
  forceStorage.getTimeColumn(forceTimeArray);
  const std::vector<double> forceTimeVec(forceTimeArray.get(), forceTimeArray.get()+forceTimeArray.getSize());

  const std::vector<double> &reporterTimeVec = comReporter->getTable().getIndependentColumn();
  const double tau_ChairForce = tF*tau_ChairForce_pct;
  const double tau_Boundary = tF*tau_Boundary_pct;

  std::vector<double> wTauBoundaryVec = expWeightVec(tau_Boundary, reporterTimeVec, tF);
  std::vector<double> wTauChairForceVec = expWeightVec(tau_ChairForce, forceTimeVec, tF);
  std::vector<double> wTauCoordVel = expWeightVec(tau_ChairForce, reporterTimeVec, tF);

  const SimTK::Vec3 feetCosts = computeCostFeet(feetWrenchTimesSeries, heelPos, toesPos);
  const SimTK::Vec2 chairCosts= computeCostsChair(wTauChairForceVec, forceStorage);

  //// Filling up the cost matrix
  if(tF<simulationDuration-1e-2){
    //const double bodyWeight = osimModel.getTotalMass(si0)*(osimModel.getGravity()[1]);
    costs[0] = -1.25;
    costs[1] = 0.0;
    costs[2] = computeCostCoordinateVel(wTauCoordVel, coordTimeSeries);
    //costs[2] = computeCostFeetForce(wTauCoordVel, feetWrenchTimesSeries, bodyWeight);
    costs[3] = 0.0;
  }
  else{
    costs[0] = computeCostComY(wTauBoundaryVec, comTimeSeries);
    costs[1] = computeCostComX(wTauBoundaryVec, comTimeSeries, feetPos);
    costs[2] = 0.0;
    costs[3] = chairCosts[0];
  }
  costs[4] = computeCostActivation(activationTimeSeries, seatReleaseTime);
  costs[5] = computeCostDiffActivation(activationTimeSeries, seatReleaseTime);
  costs[6] = computeCostLimitTorque(forceStorage);
  costs[7] = feetCosts[0];
  costs[8] = feetCosts[1];
  costs[9] = feetCosts[2] + chairCosts[1];
  #ifdef Assisted
    costs[10] = computeCostAssistance(forceStorage);
  #endif

  //// Clearing the reporters
  comReporter->clearTable();
  muscleActivReporter->clearTable();
  feetForceReporter->clearTable();
  coordReporter->clearTable();
};

double computeCostComY(const std::vector<double> &weightVec, const OpenSim::TimeSeriesTableVec3 &comTimeSeries){
  const size_t indCom = comTimeSeries.getColumnIndex("/|com_position");
  auto comTPosVec = comTimeSeries.getDependentColumnAtIndex(indCom);
  const SimTK::Vec3 comT0 = comTPosVec[0];

  double result = std::inner_product(weightVec.begin(), weightVec.end(), comTPosVec.begin(), 0.0, std::plus<double>(),
                                          [&comT0](const double &w, const SimTK::Vec3 &comT){
                                            return (comT0[1] - comT[1]);
                                          });
  result *= reportInterval;
  return result;
}

double computeCostComX(const std::vector<double> &weightVec, const OpenSim::TimeSeriesTableVec3 &comTimeSeries, 
                        const SimTK::Vec3 feetPos){
  const size_t indCom = comTimeSeries.getColumnIndex("/|com_position");
  auto comTPosVec = comTimeSeries.getDependentColumnAtIndex(indCom);
  const SimTK::Vec3 comT0 = comTPosVec[0];

  double result = std::inner_product(weightVec.begin(), weightVec.end(), comTPosVec.begin(), 0.0, std::plus<double>(),
                                          [&comT0, &feetPos](const double &w, const SimTK::Vec3 &comT){
                                            return fabs(feetPos[0] - comT[0]);
                                          });
  result *= reportInterval;                
  return result;
}

double computeCostCoordinate(const std::vector<double> &weightVec, const OpenSim::TimeSeriesTable coordTimeSeries){
  double result = 0.0;
  const int numRelevantCoordinate = coordTimeSeries.getNumColumns()/2;
  for(size_t iCoord = 0; iCoord<numRelevantCoordinate; ++iCoord){
    auto coordValueVec = coordTimeSeries.getDependentColumnAtIndex(2*iCoord);

    result += std::inner_product(weightVec.begin(), weightVec.end(), coordValueVec.begin(), 0.0,
                                std::plus<double>(), [] (const double w, const double value){
                                    return w*fabs(value);
                            });
  }
  result = SimTK::convertRadiansToDegrees(result)*reportInterval;
  return result;
}

double computeCostCoordinateVel(const std::vector<double> &weightVec, const OpenSim::TimeSeriesTable coordTimeSeries){
  double result = 0.0;
  const int numRelevantCoordinate = coordTimeSeries.getNumColumns()/2;
  const std::vector<double> &time = coordTimeSeries.getIndependentColumn();
  for(size_t iCoord = 0; iCoord<numRelevantCoordinate; ++iCoord){
    auto coordVelVec = coordTimeSeries.getDependentColumnAtIndex(2*iCoord+1);
    result += std::inner_product(weightVec.begin(), weightVec.end(), coordVelVec.begin(), 0.0,
                                std::plus<double>(), [] (const double w, const double value){
                                    return w*fabs(value);
                            });
  }
  result = SimTK::convertRadiansToDegrees(result)*reportInterval;
  return result;
}

double computeCostComVel(const std::vector<double> &weightVec, const OpenSim::TimeSeriesTableVec3 &comTimeSeries){
  const size_t indCom = comTimeSeries.getColumnIndex("/|com_velocity");
  auto comVelVec = comTimeSeries.getDependentColumnAtIndex(indCom);
  double result = std::inner_product(weightVec.begin(), weightVec.end(), comVelVec.begin(), 0.0, std::plus<double>(),
                                          [](const double &w, const SimTK::Vec3 &comVelT){
                                            return (w*std::sqrt(comVelT[0]*comVelT[0] + comVelT[1]*comVelT[1]));
                                          });

  result *= reportInterval;
  return result;
}

double computeCostFeetForce(const std::vector<double> &weightVec, const OpenSim::TimeSeriesTable_<SimTK::SpatialVec> &feetWrenchTimeSeries, 
                            const double bodyWeight){
  const int indfeetWrench = feetWrenchTimeSeries.getColumnIndex("/jointset/ground_calcn_r|reaction_on_parent");
  auto feetWrenchVec = feetWrenchTimeSeries.getDependentColumnAtIndex(indfeetWrench);
  double result = 0.0;
  for(int i=0; i<weightVec.size(); ++i){
    result += weightVec[i]*fabs(feetWrenchVec[i][1][1] - bodyWeight);
  }
  result *= reportInterval;
  return result;
}

SimTK::Vec2 computeCostsChair(const std::vector<double> &weightVec, const OpenSim::Storage &forceStorage){
  const int indChairForceX = forceStorage.getStateIndex("seatConstraint_ground_Fx");
  std::vector<double> chairForceXTVec(weightVec.size(), 0.0);
  double *chairForceXTVecRawPtr = chairForceXTVec.data();
  forceStorage.getDataColumn(indChairForceX, chairForceXTVecRawPtr);

  const int indChairForceY = forceStorage.getStateIndex("seatConstraint_ground_Fy");
  std::vector<double> chairForceYTVec(weightVec.size(), 0.0);
  double *chairForceYTVecRawPtr = chairForceYTVec.data();
  forceStorage.getDataColumn(indChairForceY, chairForceYTVecRawPtr);

  std::vector<double> tVec(weightVec.size(), 0.0);
  double *tVecStart = tVec.data();
  forceStorage.getTimeColumn(tVecStart);
  const std::vector<double> dtVec = dVector(tVec);

  std::vector<double> frictionPenaltyVec(chairForceXTVec.size(), 0.0);
  std::transform(chairForceXTVec.begin(), chairForceXTVec.end(), chairForceYTVec.begin(), frictionPenaltyVec.begin(), 
                  [](const double forceXt, const double forceYt){
                      return bool(fabs(forceXt) > mu_static*forceYt)*(fabs(forceXt) - mu_static*forceYt);
                  });
  const double slipPenalty =  std::inner_product(frictionPenaltyVec.begin(), frictionPenaltyVec.end(), dtVec.begin(), 0.0);

  std::vector<double> chairForceYPenaltyVec(dtVec.size(), 0.0);
  std::transform( chairForceYTVec.begin(), chairForceYTVec.end(), dtVec.begin(), chairForceYPenaltyVec.begin(), std::multiplies<double>());

  const double costChairForce = fabs(std::inner_product(chairForceYPenaltyVec.begin(), chairForceYPenaltyVec.end(), weightVec.begin(), 0.0));
  return SimTK::Vec2{costChairForce, slipPenalty};
}

std::vector<double> expWeightVec(const double tau, const std::vector<double> &timeVec, const double tF){
  std::vector<double> result(timeVec.size());
  std::transform(timeVec.begin(), timeVec.end(), result.begin(), 
                [tau, tF](const double &t){
                  return getExpWeight(tau, t, tF);
                });
  return result;
}

#endif