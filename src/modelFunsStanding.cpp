#include "modelFuns.h"

#ifdef Standing
/*
Computes the cost for standing simulation
*/
void computeCostsStanding(std::vector<double> &costs, OpenSim::Model &osimModel, const SimTK::State &si0, const SimTK::State &siF, 
                          const double seatOffTime){

  OpenSim::TableReporter *muscleActivReporter = dynamic_cast<OpenSim::TableReporter*>(
                                                    &osimModel.updComponent("/muscleActivReporter"));
  OpenSim::TableReporter_<SimTK::SpatialVec> *feetForceReporter = dynamic_cast<OpenSim::TableReporter_<SimTK::SpatialVec>*>(
                                                    &osimModel.updComponent("/feetForceReporter"));
  OpenSim::ForceReporter *frcReporter = dynamic_cast<OpenSim::ForceReporter*>(
                                                    &osimModel.updAnalysisSet().get("forceReporter"));

  osimModel.realizePosition(si0);
  const OpenSim::PhysicalOffsetFrame *heelCnctFrame = dynamic_cast<const OpenSim::PhysicalOffsetFrame*>
                                                      (&osimModel.updComponent("/bodyset/calcn_r/heel_cnctFrame"));
  const OpenSim::PhysicalOffsetFrame *toesCnctFrame = dynamic_cast<const OpenSim::PhysicalOffsetFrame *>
                                                      (&osimModel.getComponent("/bodyset/toes_r/toes_cnctFrame"));

  const SimTK::Vec3 heelPos = heelCnctFrame->getPositionInGround(si0);
  const SimTK::Vec3 toesPos = toesCnctFrame->getPositionInGround(si0);

  const auto &activationTimeSeries = muscleActivReporter->getTable();
  const auto &feetWrenchTimesSeries = feetForceReporter->getTable();
  const auto &forceStorage = frcReporter->getForceStorage();

  const SimTK::Vec3 feetCosts = computeCostFeet(feetWrenchTimesSeries, heelPos, toesPos);
  const SimTK::Vec2 chairCosts= computeCostsChair(forceStorage);

  const double bodyWeight = osimModel.getTotalMass(si0)*(osimModel.getGravity()[1]);
  const SimTK::Vec3 comT0 = osimModel.calcMassCenterPosition(si0);
  const SimTK::Vec3 comTf = osimModel.calcMassCenterPosition(siF);

  const double d0 = eucledianDis(comT0, comTarget);
  const double df = eucledianDis(comTf, comTarget);

  const double progress = 1.0-std::min(d0,df)/d0;
  std::cout << "progress " << progress << std::endl;

  costs[0] = df/d0;
  costs[1] = progress*computeCostJointVel(osimModel, siF);
  costs[2] = progress*computeCostFeetForce(feetWrenchTimesSeries, bodyWeight, seatOffTime, siF.getTime());
  costs[3] = (1.0-progress)*chairCosts[0];
  costs[4] = computeCostActivation(activationTimeSeries);
  costs[5] = computeCostDiffActivation(activationTimeSeries);
  costs[6] = computeCostLimitTorque(forceStorage);
  costs[7] = progress*feetCosts[0];
  costs[8] = progress*feetCosts[1];
  costs[9] = progress*(feetCosts[2] + chairCosts[1]);
  #ifdef Assisted
    costs[10] = computeCostAssistance(forceStorage);
  #endif

  //// Clearing the reporters
  muscleActivReporter->clearTable();
  feetForceReporter->clearTable();
};

double computeCostFeetForce(const OpenSim::TimeSeriesTable_<SimTK::SpatialVec> &feetWrenchTimeSeries, 
                            const double bodyWeight, const double seatOffTime, const double tF){
  double result = 0.0;
  if(seatOffTime<tF-1e-2){
    const int indfeetWrench = feetWrenchTimeSeries.getColumnIndex("/jointset/ground_calcn_r|reaction_on_parent");
    auto feetWrenchVec = feetWrenchTimeSeries.getDependentColumnAtIndex(indfeetWrench);
    const size_t seatOffInd = feetWrenchTimeSeries.getNearestRowIndexForTime(seatOffTime);

    result += std::accumulate(feetWrenchVec.begin()+seatOffInd, feetWrenchVec.end(), 0.0,
                                        [bodyWeight](const double &lhs, const SimTK::SpatialVec &feetWrenchT){
                                          return lhs+fabs(feetWrenchT[1][1] - bodyWeight);
                                        });
    result = result*reportInterval/(tF-seatOffTime);
    //result = fabs(feetWrenchVec[feetWrenchVec.size()-1][1][1] - bodyWeight);
  }
  return result;
}

//// Can not convert to timeseries as it will break the storage due to constraint being disabled
SimTK::Vec2 computeCostsChair(const OpenSim::Storage &forceStorage){
  OpenSim::Array<double> forceTimeArray;
  forceStorage.getTimeColumn(forceTimeArray);
  const std::vector<double> tVec(forceTimeArray.get(), forceTimeArray.get()+forceTimeArray.getSize());
  const std::vector<double> dtVec = dVector(tVec);

  //// Force applied to ground by the constraint
  const int indChairForceX = forceStorage.getStateIndex("seatConstraint_ground_Fx");
  std::vector<double> chairForceXTVec(tVec.size(), 0.0);
  double *chairForceXTVecRawPtr = chairForceXTVec.data();
  forceStorage.getDataColumn(indChairForceX, chairForceXTVecRawPtr);

  const int indChairForceY = forceStorage.getStateIndex("seatConstraint_ground_Fy");
  std::vector<double> chairForceYTVec(tVec.size(), 0.0);
  double *chairForceYTVecRawPtr = chairForceYTVec.data();
  forceStorage.getDataColumn(indChairForceY, chairForceYTVecRawPtr);

  std::vector<double> slipVec(chairForceXTVec.size(), 0.0);
  std::transform(chairForceXTVec.begin(), chairForceXTVec.end(), chairForceYTVec.begin(), slipVec.begin(), 
                  [](const double forceXt, const double forceYt){
                      return std::max(0.0, fabs(forceXt) + mu_static*forceYt);
                  });
  const double slipPenalty =  *std::max_element(slipVec.begin(), slipVec.end());

  double costChairForce = fabs(std::inner_product(chairForceYTVec.begin(), chairForceYTVec.end(), dtVec.begin(), 0.0));
  costChairForce = costChairForce/tVec[tVec.size()-1];

  return SimTK::Vec2{costChairForce, slipPenalty};
}

double computeCostJointVel(const OpenSim::Model &osimModel, const SimTK::State &siF){
  const OpenSim::CoordinateSet &coordSet = osimModel.getCoordinateSet();
  const double hipVel = coordSet.get("hip_flexion").getSpeedValue(siF);
  const double kneeVel= coordSet.get("knee_angle").getSpeedValue(siF);
  const double ankleVel= coordSet.get("ankle_angle").getSpeedValue(siF);

  return SimTK::convertRadiansToDegrees(fabs(hipVel)+fabs(kneeVel)+fabs(ankleVel));
}

#endif