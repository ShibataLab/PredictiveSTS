#include "modelFuns.h"

#ifdef Standing
/*
Computes the cost for standing simulation
*/
void computeCostsStanding(std::vector<double> &costsOutputVec, OpenSim::Model &osimModel, 
                          const SimTK::State &si0, const SimTK::State &siF, 
                          const double seatOffTime){

	OpenSim::TableReporter *actuatorActivReporter = dynamic_cast<OpenSim::TableReporter*>
										(&osimModel.updComponent("/actuatorActivReporter"));
	OpenSim::ForceReporter *frcReporter = dynamic_cast<OpenSim::ForceReporter*>
										(&osimModel.updAnalysisSet().get("forceReporter"));

	const auto &activationTimeSeries = actuatorActivReporter->getTable();
	const auto &forceStorage = frcReporter->getForceStorage();

	const SimTK::Vec3 feetCosts = computeCostFeet(osimModel, si0, seatOffTime);
	const SimTK::Vec3 comT0 = osimModel.calcMassCenterPosition(si0);
	const SimTK::Vec3 comTf = osimModel.calcMassCenterPosition(siF);
	const double d0 = eucledianDis(comT0, COM_TARGET);
	const double df = eucledianDis(comTf, COM_TARGET);
	const double progress = 1.0 - std::min(d0,df) / d0;

	costsOutputVec[0] = df / d0;
	costsOutputVec[1] = progress * computeCostJointVel(osimModel, siF);
	costsOutputVec[2] = progress * feetCosts[2];
	costsOutputVec[3] = (1.0 - progress) * computeCostChair(forceStorage);
	costsOutputVec[4] = computeCostActivation(activationTimeSeries);
	costsOutputVec[5] = computeCostDiffActivation(activationTimeSeries);
	costsOutputVec[6] = computeCostLimitTorque(forceStorage);
	costsOutputVec[7] = progress * feetCosts[0];
	costsOutputVec[8] = progress * feetCosts[1];
	#ifdef Assisted
		costsOutputVec[9] = progress*computeCostAssistance(forceStorage);
	#endif

	actuatorActivReporter->clearTable();
};

double computeCostJointVel(const OpenSim::Model &osimModel, const SimTK::State &siF){
	const OpenSim::CoordinateSet &coordSet = osimModel.getCoordinateSet();
	const double hipVel = coordSet.get("hip_flexion").getSpeedValue(siF);
	const double kneeVel = coordSet.get("knee_angle").getSpeedValue(siF);
	const double ankleVel = coordSet.get("ankle_angle").getSpeedValue(siF);

	return fabs(hipVel) + fabs(kneeVel) + fabs(ankleVel);
}

double computeCostChair(const OpenSim::Storage &forceStorage){
	OpenSim::Array<double> forceTimeArray;
	forceStorage.getTimeColumn(forceTimeArray);
	const std::vector<double> tVec(forceTimeArray.get(), forceTimeArray.get() + forceTimeArray.getSize());
	const std::vector<double> dtVec = dVector(tVec);

	//// Force applied by the ground to keep the constraint
	const int indChairForceY = forceStorage.getStateIndex("chairForce.pelvis_cnctFrame.force.Y");
	std::vector<double> chairForceYVec(tVec.size(), 0.0);
	double *chairForceYVecPtr = chairForceYVec.data();
	forceStorage.getDataColumn(indChairForceY, chairForceYVecPtr);
	std::vector<double> weightVec = expWeightVec(TAU_CHAIR_FORCE, tVec, tVec.back());
	std::transform(weightVec.begin(), weightVec.end(), dtVec.begin(), weightVec.begin(), std::multiplies<double>());

	const double costChairForce = fabs(std::inner_product(chairForceYVec.begin(), chairForceYVec.end(), weightVec.begin(), 0.0,
	                                                std::plus<double>(), [](const double chairForce, const double w){
	                                                  return w * fabs(chairForce);
	                                                }));
	return costChairForce;
}

std::vector<double> expWeightVec(const double tau, const std::vector<double> &timeVec, const double T){
  std::vector<double> result(timeVec.size());
  std::transform(timeVec.begin(), timeVec.end(), result.begin(), 
                [tau, T](const double &t){
                  return getExpWeight(tau, t, T);
                });
  return result;
}

#endif