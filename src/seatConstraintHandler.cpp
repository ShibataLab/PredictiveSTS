#include "seatConstraintHandler.h"

ReleaseSeatConstraint::ReleaseSeatConstraint(OpenSim::Model& m, const double threshold) : 
								TriggeredEventHandler(SimTK::Stage::Acceleration),
								_model(m), _forceThreshold(threshold){

	OpenSim::ForceSet &forceSet = _model.updForceSet();
	chairForce = &forceSet.get("chairForce");
}

// WITNESS FUNCTION
SimTK::Real ReleaseSeatConstraint::getValue(const SimTK::State& s) const {

  const OpenSim::Array<double> chairForces = chairForce->getRecordValues(s);
  double normChairForces = 0;
  for(int i=0; i<3; ++i){
    normChairForces += chairForces[i]*chairForces[i];
  }

  return normChairForces-_forceThreshold;
}

// EVENT HANDLER FUNCTION
void ReleaseSeatConstraint::handleEvent(SimTK::State& s, SimTK::Real accuracy, bool& terminate) const
{

  chairForce->setAppliesForce(s, false);
	terminate = false;
	seatReleaseTime = s.getTime(); 
	std::cout << "Releasing Seat Constraint : " << seatReleaseTime << std::endl;
}

double ReleaseSeatConstraint::getSeatRleaseTime() const {
	return seatReleaseTime;
}