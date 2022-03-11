#pragma once
#include "OpenSim/OpenSim.h"
#include "universalConsts.h"
/*
Disables the seat constraint if
1) the vertical component of chair force turns non compressive, or 
2) the slip occurs i.e. fabs(Fx) > mu * Fy
*/
class ReleaseSeatConstraint : public SimTK::TriggeredEventHandler {
public:

  ReleaseSeatConstraint(OpenSim::Model &m, const double threshold);
  SimTK::Real getValue(const SimTK::State& s) const;
  void handleEvent(SimTK::State& s, SimTK::Real accuracy, bool& terminate) const;

  // Returns -1 if seat off doesn't take place
  double getSeatRleaseTime() const ;

private:
  OpenSim::Model &_model;
  const double _forceThreshold;
  mutable double seatReleaseTime = -1.0;
  OpenSim::Force *chairForce; 
};