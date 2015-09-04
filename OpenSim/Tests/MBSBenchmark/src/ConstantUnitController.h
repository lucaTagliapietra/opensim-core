// This is part of
// Multi-Body Systems Benchmark in OpenSim (MBS-BOS)
// Copyright (C) 2013-2015 Luca Tagliapietra, Michele Vivian, Elena Ceseracciu, and Monica Reggiani
//
// MBS-BOS is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MBS-BOS is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MBS-BOS.  If not, see <http://www.gnu.org/licenses/>.
//
// The authors may be contacted via:
// email: tagliapietra@gest.unipd.it

#include <OpenSim/OpenSim.h>
#include <cmath>

using namespace OpenSim;
using namespace SimTK;

class ConstantUnitController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(ConstantUnitController, Controller);
// This section contains methods that can be called in this controller class.

public:
  ConstantUnitController():Controller(){};

  /**
    * This function is called at every time step for every actuator.
    *
    * @param s Current state of the system
    * @param index Index of the current actuator whose control is being calculated
    * @return Control value to be assigned to the current actuator at the current time
    */
  virtual void computeControls( const SimTK::State& s, SimTK::Vector &controls ) const {
    for (int i = 0; i<controls.size(); ++i)
      controls[i] = 1.0;
  }
};
