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

#include <iostream>
using std::cout;
using std::endl;

#include "OpenSim/OpenSim.h"
#include "OpenSim/Simulation/Model/SystemEnergyProbe.h"

int main(int argc, char **argv) {
  cout << "--------------------------------------------------------------------------------" << endl;
  cout << " Multi-Body System Benchmark in OpenSim" << endl;
  cout << " Benchmark reference url: http://lim.ii.udc.es/mbsbenchmark/" << endl;
  cout << " Problem A01: Simple Pendulum Mechanism Simulator" << endl;
  cout << " Copyright (C) 2013-2015 Luca Tagliapietra, Michele Vivian, Elena Ceseracciu, and Monica Reggiani" << endl;
  cout << "--------------------------------------------------------------------------------" << endl;

  //Simulation Parameters
  const double initialTime = 0.0;
  const double finalTime = 10.0;
  const double accuracy = 1.0e-12;
  const double tolerance = 1.0e-9;
  const double minStepSize = 1.0e-6;
  const double maxStepSize = 1.0e-3;
  const double internalStepLimit = 2.0e6;
  const double reportingStep = 1.0e-2;

  // Load the Opensim Model
  OpenSim::Model simplePendulumModel("../SimplePendulumMechanism.osim");

  // Add System Energy Reporter
  OpenSim::SystemEnergyProbe *energyProbe = new OpenSim::SystemEnergyProbe(true, true);
  energyProbe->setName("SystemEnergy");
  energyProbe->setGain(1.0);
  energyProbe->setOperation("value");
  energyProbe->setComputeKineticEnergy(true);
  energyProbe->setComputePotentialEnergy(true);
  simplePendulumModel.addProbe(energyProbe);
  OpenSim::ProbeReporter *energyReporter = new OpenSim::ProbeReporter(&simplePendulumModel);
  energyReporter->setName(std::string("energyReporter"));
  std::cout << energyReporter->getName() << std::endl;
  simplePendulumModel.addAnalysis(energyReporter);
  simplePendulumModel.updAnalysisSet().get("energyReporter").setOn(true);

  //Initialize System State
  SimTK::State initialState = simplePendulumModel.initSystem();
  simplePendulumModel.getMultibodySystem().realize(initialState, SimTK::Stage::Position);
  simplePendulumModel.getMultibodySystem().realize(initialState, SimTK::Stage::Velocity);
  simplePendulumModel.getMultibodySystem().realize(initialState, SimTK::Stage::Acceleration);
  simplePendulumModel.getMultibodySystem().realize(initialState, SimTK::Stage::Report);

  //Create Integrator and set its parameters
  SimTK::Integrator* integrator = new SimTK::CPodesIntegrator(simplePendulumModel.getMultibodySystem(), SimTK::CPodes::BDF);
  integrator->setMaximumStepSize(maxStepSize);
  integrator->setAccuracy(accuracy);
  integrator->setConstraintTolerance(tolerance);
  integrator->setProjectEveryStep(true);
  integrator->setInternalStepLimit(internalStepLimit);

  // Create the manager for the integrator and set its parameters
  OpenSim::Manager manager(simplePendulumModel, *integrator);
  manager.setInitialTime(initialTime);
  manager.setFinalTime(finalTime);

  // Perform the integration
  manager.integrate(initialState);
  
  // Save simulation results
  OpenSim::IO::SetPrecision(15);
  simplePendulumModel.getMultibodySystem().realize(initialState, SimTK::Stage::Report);
  simplePendulumModel.updAnalysisSet().get("energyReporter").printResults("simplePendulum_energy", "../", reportingStep);
 }
