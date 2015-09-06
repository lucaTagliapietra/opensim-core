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
#include "ConstantUnitController.h"


int main(int argc, char **argv) {
  cout << "--------------------------------------------------------------------------------" << endl;
  cout << " Multi-Body System Benchmark in OpenSim" << endl;
  cout << " Benchmark reference url: http://lim.ii.udc.es/mbsbenchmark/" << endl;
  cout << " Problem A03: Andrew's Mechanism Simulator" << endl;
  cout << " Copyright (C) 2013-2015 Luca Tagliapietra, Michele Vivian, Elena Ceseracciu, and Monica Reggiani" << endl;
  cout << "--------------------------------------------------------------------------------" << endl;

  //Model additional parameters
  const SimTK::Vec3 springInsertionPointInGround(0.014,0.072,0);
  const SimTK::Vec3 springInsertionPointInBDEelement(-0.035/2+0.018, 0.02,0);
  const double springK = 4530.0;
  const double springRestLength = 0.07785;
  const double motorTorque = 0.033;

  //Simulation Parameters
  const double initialTime = 0.0;
  const double finalTime = 0.05;
  const double accuracy = 1.0e-8;
  const double tolerance = 1.0e-8;
  const double minStepSize = 1.0e-5;
  const double maxStepSize = 1.0e-4;
  const double internalStepLimit = 2.0e5;
  const double reportingStep = 2.5e-4;

  try {
    // Load the Opensim Model
    OpenSim::Model andrewsMechanism("../AndrewsMechanism.osim");

    // Add Point to Point Spring to the model
    OpenSim::PointToPointSpring* spring = new OpenSim::PointToPointSpring( andrewsMechanism.getGround(),springInsertionPointInGround,
            andrewsMechanism.getBodySet().get(std::string("BDE")), springInsertionPointInBDEelement, springK, springRestLength);
    andrewsMechanism.addModelComponent(spring);

    // Add motor and an unitary constant controller
    SimTK::Vec3 pointA(0,0,0);
    SimTK::Vec3 pointB(0,0,1);
    SimTK::Vec3 axis = pointA - pointB;
    OpenSim::TorqueActuator *motor = new OpenSim::TorqueActuator(andrewsMechanism.getGround(), andrewsMechanism.getBodySet().get(std::string("OF")),axis, true);
    motor->setName("motor");
    motor->set_optimal_force(motorTorque);
    andrewsMechanism.addForce(motor);

    int indexMotor = andrewsMechanism.getActuators().getIndex("motor");
    ConstantUnitController *constController = new ConstantUnitController();
    constController->setName("Constant Controller");
    constController->setActuators(andrewsMechanism.getActuators());
    andrewsMechanism.addController(constController);

    // Add System Energy Reporter
    OpenSim::SystemEnergyProbe *energyProbe = new OpenSim::SystemEnergyProbe(true, true);
    energyProbe->setName("ener");
    energyProbe->setGain(1.0);
    energyProbe->setOperation("value");
    energyProbe->setComputeKineticEnergy(true);
    energyProbe->setComputePotentialEnergy(true);
    andrewsMechanism.addProbe(energyProbe);
    OpenSim::ProbeReporter *energyReporter = new OpenSim::ProbeReporter(&andrewsMechanism);
    energyReporter->setName(std::string("energyReporter"));
    std::cout << energyReporter->getName() << std::endl;
    andrewsMechanism.addAnalysis(energyReporter);

    //Initialize System State
    SimTK::State initialState = andrewsMechanism.initSystem();
    andrewsMechanism.getMultibodySystem().realize(initialState, SimTK::Stage::Position);
    andrewsMechanism.getMultibodySystem().realize(initialState, SimTK::Stage::Velocity);
    andrewsMechanism.getMultibodySystem().realize(initialState, SimTK::Stage::Acceleration);
    andrewsMechanism.getMultibodySystem().realize(initialState, SimTK::Stage::Report);

    //Create Integrator and set its parameters
    SimTK::Integrator* integrator = new SimTK::RungeKuttaFeldbergIntegrator(andrewsMechanism.getMultibodySystem());
    integrator->setMaximumStepSize(maxStepSize);
    integrator->setAccuracy(accuracy);
    integrator->setConstraintTolerance(tolerance);
    integrator->setProjectEveryStep(true);
    integrator->setInternalStepLimit(internalStepLimit);

    // Create the manager for the integrator and set its parameters
    OpenSim::Manager manager(andrewsMechanism, *integrator);
    manager.setInitialTime(initialTime);
    manager.setFinalTime(finalTime);

    // Perform the integration
    manager.integrate(initialState);

    // Save simulation results
    OpenSim::IO::SetPrecision(15);
    andrewsMechanism.getMultibodySystem().realize(initialState, SimTK::Stage::Report);
    andrewsMechanism.updAnalysisSet().get("energyReporter").printResults("andrewsMechanism_energy", "../", reportingStep);
  }
  catch (const std::exception& ex){
    std::cerr << ex.what() << std::endl;
    return 1;
  }
  catch (...){
    std::cerr << "UNRECOGNIZED EXCEPTION" << std::endl;
    return 1;
  }
  std::cout << "Simulation successfully completed." << std::endl;
  return 0;
}
