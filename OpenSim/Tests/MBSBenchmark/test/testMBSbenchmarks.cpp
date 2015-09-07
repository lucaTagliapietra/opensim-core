#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

int main()
{
  try {
    OpenSim::Storage result1("../simplePendulum_energy_probes.sto"), standard1("../std_simplePendulum_energy_probes.sto");
    CHECK_STORAGE_AGAINST_STANDARD(result1, standard1, Array<double>(1e-6,1), __FILE__, __LINE__, "MBS_simplePendulum failed");
    std::cout << "MBS_simplePendulum failed" << std::endl;

    OpenSim::Storage result2("../nFourBarMechanism_energy_probes.sto"), standard2("../std_nFourBarMechanism_energy_probes.sto");
    CHECK_STORAGE_AGAINST_STANDARD(result2, standard2, Array<double>(1e-6,1), __FILE__, __LINE__, "MBS_nFourBarMechanism failed");
    std::cout << "MBS_nFourBarMechanism failed" << std::endl;

    OpenSim::Storage result3("../andrewsMechanism_energy_probes.sto"), standard3("../std_andrewsMechanism_energy_probes.sto");
    CHECK_STORAGE_AGAINST_STANDARD(result3, standard3, Array<double>(1e-6,1), __FILE__, __LINE__, "MBS_andrewsMechanism failed");
    std::cout << "MBS_andrewsMechanism failed" << std::endl;
    
    OpenSim::Storage result4("../bricardsMechanism_energy_probes.sto"), standard4("../std_bricardsMechanism_energy_probes.sto");
    CHECK_STORAGE_AGAINST_STANDARD(result4, standard4, Array<double>(1e-6,1), __FILE__, __LINE__, "MBS_bricardsMechanism failed");
    std::cout << "MBS_bricardsMechanism failed" << std::endl;
    
    OpenSim::Storage result5("../flyballGovernor_kinematics_S_pos.sto"), standard5("../std_flyballGovernor_kinematics_S_pos.sto");
    CHECK_STORAGE_AGAINST_STANDARD(result5, standard5, Array<double>(1e-6,3), __FILE__, __LINE__, "MBS_flyballGovernor failed");
    std::cout << "MBS_flyballGovernor failed" << std::endl;
  }
  catch (const OpenSim::Exception& e) {
    e.print(std::cerr);
    return 1;
  }
  std::cout << "Done" << std::endl;
  return 0;
}