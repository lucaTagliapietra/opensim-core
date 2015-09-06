# Authors: Luca Tagliapietra, Elena Ceseracciu, and Monica Reggiani

# Define project
project (MBSBenchmarks)

add_subdirectory(src)
add_subdirectory(test)

set (data_files data/AndrewsMechanism.osim
                data/BricardMechanism.osim
                data/FlyballGovernor.osim
                data/FourBarMechanism.osim
                data/SimplePendulumMechanism.osim
                data/Geometry/block.vtp
                data/Geometry/box.vtp
                data/Geometry/cylinder.vtp
                data/Geometry/sphere.vtp
                data/Geometry/triangle.vtp
                data/std_simplePendulum_energy_energyReporter_probes.sto
                data/std_nFourBarMechanism_energy_energyReporter_probes.sto
                data/std_andrewsMechanism_energy_energyReporter_probes.sto
                data/std_bricardsMechanism_energy_energyReporter_probes.sto
                data/std_flyballGovernor_kinematics_pointKinematicsReporter_NONAME_pos.sto
                )

file(COPY ${data_files} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
