# Authors: Luca Tagliapietra, Elena Ceseracciu, and Monica Reggiani

# Define project
project (MBSBenchmarks)

add_subdirectory(src)

set (data_files    data/AndrewsMechanism.osim
                                data/BricardMechanism.osim
                                data/FlyballGovernor.osim
                                data/FourBarMechanism.osim
                                data/SimplePendulumMechanism.osim
                                data/Geometry/block.vtp
                                data/Geometry/box.vtp
                                data/Geometry/cylinder.vtp
                                data/Geometry/sphere.vtp
                                data/Geometry/triangle.vtp)

file(COPY ${data_files} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
