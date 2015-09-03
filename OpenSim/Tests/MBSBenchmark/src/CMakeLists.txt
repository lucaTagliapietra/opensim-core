# Authors: Luca Tagliapietra, Elena Ceseracciu, and Monica Reggiani

OpenSimAddTests(
    TESTPROGRAMS testMBSsimplePendulumRuns.cpp
    LINKLIBS osimSimulation  osimAnalyses osimTools
    )  

    #add_executable(SimplePendulumExample A01_SimplePendulum.cpp)
    #target_link_libraries(SimplePendulumExample osimSimulation osimAnalyses osimTools)

#    add_executable(NFourBarMechanismExample A02_NFourBarMechanism.cpp)
#    target_link_libraries(NFourBarMechanismExample osimSimulation)

#    add_executable(AndrewsMechanismExample A03_AndrewsMechanism.cpp)
#    target_link_libraries(AndrewsMechanismExample osimSimulation)

#    add_executable(BricardsMechanismExample A04_BricardsMechanism.cpp)
#    target_link_libraries(BricardsMechanismExample osimSimulation)

#    add_executable(FlyballGovernorExample A05_FlyballGovernor.cpp)
#    target_link_libraries(FlyballGovernorExample osimSimulation)
