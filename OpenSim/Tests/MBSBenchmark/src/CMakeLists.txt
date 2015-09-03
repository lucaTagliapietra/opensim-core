# Authors: Luca Tagliapietra, Elena Ceseracciu, and Monica Reggiani

OpenSimAddTests(
    TESTPROGRAMS testMBS_simplePendulumRuns.cpp
    LINKLIBS osimSimulation osimAnalyses osimTools
    )  

OpenSimAddTests(
    TESTPROGRAMS testMBS_nFourBarMechanismRuns.cpp
    LINKLIBS osimSimulation osimAnalyses osimTools
    )  

OpenSimAddTests(
    TESTPROGRAMS testMBS_andrewsMechanismRuns.cpp
    LINKLIBS osimSimulation osimAnalyses osimTools
    )  

OpenSimAddTests(
    TESTPROGRAMS testMBS_bricardsMechanismRuns.cpp
    LINKLIBS osimSimulation osimAnalyses osimTools
    )  

OpenSimAddTests(
    TESTPROGRAMS testMBS_flyballGovernorRuns.cpp
    LINKLIBS osimSimulation osimAnalyses osimTools
    )  

