#pragma once

#include <iostream>
#include <map>
#include <chrono>
#include <string>

class ExperimentLog {
public:
    ExperimentLog(std::string folder, std::string experimentName);
    virtual ~ExperimentLog();

    void addStartedMeasurement(long id, std::chrono::time_point<std::chrono::high_resolution_clock> start);
    void finishMeasurement(long id, std::chrono::time_point<std::chrono::high_resolution_clock> end);
    void resetStatistics();
    void calcStatistics();
    void serialise();

protected:
    std::string folder;
    std::string experimentName;
    long count;
    double max;
    double min;
    double mean;
    double stdDev;
    std::map<long, std::chrono::time_point<std::chrono::high_resolution_clock>> startedMeasurements;
    std::map<long, std::chrono::nanoseconds> finishedMeasurements;
};