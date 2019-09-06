#include "ExperimentLog.h"

#include <algorithm>
#include <iostream>
#include <fstream>

ExperimentLog::ExperimentLog(std::string folder, std::string experimentName)
        : folder(folder)
        , experimentName(experimentName)
{
    this->resetStatistics();
}

ExperimentLog::~ExperimentLog() {}

void ExperimentLog::addStartedMeasurement(long id, std::chrono::time_point<std::chrono::high_resolution_clock> start)
{
    this->startedMeasurements.emplace(id, start);
}

void ExperimentLog::finishMeasurement(long id, std::chrono::time_point<std::chrono::high_resolution_clock> end)
{
    auto mapEntry = this->startedMeasurements.find(id);
    if (mapEntry != this->startedMeasurements.end()) {
        std::chrono::nanoseconds measuredDuration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - mapEntry->second);
        this->finishedMeasurements.emplace(id, measuredDuration);
    } else {
        std::cerr << "ExperimentLog: Received measurement whose ID (" << id << ") is unknown!" << std::endl;
    }
}

void ExperimentLog::calcStatistics()
{
    this->resetStatistics();
    double M2 = 0; // (value - oldMean) * (value - newMean)

    for (auto mapEntry : this->finishedMeasurements) {
        // max
        if (mapEntry.second.count() > max) {
            max = mapEntry.second.count();
        }

        // min
        if (mapEntry.second.count() < min) {
            min = mapEntry.second.count();
        }

        // mean & stdDev
        receivedMsgs += 1;
        double delta = mapEntry.second.count() - mean;
        mean += delta / receivedMsgs; // (value - oldMean) / (oldMessages + 1)
        double delta2 = mapEntry.second.count() - mean;
        M2 += delta * delta2;
    }
    this->stdDev = M2 / receivedMsgs;

    // missed messages
    this->missedMsgs = this->startedMeasurements.size();
}

void ExperimentLog::resetStatistics()
{
    this->receivedMsgs = 0;
    this->max = 0;
    this->min = std::numeric_limits<double>::max();
    this->mean = 0;
    this->stdDev = 0;
}

void ExperimentLog::serialise()
{
    std::ofstream fileWriter;
    fileWriter.open(std::string(this->folder + "/CapnZeroEval.csv"), std::ios_base::app);
    fileWriter << experimentName << "\t" << this->mean << "\t" << this->stdDev <<  "\t" << this->min << "\t" << this->max << "\t" << this->receivedMsgs << "\t" << this->missedMsgs << std::endl;
    fileWriter.flush();
    fileWriter.close();
}