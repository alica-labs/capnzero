#include "ExperimentLog.h"

#include <essentials/FileSystem.h>
//#include <sys/stat.h> // used for folder permissions

#include <algorithm>
#include <iostream>
#include <fstream>
#include <cmath>

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
        this->startedMeasurements.erase(mapEntry);
    } else {
        std::cerr << "ExperimentLog: Received measurement whose ID (" << id << ") is unknown!" << std::endl;
    }
}

void ExperimentLog::calcStatistics()
{
    this->resetStatistics();
    double M2 = 0;

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
        double delta = mapEntry.second.count() - mean; // (value - oldMean)
        mean += delta / receivedMsgs; // (value - oldMean) / (oldMessages + 1)
        double delta2 = mapEntry.second.count() - mean; // (value - newMean)
        M2 += delta * delta2; // (value - oldMean) * (value - newMean)
    }
    this->stdDev = sqrt(M2 / receivedMsgs);

    // missed messages
    this->missedMsgs = this->startedMeasurements.size();
}

void ExperimentLog::reset() {
    this->startedMeasurements.clear();
    this->finishedMeasurements.clear();
    this->resetStatistics();
}

void ExperimentLog::resetStatistics()
{
    this->receivedMsgs = 0;
    this->missedMsgs = 0;
    this->max = 0;
    this->min = std::numeric_limits<double>::max();
    this->mean = 0;
    this->stdDev = 0;
}

void ExperimentLog::serialise(std::string prefix)
{
    std::ofstream fileWriter;
//    essentials::FileSystem::createDirectory(this->folder, S_IRWXU); // setting the rights of the folder did not work for me :(
    fileWriter.open(essentials::FileSystem::combinePaths(this->folder, "CapnZeroEval.csv"), std::ios_base::app);
    fileWriter << std::fixed << experimentName << "\t" << prefix << "\t" << this->mean << "\t" << this->stdDev <<  "\t" << this->min << "\t" << this->max << "\t" << this->receivedMsgs << "\t" << this->missedMsgs << std::endl;
}
