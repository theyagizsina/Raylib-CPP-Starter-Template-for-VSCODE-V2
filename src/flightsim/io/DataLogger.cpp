#include "DataLogger.h"

DataLogger::DataLogger() : isOpen(false)
{
}

DataLogger::~DataLogger()
{
    close();
}

bool DataLogger::initialize(const std::string& filename)
{
    csvFile.open(filename);
    if (csvFile.is_open()) {
        isOpen = true;
        writeHeader();
        return true;
    }
    return false;
}

void DataLogger::writeHeader()
{
    if (isOpen) {
        csvFile << "time,X,Y,Z,rollDeg,pitchDeg,yawDeg,alphaDeg,betaDeg\n";
    }
}

void DataLogger::logFlightData(double time, double X, double Y, double Z,
                              double rollDeg, double pitchDeg, double yawDeg,
                              double alphaDeg, double betaDeg)
{
    if (isOpen) {
        csvFile << time << "," << X << "," << Y << "," << Z <<
                  "," << rollDeg << "," << pitchDeg << "," << yawDeg <<
                  "," << alphaDeg << "," << betaDeg << "\n";
    }
}

void DataLogger::close()
{
    if (isOpen) {
        csvFile.close();
        isOpen = false;
    }
}