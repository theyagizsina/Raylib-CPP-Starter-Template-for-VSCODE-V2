#pragma once
#include <fstream>
#include <string>

class DataLogger {
private:
    std::ofstream csvFile;
    bool isOpen;

public:
    DataLogger();
    ~DataLogger();

    bool initialize(const std::string& filename);
    void writeHeader();
    void logFlightData(double time, double X, double Y, double Z,
                      double rollDeg, double pitchDeg, double yawDeg,
                      double alphaDeg, double betaDeg);
    void close();
};