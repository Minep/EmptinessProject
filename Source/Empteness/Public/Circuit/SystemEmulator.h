#pragma once

class CircuitSystemEmulator
{
    double DesignatedDuty = 0.0;
    double DesignatedFreq = 0.0;

public:
    double GetDesignatedPluseWidth() const
    {
        return DesignatedDuty;
    }
};