#pragma once

#include "Orbital/InertialFrame.h"
#include "Orbital/Physical.h"

class ISolver {
    public:
        virtual void SolveOrbitalElement(double mu, const FSpatialState& state, FOrbitalState& orbit);
        virtual void SolveSpatialStateAt(double t, FSpatialState& state, int iteration = 3);
        virtual void SimulationStep(double mu, double dt, FSpatialState& state);
		virtual FSpatialState SolveLocalSpatialState(IInertialFrame* ref, FOrbitalState& orbit_el);
};

