#pragma once

#include "Physics/InertialFrame.h"
#include "Physics/Physical.h"

class IOrbitalMechanicSolver {
    public:
        virtual void SolveOrbitalElement(double mu, const FSpatialState& state, FOrbitalState& orbit) = 0;
        virtual void SolveSpatialStateAt(double t, FSpatialState& state, int iteration = 3) = 0;
        virtual void SimulationStep(double mu, double dt, FSpatialState& state) = 0;
		virtual FSpatialState SolveLocalSpatialState(IInertialFrame* ref, FOrbitalState& orbit_el) = 0;
		virtual FVector3d GetGravitationPull(double mu, FSpatialState& state) = 0;
};

