#pragma once

#include "IOrbitalMechanicSolver.h"

class PatchedConicSolver: public IOrbitalMechanicSolver
{
    public:
        PatchedConicSolver()
        {
            
        }

        virtual void SolveOrbitalElement(double mu, const FSpatialState& state, FOrbitalState& orbit) override
        {
            FVector v = Physical::ToEquatorial(state.Velocity);
            FVector r = Physical::ToEquatorial(state.Position);
            FVector h = r ^ v;
            FVector N = FVector::UnitZ() ^ h;

            double r_s = r.Size();
            double h_s = h.Size();
            double N_s = FGenericPlatformMath::Max(N.Size(), 0.001);

            double v_r = (v | r) / r_s;
    
            FVector e = (v ^ h) / mu - r / r_s;
            double ecc = e.Size();

            double c = h.SizeSquared() / mu;
            double a = c * (1 / (1 - ecc * ecc));
            orbit.period = TWO_PI * a * a * sqrt(1 - ecc * ecc);
            orbit.period /= h_s;

            double inc = acos(h.Z / h_s);
            double ras = acos(N.X / N_s);
            if (N.Y < 0) {
                ras = TWO_PI - ras;
            }

            double aop = acos((N | e) / (N_s * ecc));
            if (e.Z < 0) {
                aop = TWO_PI - aop;
            }

            double tan = acos((e / ecc) | (r / r_s));
            if (v_r < 0) {
                tan = TWO_PI - tan;
            }

            orbit.ap = c * (1 + ecc);
            orbit.pa = c * (1 - ecc);

            orbit.aop = FMath::RadiansToDegrees(aop);
            orbit.tan = FMath::RadiansToDegrees(tan);
            orbit.ras = FMath::RadiansToDegrees(ras);
            orbit.inc = FMath::RadiansToDegrees(inc);
            orbit.ecc = ecc;
        }
    
        virtual void SolveSpatialStateAt(double t, FSpatialState& state, int iteration = 3) override
        {
            
        }
    
        virtual void SimulationStep(double mu, double dt, FSpatialState& state) override
        {
            FVector a_normal = GetGravitationPull(mu, state);
            
            state.Velocity += (a_normal + state.Acceleration) * dt;
            state.Position += state.Velocity * dt;
        }

        virtual FVector3d GetGravitationPull(double mu, FSpatialState& state) override
        {
            if (mu != 0) {
                return mu / state.Position.SizeSquared() * (-state.Position / state.Position.Size());
            }
            return FVector3d::Zero();
        }
    
        virtual FSpatialState SolveLocalSpatialState(IInertialFrame* ref, FOrbitalState& orbit_el) override
        {
            FQuat ras_rot = FQuat(FVector::UnitZ(), FMath::DegreesToRadians(-orbit_el.ras));

            FSpatialState state;

            state.Position = orbit_el.pa * FVector3d::UnitX();

            FVector r = ras_rot.RotateVector(-state.Position);
            double mu = Physical::GetGParam(ref->GetObjectMass(),0);
            double v = sqrt((mu * (1 + orbit_el.ecc)) / r.Size());
    

            FVector vt = r / r.Size();
            double k = 1 + (vt.Y * vt.Y) / (vt.X * vt.X);
            double px = sqrt(1 - 1 / k);
            double py = sqrt(1 / k);
    
            vt.Set(px, py, 0);
            vt *= v;

            state.Velocity.Set(vt.X, vt.Y, vt.Z);

            FQuat inc_rot = FQuat(r / r.Size(), FMath::DegreesToRadians(orbit_el.inc));
            state.Velocity = inc_rot.RotateVector(-state.Velocity);
            state.Position = -r;
            state.Acceleration = FVector3d::Zero();

            return state;
        }
};