#pragma once

#include "Math/MathFwd.h"

#include "Physical.generated.h"

// G constant in km^3.s^-2.M_earth^-1
#define G_CONST 4.002E4

USTRUCT()
struct FSpatialState {
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, Meta = (DisplayName = "Position"))
    FVector position;

    UPROPERTY(VisibleAnywhere, Meta = (DisplayName = "Velocity"))
    FVector velocity;

    UPROPERTY(VisibleAnywhere, Meta = (DisplayName = "Acceleration"))
    FVector acceleration;

	FSpatialState operator+(const FSpatialState& b) const
	{
		FSpatialState out;
		out.position = position + b.position;
		out.velocity = velocity + b.velocity;
		out.acceleration = acceleration + b.acceleration;

		return out;
	}

	FSpatialState operator-(const FSpatialState& b) const
	{
		FSpatialState out;
		out.position = position - b.position;
		out.velocity = velocity - b.velocity;
		out.acceleration = acceleration - b.acceleration;

		return out;
	}
};

USTRUCT()
struct FOrbitalState {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, Meta = (DisplayName = "Inclination"))
	// inclination
	double inc;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Ascension"))
	// right ascension
	double ras;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Eccentricity"))
	// eccentricity
	double ecc;

	UPROPERTY(VisibleAnywhere, Meta = (DisplayName = "Perigee Argument"))
	// arugument of perigee
	double aop;

	UPROPERTY(VisibleAnywhere, Meta = (DisplayName = "True Anomaly"))
	// true anomaly
	double tan;

	UPROPERTY(VisibleAnywhere, Meta = (DisplayName = "Apoapsis"))
	// apoapsis
	double ap;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Periapsis"))
	// periapsis
	double pa;

	UPROPERTY(VisibleAnywhere, Meta = (DisplayName = "Orbital Period"))
	double period;
};

class Physical {
    public:
        static FMatrix44d U2E;
        static FMatrix44d E2U;
        
        static FVector ToEquatorial(const FVector& v);
        static FVector FromEquatorial(const FVector& v);
        static inline double GetGParam(double mass1, double mass2) {
            return G_CONST * (mass1 + mass2);
        }

        static inline double GetSOIRadius(double m, double min_force) {
            return sqrt(G_CONST * m / min_force);
        }
};
