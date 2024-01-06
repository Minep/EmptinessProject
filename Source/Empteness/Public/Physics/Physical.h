#pragma once

#include "Math/MathFwd.h"

#include "Physical.generated.h"

// G constant in km^3.s^-2.M_earth^-1
#define G_CONST 4.002E4


USTRUCT()
struct FSpatialState {
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, Meta = (DisplayName = "Position"))
    FVector Position;

    UPROPERTY(VisibleAnywhere, Meta = (DisplayName = "Velocity"))
    FVector Velocity;

    UPROPERTY(VisibleAnywhere, Meta = (DisplayName = "Acceleration"))
    FVector Acceleration;

    FSpatialState operator+(const FSpatialState& b) const
    {
        FSpatialState out;
        out.Position = Position + b.Position;
        out.Velocity = Velocity + b.Velocity;
        out.Acceleration = Acceleration + b.Acceleration;

        return out;
    }

    FSpatialState operator-(const FSpatialState& b) const
    {
        FSpatialState out;
        out.Position = Position - b.Position;
        out.Velocity = Velocity - b.Velocity;
        out.Acceleration = Acceleration - b.Acceleration;

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

#define ASTRO_MASS 6e24
#define ASTRO_LENGTH 1e3
#define ASTRO_FORCE 6e27
#define NEGLIGIBLE_MAGNITUDE 1e6

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

        enum EQuantity
        {
            // Unit representing a force (SI: kg.m.s^-2)
            Force,
            // Unit representing mass (SI: kg)
            Mass,
            // Unit representing length (SI: m)
            Length
        };

        /**
         * Convert the standard scaled value (as within SI unit) to the astronomic
         * scaled value which is used to simulate orbital mechanic.
         * @param StandardScale 
         * @return 
         */
        static double ToAstroScale(double StandardScale, EQuantity type)
        {
            double Base = ASTRO_FORCE;
            switch (type) {
                case Force:
                    Base = ASTRO_FORCE;
                    break;
                case Mass:
                    Base = ASTRO_MASS;
                    break;
                case Length:
                    Base = ASTRO_LENGTH;
                    break;
            }
            
            if (abs(StandardScale) <= (Base / NEGLIGIBLE_MAGNITUDE)) {
                return 0;
            }
            return StandardScale / Base;
        }

        template<class T>
        static T ToMetricScale(T AstroScale, EQuantity type)
        {
            switch (type) {
                case Force:
                    return AstroScale * ASTRO_FORCE;
                case Mass:
                    return AstroScale * ASTRO_MASS;
                case Length:
                    return AstroScale * ASTRO_LENGTH;
            }
        }

        static FVector3d ToAstroScale(FVector3d StandardScaledVec, EQuantity type)
        {
            double Base = ASTRO_FORCE;
            switch (type) {
                case Force:
                    Base = ASTRO_FORCE;
                break;
                case Mass:
                    Base = ASTRO_MASS;
                break;
                case Length:
                    Base = ASTRO_LENGTH;
                break;
            }
            
            if (StandardScaledVec.Size() <= (Base / NEGLIGIBLE_MAGNITUDE)) {
                return FVector3d::Zero();
            }
            return StandardScaledVec / Base;
        }
};
