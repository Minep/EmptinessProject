#pragma once
#include "MatrixTypes.h"
#include "InertiaTensor.generated.h"

USTRUCT()
struct FMassDistribution
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, DisplayName="Position")
    FVector Position;

    UPROPERTY(EditAnywhere, DisplayName="Mass")
    double Mass;
};

struct FInertiaTensor
{
private:
    UE::Geometry::FMatrix3d Tensor;
    UE::Geometry::FMatrix3d TensorInv;

    double Mass = 0;
    FVector3d CenterOfMass = FVector3d::Zero();

protected:
    void UpdateCenterOfMass(FVector3d& NewCenterOfMass)
    {
        const auto DeltaC = CenterOfMass - NewCenterOfMass;
        const double DX = DeltaC.X, DY = DeltaC.Y, DZ = DeltaC.Z; 
        const double Dxx = Mass * (DY * DY + DZ * DZ),
                     Dyy = Mass * (DX * DX + DZ * DZ),
                     Dzz = Mass * (DX * DX + DY * DY),
                     Dxy = -Mass * DX * DY,
                     Dxz = -Mass * DX * DZ,
                     Dyz = -Mass * DY * DZ;

        const UE::Geometry::FMatrix3d InertiaShifter(Dxx, Dxy, Dxz,
                                                     Dxy, Dyy, Dyz,
                                                     Dxz, Dyz, Dzz);

        Tensor += InertiaShifter;
        CenterOfMass.Set(NewCenterOfMass.X, NewCenterOfMass.Y, NewCenterOfMass.Z);
    }
    
public:
    enum ESolidTopology
    {
        SolidCylinder,
        CylindricalShell,
        RectParallelepiped
    };
public:
    FInertiaTensor()
    {
        
    }
    
    FInertiaTensor(TArray<FMassDistribution>& MassDistribution)
    {
        for (auto& md : MassDistribution) {
            Mass += md.Mass;
            CenterOfMass += md.Position * md.Mass;
        }

        CenterOfMass /= Mass;
        
        UE::Geometry::FMatrix3d PartialInertia = UE::Geometry::FMatrix3d::Zero(); 
        for (auto& md : MassDistribution) {
            const double m = md.Mass;
            const FVector3d p = md.Position - CenterOfMass;
        
            const double Ixx = m * (p.Y * p.Y + p.Z * p.Z);
            const double Iyy = m * (p.X * p.X + p.Z * p.Z);
            const double Izz = m * (p.X * p.X + p.Y * p.Y);
            const double Ixy = m * (-p.X * p.Y);
            const double Ixz = m * (-p.X * p.Z);
            const double Iyz = m * (-p.Y * p.Z);

            PartialInertia.Row0.Set(Ixx, Ixy, Ixz);
            PartialInertia.Row1.Set(Ixy, Iyy, Iyz);
            PartialInertia.Row2.Set(Ixz, Iyz, Izz);

            Tensor += PartialInertia;
        }
        
        TensorInv = Tensor.Inverse();
    }

    FInertiaTensor(const FVector3d& Bound, const double Mass, const ESolidTopology Topology)
    {
        this->Mass = Mass;
        double Ixx = 0, Iyy = 0, Izz = 0;
        switch (Topology) {
            case SolidCylinder:
                {
                    const double R = Bound.X, L = Bound.Y;
                    Ixx = Mass / 12 * (3 * R * R + L * L);
                    Izz = Mass * R * R / 2;
                }
            break;
            case CylindricalShell:
                {
                    const double R = Bound.X, L = Bound.Y;
                    Ixx = Mass / 12 * (6 * R * R + L * L);
                    Izz = Mass * R * R;
                }
            break;
            case RectParallelepiped:
                {
                    const double A2 = Bound.X * Bound.X,
                                 B2 = Bound.Y * Bound.Y,
                                 L2 = Bound.Z * Bound.Z;
                    Ixx = Mass * (A2 + L2) / 12;
                    Iyy = Mass * (B2 + L2) / 12;
                    Izz = Mass * (B2 + A2) / 12;
                }
            break;
        }
            
        Tensor.Row0.Set( Ixx,  0,   0  );
        Tensor.Row1.Set(  0,  Iyy,  0  );
        Tensor.Row2.Set(  0,   0,  Izz );

        TensorInv = Tensor.Inverse();
    }

    void MergeWith(const FInertiaTensor& Guest, const FTransform& HostFrame, const FTransform& GuestFrame)
    {
        auto Point = HostFrame.InverseTransformPosition(GuestFrame.TransformPosition(Guest.CenterOfMass));
        auto Cp = (Guest.Mass * Point + Mass * CenterOfMass) / (Mass + Guest.Mass);
        
        UpdateCenterOfMass(Cp);
        
        Point = GuestFrame.InverseTransformPosition(HostFrame.TransformPosition(Cp));
        Point = Guest.CenterOfMass - Point;
        const double X = Point.X, Y = Point.Y, Z = Point.Z; 
        const double Dxx = Guest.Mass * (Y * Y + Z * Z),
                     Dyy = Guest.Mass * (X * X + Z * Z),
                     Dzz = Guest.Mass * (X * X + Y * Y),
                     Dxy = -Guest.Mass * X * Y,
                     Dxz = -Guest.Mass * X * Z,
                     Dyz = -Guest.Mass * Y * Z;

        const UE::Geometry::FMatrix3d InertiaShifter(Dxx, Dxy, Dxz,
                                                     Dxy, Dyy, Dyz,
                                                     Dxz, Dyz, Dzz);
        
        Mass = Mass + Guest.Mass;
        Tensor += Guest.Tensor + InertiaShifter;
        TensorInv = Tensor.Inverse();
    }

    void StripFor(const FInertiaTensor& Guest, const FTransform& HostFrame, const FTransform& GuestFrame)
    {
        auto Point = GuestFrame.InverseTransformPosition(HostFrame.TransformPosition(CenterOfMass));
        const double X = Point.X, Y = Point.Y, Z = Point.Z; 
        const double Dxx = Guest.Mass * (Y * Y + Z * Z),
                     Dyy = Guest.Mass * (X * X + Z * Z),
                     Dzz = Guest.Mass * (X * X + Y * Y),
                     Dxy = -Guest.Mass * X * Y,
                     Dxz = -Guest.Mass * X * Z,
                     Dyz = -Guest.Mass * Y * Z;

        const UE::Geometry::FMatrix3d InertiaShifter(Dxx, Dxy, Dxz,
                                                     Dxy, Dyy, Dyz,
                                                     Dxz, Dyz, Dzz);

        Tensor = Tensor - (Guest.Tensor + InertiaShifter);

        Point = HostFrame.InverseTransformPosition(GuestFrame.TransformPosition(Guest.CenterOfMass));
        auto Cp = (Mass * CenterOfMass - Guest.Mass * Point) / (Mass - Guest.Mass);

        ensureMsgf(Mass > 0, TEXT("Invalid inertia tensor state: negative mass!"));
        Mass = Mass - Guest.Mass;

        UpdateCenterOfMass(Cp);
        TensorInv = Tensor.Inverse();
    }

    FVector3d GetAngularMomentum(const FVector3d& AngularVelocity) const
    {
        return Tensor * AngularVelocity;
    }

    FVector3d GetAngularVelocity(const FVector3d& AngularMomentum) const
    {
        return TensorInv * AngularMomentum;
    }

    FVector3d GetTorque(const FVector3d& Force, const FVector3d& Position) const
    {
        return (Position - CenterOfMass) ^ Force;
    }

    FVector3d InverseTransform(const FVector3d& Vec, const FTransform& T) const
    {
        return T.TransformVectorNoScale(TensorInv * T.InverseTransformVectorNoScale(Vec));
    }

    void ChangeMassDistribution(const FVector3d& Position, double DeltaMass)
    {
        // Step 1: Remove the contribution of this point mass from tensor
        const double m = DeltaMass;
        const FVector3d p = Position - CenterOfMass;
        
        const double Ixx = m * (p.Y * p.Y + p.Z * p.Z);
        const double Iyy = m * (p.X * p.X + p.Z * p.Z);
        const double Izz = m * (p.X * p.X + p.Y * p.Y);
        const double Ixy = m * (-p.X * p.Y);
        const double Ixz = m * (-p.X * p.Z);
        const double Iyz = m * (-p.Y * p.Z);

        const UE::Geometry::FMatrix3d TensorChange(Ixx, Ixy, Ixz,
                                                   Ixy, Iyy, Iyz,
                                                   Ixz, Iyz, Izz);

        Tensor += TensorChange;

        // Step 2: Shift the center of mass
        auto Cp = CenterOfMass * Mass + DeltaMass * Position; 
        Mass += DeltaMass;
        Cp /= Mass;

        // Step 3: Since the center of mass is changed, so as the principle axis
        //           We must correct the inertial tensor to reflect this.
        //         Which follows immediate from the parallel axis theorem.
        UpdateCenterOfMass(Cp);
    }

    static FInertiaTensor MergeTwo(const FInertiaTensor& Dest, const FTransform& DestFrame,
                                   const FInertiaTensor& Source, const FTransform& SourceFrame)
    {
        FInertiaTensor Copy = Dest;
        Copy.MergeWith(Source, DestFrame, SourceFrame);

        return Copy;
    }

    const FVector3d& GetCenterOfMass() const
    {
        return CenterOfMass;
    }

    double GetMass() const
    {
        return Mass;
    }
};
