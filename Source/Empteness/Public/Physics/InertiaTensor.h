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
        UE::Geometry::FMatrix3d InertiaShifter;

        ComputePartialTensor(InertiaShifter, DeltaC, Mass);

        Tensor += InertiaShifter;
        CenterOfMass.Set(NewCenterOfMass.X, NewCenterOfMass.Y, NewCenterOfMass.Z);
    }

    static void ComputePartialTensor(UE::Geometry::FMatrix3d& PartialTensorOut, const FVector3d& Pos, const double Mass)
    {
        const double Ixx = Mass * (Pos.Y * Pos.Y + Pos.Z * Pos.Z);
        const double Iyy = Mass * (Pos.X * Pos.X + Pos.Z * Pos.Z);
        const double Izz = Mass * (Pos.X * Pos.X + Pos.Y * Pos.Y);
        const double Ixy = Mass * (-Pos.X * Pos.Y);
        const double Ixz = Mass * (-Pos.X * Pos.Z);
        const double Iyz = Mass * (-Pos.Y * Pos.Z);

        PartialTensorOut.Row0.Set(Ixx, Ixy, Ixz);
        PartialTensorOut.Row1.Set(Ixy, Iyy, Iyz);
        PartialTensorOut.Row2.Set(Ixz, Iyz, Izz);
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
        for (auto& MD : MassDistribution) {
            const double M = MD.Mass;
            const FVector3d P = MD.Position - CenterOfMass;
        
            ComputePartialTensor(PartialInertia, P, M);

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

        UE::Geometry::FMatrix3d InertiaShifter;
        ComputePartialTensor(InertiaShifter, Point, Guest.Mass);
        
        Mass = Mass + Guest.Mass;
        Tensor += Guest.Tensor + InertiaShifter;
        TensorInv = Tensor.Inverse();
    }

    void StripFor(const FInertiaTensor& Guest, const FTransform& HostFrame, const FTransform& GuestFrame)
    {
        auto Point = GuestFrame.InverseTransformPosition(HostFrame.TransformPosition(CenterOfMass));

        UE::Geometry::FMatrix3d InertiaShifter;
        ComputePartialTensor(InertiaShifter, Point, Guest.Mass);

        Tensor = Tensor - (Guest.Tensor + InertiaShifter);

        Point = HostFrame.InverseTransformPosition(GuestFrame.TransformPosition(Guest.CenterOfMass));
        auto Cp = (Mass * CenterOfMass - Guest.Mass * Point) / (Mass - Guest.Mass);

        ensureMsgf(Mass > 0, TEXT("Invalid inertia tensor state: negative mass!"));
        Mass = Mass - Guest.Mass;

        UpdateCenterOfMass(Cp);
        TensorInv = Tensor.Inverse();
    }

    FVector3d ComputeAngularMomentum(const FVector3d& AngularVelocity) const
    {
        return Tensor * AngularVelocity;
    }

    FVector3d ComputeAngularVelocity(const FVector3d& AngularMomentum) const
    {
        return TensorInv * AngularMomentum;
    }

    FVector3d ComputeTorque(const FVector3d& Force, const FVector3d& Position) const
    {
        return (Position - CenterOfMass) ^ Force;
    }

    FVector3d InverseScale(const FVector3d& Vec, const FTransform& RefFrame) const
    {
        return RefFrame.TransformVectorNoScale(TensorInv * RefFrame.InverseTransformVectorNoScale(Vec));
    }

    void ChangeMassDistribution(const FVector3d& Position, double DeltaMass)
    {
        // Step 1: Remove the contribution of this point mass from tensor
        const FVector3d P = Position - CenterOfMass;
        
        UE::Geometry::FMatrix3d InertiaShifter;
        ComputePartialTensor(InertiaShifter, P, DeltaMass);

        Tensor += InertiaShifter;

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
