// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "InertiaTensor.h"
#include "Components/ActorComponent.h"
#include "Orbital/Celestia.h"
#include "RigidBody.generated.h"

UCLASS()
class EMPTENESS_API ARigidBody : public ACelestia
{
    GENERATED_BODY()

public:
    // Sets default values for this component's properties
    ARigidBody();

    enum EAssignMode
    {
        Set,
        Add
    };

protected:
    void UpdateSystemMechanicProperty();

protected:
    // Called when the game starts
    virtual void BeginPlay() override;
    
    // UPROPERTY(VisibleAnywhere, DisplayName="System Mass", meta=(Units="kg"))
    // double Mass = 0;
    //
    UPROPERTY(EditAnywhere, DisplayName="Local Mass", meta=(Units="kg"))
    double LocalMass = 0;

    double DampRooted = 0;

    FInertiaTensor BodyInertia;

    UPROPERTY(VisibleAnywhere, DisplayName="Angular Velocity")
    FVector3d AngularVelocity;
    
    UPROPERTY(VisibleAnywhere, DisplayName="Angular Momentum")
    FVector3d AngularMomentum;
    
    TObjectPtr<ARigidBody> AnchorBody = nullptr;

    TArray<FHitResult> CollisionHits;

    virtual void PreInitializeComponents() override;

public:
    UPROPERTY(EditAnywhere, DisplayName="Non-uniform Mass Distribution")
    bool bNonUniformMassDistribution;

    UPROPERTY(EditAnywhere, DisplayName="Detach From Parent")
    bool bDetach = false;

    UPROPERTY(EditAnywhere, DisplayName="Mass Distrubtion")
    TArray<FMassDistribution> MassDistribution;

    UPROPERTY(EditAnywhere, DisplayName="Angular Damping Coefficient", meta=(ClampMax=1, ClampMin=0))
    // Damping coefficient for angular momentum
    double DampingFactor;

    UPROPERTY(EditAnywhere, DisplayName="Restitution Coefficient", meta=(ClampMax=1, ClampMin=0))
    // Damping coefficient for angular momentum
    double RestCoef = 0;

public:
    /**
     * Add an impulse to the body which is local to the body's local frame.
     * The torque and linear velocity change of center of mass will be calculated
     * @param Impulse Impulse
     * @param Position Position local to rotating frame where the impulse is applied
     */
    void AssignLocalImpulse(const FVector3d& Impulse, const FVector3d& Position, EAssignMode AssignMode = Add);

    void AssignGlobalImpulse(const FVector3d& Impulse, const FVector3d& Position, EAssignMode mode = Add)
    {
        const auto T = GetTransform();
        const auto LocalImpulse = T.InverseTransformVectorNoScale(Impulse);
        const auto LocalPosition = T.InverseTransformPositionNoScale(Position);
        AssignLocalImpulse(LocalImpulse, LocalPosition, mode);
    }


    /*
     * Physics Simulation Logic
     */
    
    virtual void AsyncPhysicsTickActor(float DeltaTime, float SimTime) override;
    virtual void UpdatePhysicsObjectTransform(double DeltaTime, FTransform& Transform) override;

    
    /*
     * Rigid Body Welding and Unwelding
     */
    
    void AttachBody(ARigidBody* GuestObject);
    void DetachSelfFromParent();
    
    void SetAnchorBody(ARigidBody* Anchor)
    {
        TArray<AActor*> AllChildren;

        // Get all desendent, to avoid stack cost of TArray
        GetAttachedActors(AllChildren, true, true);
        
        for (auto& actor : AllChildren) {
            ARigidBody* rb = Cast<ARigidBody>(actor);
            if (!rb) {
                continue;
            }
            
            rb->AnchorBody = Anchor;
        }

        AnchorBody = Anchor;
    }

    bool IsRigidAttachment() const
    {
        return AnchorBody != this;
    }

    
    /*
     * Rigid Body Collision Response
     */
    
    void HandleCollisionHit(const FHitResult& Hit);
    FVector3d GetHitPointTangentVelocity(const FVector3d& HitPoint) const;
    virtual bool PreCheckCollisions(const FTransform TargetTransform, TArray<FHitResult>& Hits);
    
    /*
     * Mechanic Property Calculation
     */
    
    void RecalculateMechanicProperty()
    {
        CalculateLocalMechanicProperty();
        UpdateSystemMechanicProperty();
    }
    
    void CalculateLocalMechanicProperty()
    {
        if (!bNonUniformMassDistribution) {
            FVector3d Dim = GetTransform().GetScale3D();
            const auto Mesh = Cast<UMeshComponent>(GetComponentByClass(UStaticMeshComponent::StaticClass()));
            if (!Mesh) {
                Dim *= 100;
            } else {
                Dim *= Mesh->Bounds.BoxExtent;
            }

            LocalMass = std::max(LocalMass, 1.0);
            // Assume the uniformity following rectangular parallelepiped
            BodyInertia = FInertiaTensor(Dim, LocalMass, FInertiaTensor::RectParallelepiped);

            UE_LOG(LogTemp, Warning, TEXT("%f, %f, %f"), Dim.X, Dim.Y, Dim.Z);

        } else {
            BodyInertia = FInertiaTensor(MassDistribution);
            LocalMass = BodyInertia.GetMass();
        }
    }

    void NotifyMassDistributionChange(const FVector3d& LocalPosition, const double DeltaMass);


    /*
     * General Spatial Transformation Helper Functions
     */

    FVector3d TransformVectorFrom(const ARigidBody* Src, const FVector3d& Vec) const
    {
        if (this == Src) {
            return Vec;
        }

        
        auto t1 = Src->GetActorTransform().TransformVectorNoScale(Vec);
        return GetActorTransform().InverseTransformVectorNoScale(t1);
    }

    FVector3d TransformPositionFrom(const ARigidBody* Src, const FVector3d& Pos) const
    {
        if (this == Src) {
            return Pos;
        }
        
        auto t1 = Src->GetActorTransform().TransformPositionNoScale(Pos);
        return GetActorTransform().InverseTransformPositionNoScale(t1);
    }

    FVector3d GetCenterRelative(const FVector3d& GlobalPos) const
    {
        const auto MassCenterPos = GetTransform().TransformPositionNoScale(BodyInertia.GetCenterOfMass());
        return GlobalPos - MassCenterPos;
    }
    
    void UpdateSpatialState();

    
    /*
     * Debug Drawing Helper Functions
     */

    void DebugDrawLocalMassDistribution(float MinSize, float MassMultiplier) const
    {
        const auto T = GetTransform();
    
        for (auto& Md: MassDistribution) {
            DrawDebugLine(GetWorld(),
                T.TransformPositionNoScale(BodyInertia.GetCenterOfMass()), T.TransformPositionNoScale(Md.Position),
                          FColor::Magenta, false, -1, 1);

            DrawDebugPoint(GetWorld(), T.TransformPositionNoScale(Md.Position), MinSize + MassMultiplier * Md.Mass,
                          FColor::Red, false, -1, 1);
        }
        DrawDebugPoint(GetWorld(), T.TransformPositionNoScale(BodyInertia.GetCenterOfMass()), 50, FColor::Green,false, -1, 10);
        DrawDebugPoint(GetWorld(), T.TransformPositionNoScale(BodyInertia.GetCenterOfMass()), 5, FColor::Green,true, -1, 10);
        DrawDebugPoint(GetWorld(), GetActorLocation(), 30, FColor::Blue,false, -1, 10);
    }

    void DebugDrawMotionInfo(float Multiplier = 1)
    {
        const auto T = GetTransform();
        const auto origin = T.TransformPositionNoScale(BodyInertia.GetCenterOfMass());
        DrawDebugLine(GetWorld(), origin, T.TransformPositionNoScale(Multiplier * AngularMomentum) + origin,
                     FColor(0xfcba03), false, -1, 2);
        DrawDebugLine(GetWorld(), origin, T.TransformPositionNoScale(Multiplier * AngularVelocity) + origin,
                     FColor(0x32a852), false, -1, 3);
    }
};
