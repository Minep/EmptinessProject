// Fill out your copyright notice in the Description page of Project Settings.


#include "Physics/RigidBody.h"
#include "DrawDebugHelpers.h"
#include "Flags.h"


// Sets default values for this component's properties
ARigidBody::ARigidBody()
{
    // Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
    // off to improve performance if you don't need them.
    
    // ...
}

void ARigidBody::UpdateSystemMechanicProperty()
{
    TArray<AActor*> AllChildren;

    GetAttachedActors(AllChildren, true,false);
    for (const auto& Actor : AllChildren) {
        ARigidBody* RB = Cast<ARigidBody>(Actor);
        if (!RB) {
            continue;
        }

        RB->UpdateSystemMechanicProperty();
        Inertia.MergeWith(RB->Inertia, GetActorTransform(), RB->GetActorTransform());
    }
}

void ARigidBody::PreInitializeComponents()
{
    Super::PreInitializeComponents();
    
    CalculateLocalMechanicProperty();

    if (!AnchorBody) {
        SetAnchorBody(this);
    }
    else {
        bKinematicObject = true;
    }
}


// Called when the game starts
void ARigidBody::BeginPlay()
{
    DampRooted = sqrt(1 - DampingFactor);
    Super::BeginPlay();
    
    if (!bKinematicObject) {
        UpdateSystemMechanicProperty();
        const auto CenterMassWorldOffset = GetTransform().TransformVectorNoScale(Inertia.GetCenterOfMass());
        GlobalSpatial.Position += Physical::ToAstroScale(CenterMassWorldOffset, Physical::Length);
    }
    
    // FIXME for debug only
    if (!bKinematicObject) {
        // AddLocalImpulse(10 * FVector3d(0, 1, 0), 100 * FVector3d(0, 0, 1));
        // AddLocalImpulse(10 * FVector3d(0, -1, 0), 100 * FVector3d(0, 0, -1));
        AssignLocalImpulse(10 * FVector3d(1, 0, 0), 100 * FVector3d(0, 1, 0));
        AssignLocalImpulse(10 * FVector3d(-1, 0, 0), 100 * FVector3d(0, -1, 0));
    }

}

void ARigidBody::AssignLocalImpulse(const FVector3d& Impulse, const FVector3d& Position, const EAssignMode AssignMode)
{
    if (IsRigidAttachment()) {
        const auto AnchorLocalPos = AnchorBody->TransformPositionFrom(this, Position);
        const auto AnchorLocalImp = AnchorBody->TransformVectorFrom(this, Impulse);
        AnchorBody->AssignLocalImpulse(AnchorLocalImp, AnchorLocalPos);
        return;
    }

    
#if SHOULD_DEBUG_RIGID
    const auto Center = FVector3d::Zero();
    const auto T = GetTransform();
    DrawDebugLine(GetWorld(),
            T.TransformPositionNoScale(Position), T.TransformPositionNoScale(Position + Impulse),
                    FColor::Magenta, false, -1, 10);
    DrawDebugLine(GetWorld(),
            T.TransformPositionNoScale(Inertia.GetCenterOfMass()), T.TransformPositionNoScale(Position),
            FColor::Turquoise, false, -1, 10);
#endif

    
    const FVector3d Torque = Inertia.ComputeTorque(Impulse, Position);
    const FVector3d DeltaAngVel = Inertia.ComputeAngularVelocity(Torque);

    if (AssignMode == Add) {
        AngularMomentum += Torque;
        AngularVelocity += DeltaAngVel;
    }
    else {
        AngularMomentum = Torque;
        AngularVelocity = DeltaAngVel;
    }
    
    const auto CenterVelChange =
        Physical::ToAstroScale(Impulse / Inertia.GetMass(), Physical::Length);
    if (CenterVelChange.Size() > 0) {
        GlobalSpatial.Velocity += GetTransform().TransformVectorNoScale(CenterVelChange);
    }
}

void ARigidBody::UpdatePhysicsObjectTransform(double DeltaTime, FTransform& Transform)
{
    Super::UpdatePhysicsObjectTransform(DeltaTime, Transform);

    FVector3d Euler = AngularVelocity * 180.0 / DOUBLE_PI * DeltaTime;
    // UE's interpretation on pitch clockwise-ness does not agree with physics's definition
    Euler.Y = -Euler.Y;    
    Euler.X = -Euler.X;    
    Transform.ConcatenateRotation(FQuat::MakeFromEuler(Euler));

    const auto MassCenter = Transform.TransformVectorNoScale(Inertia.GetCenterOfMass());
    Transform.AddToTranslation(-MassCenter);
    
    AngularMomentum *= DampRooted;
    AngularVelocity *= DampRooted;
    if (AngularMomentum.Size() < 1e-6) {
        AngularMomentum = FVector3d::Zero();
    }
}

void ARigidBody::AsyncPhysicsTickActor(float DeltaTime, float SimTime) {

    if (bDetach) {
        bDetach = false;
        DetachSelfFromParent();
    }

    if (bKinematicObject) {
        return;
    }

#if SHOULD_DEBUG_RIGID_MASS
    DebugDrawLocalMassDistribution(10, 2);
#endif
#if SHOULD_DEBUG_MOTION_TRACE
    DebugDrawMotionInfo(1);
#endif

    FTransform T = GetActorTransform();
    
    UpdatePhysicsObjectTransform(DeltaTime, T);
    
    if (!PreCheckCollisions(T, CollisionHits)) {
        SetActorTransform(T);
        return;
    }
    
    FVector3d Loc(0,0,0);
    double Time = 0;
    for (auto& HitResult : CollisionHits) {
        HandleCollisionHit(HitResult);
        Time += HitResult.Time;
        Loc += HitResult.Location;
    }

    Time = Time / CollisionHits.Num();
    if (Time > 1e-4) {
        const auto SrcT = GetTransform();
        const auto RelQ = FMath::Lerp(SrcT.GetRotation(), T.GetRotation(), Time);
        const auto RelL = FMath::Lerp(SrcT.GetLocation(), T.GetLocation(), Time);
        T.SetTranslation(RelL);
        T.SetRotation(RelQ);
        SetActorTransform(T);
    }
    else {
        //SetActorRelativeLocation(Loc / CollisionHits.Num());
    }

    SetActorTransform(T);
    CollisionHits.Reset();
}

bool ARigidBody::PreCheckCollisions(const FTransform& TargetTransform, TArray<FHitResult>& Hits)
{
    const auto Component = Cast<UPrimitiveComponent>(RootComponent);

    if (!Component) {
        return false;
    }

    const auto StartPos = GetActorLocation();
    const auto EndPos = TargetTransform.GetLocation();

    return GetWorld()->ComponentSweepMulti(Hits, Component, StartPos, EndPos, TargetTransform.GetRotation(),
                                        FComponentQueryParams::DefaultComponentQueryParams);
}


void ARigidBody::HandleCollisionHit(const FHitResult& Hit)
{
    if (!(Hit.bBlockingHit || Hit.bStartPenetrating)) {
        return;
    }

    const ARigidBody* Target = Cast<ARigidBody>(Hit.Component->GetOwner());
    if (!Target || Target == this || Target->AnchorBody == this) {
        return;
    }

    const auto HitNormal = Hit.ImpactNormal;
    const auto HitPoint = Hit.ImpactPoint;

#if SHOULD_DEBUG_RIGID_COLLISION
    DrawDebugPoint(GetWorld(), HitPoint, 10, FColor::Purple, false, -1, 10);
    DrawDebugLine(GetWorld(),
            HitPoint, HitPoint + HitNormal * 10,
                    FColor::Yellow, false, -1, 10);
#endif
    
    const FTransform& T2 = AnchorBody->GetTransform();
    const FTransform& T1 = Target->AnchorBody->GetTransform();

    const FInertiaTensor& I2 = AnchorBody->Inertia;
    const FInertiaTensor& I1 = Target->AnchorBody->Inertia;
    
    const auto R2 = AnchorBody->GetCenterRelative(HitPoint);
    const auto R1 = Target->AnchorBody->GetCenterRelative(HitPoint);
    
    const auto V2 = AnchorBody->GetHitPointTangentialVelocity(HitPoint);
    const auto V1 = Target->AnchorBody->GetHitPointTangentialVelocity(HitPoint);
    
    auto J = ((I1.InverseScale(R1 ^ HitNormal, T1) ^ R1)
                     + (I2.InverseScale(R2 ^ HitNormal, T2) ^ R2)) | HitNormal;
    J += 1 / I1.GetMass() + 1 / I2.GetMass();
    J = -(1 + RestCoef) * ((V2 - V1) | HitNormal) / J;

    AnchorBody->AssignGlobalImpulse(J * HitNormal, HitPoint, Add);
    Target->AnchorBody->AssignGlobalImpulse(-J * HitNormal, HitPoint, Add);
}

FVector3d ARigidBody::GetHitPointTangentialVelocity(const FVector3d& HitPoint) const
{
    const auto HitPointRelVec = GetCenterRelative(HitPoint);
    const FVector3d MassCenterVel = Physical::ToMetricScale(GlobalSpatial.Velocity, Physical::Length);
    auto HitPointVel = GetTransform().TransformVectorNoScale(AngularVelocity) ^ HitPointRelVec;
    HitPointVel = HitPointVel + MassCenterVel;

#if SHOULD_DEBUG_RIGID_COLLISION
    DrawDebugLine(GetWorld(),
            HitPoint, HitPoint + HitPointVel,
                    FColor::Cyan, false, -1, 10);
#endif

    return HitPointVel;
}


void ARigidBody::AttachBody(ARigidBody* GuestObject)
{
    if (!GuestObject) {
        return;
    }

    if (GuestObject->AnchorBody != GuestObject) {
        return;
    }
    
    GuestObject->SetAnchorBody(this);
    GuestObject->AttachToActor(this, FAttachmentTransformRules::KeepWorldTransform);
    GuestObject->bKinematicObject = true;

    Inertia.MergeWith(GuestObject->Inertia, GetTransform(), GuestObject->GetTransform());

    if (IsRigidAttachment()) {
        AnchorBody->Inertia.MergeWith(
            GuestObject->Inertia,
            AnchorBody->GetTransform(),
            GuestObject->GetTransform());
    }

    AnchorBody->AngularMomentum += AnchorBody->TransformVectorFrom(GuestObject, GuestObject->AngularMomentum);
    AnchorBody->AngularVelocity = AnchorBody->Inertia.ComputeAngularVelocity(AnchorBody->AngularMomentum);

    AnchorBody->UpdateSpatialState();
}

void ARigidBody::UpdateSpatialState()
{
    auto AnchorT = AnchorBody->GetTransform();
    auto NewCenterMass = GetTransform().TransformPositionNoScale(Inertia.GetCenterOfMass());
    NewCenterMass = Physical::ToAstroScale(NewCenterMass, Physical::Length);
    
    auto R = NewCenterMass - GlobalSpatial.Position;
    GlobalSpatial.Velocity += AnchorT.TransformVectorNoScale(AnchorBody->AngularVelocity) ^ R;

    GlobalSpatial.Position = NewCenterMass;
}


void ARigidBody::DetachSelfFromParent()
{
    if (AnchorBody == this) {
        return;
    }

    const auto Parent = Cast<ARigidBody>(GetAttachParentActor());
    if (!Parent) {
        return;
    }
    
    DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
    Parent->Inertia.StripFor(Inertia, Parent->GetTransform(), GetTransform());

    if (Parent != AnchorBody) {
        AnchorBody->Inertia.StripFor(
            Inertia,
            AnchorBody->GetTransform(),
            GetTransform());
    }

    AngularVelocity = TransformVectorFrom(AnchorBody, AnchorBody->AngularVelocity);
    AngularMomentum = Inertia.ComputeAngularMomentum(AngularVelocity);
    
    AnchorBody->AngularMomentum -= AnchorBody->TransformVectorFrom(this, AngularMomentum);
    
    GlobalSpatial = AnchorBody->GlobalSpatial;

    UpdateSpatialState();
    AnchorBody->UpdateSpatialState();

    SetAnchorBody(this);
    bKinematicObject = false;
}


void ARigidBody::NotifyMassDistributionChange(const FVector3d& LocalPosition, const double DeltaMass)
{
    Inertia.ChangeMassDistribution(LocalPosition, DeltaMass);

    
    // Update the angular velocity to account changed inertia.
    AngularVelocity = Inertia.ComputeAngularVelocity(AngularMomentum);

    if (IsRigidAttachment()) {
        const auto Pos = AnchorBody->TransformPositionFrom(this, LocalPosition);
        AnchorBody->NotifyMassDistributionChange(Pos, DeltaMass);
    }

    // Update the spatial state w.r.t. to shifted center of mass
    AnchorBody->UpdateSpatialState();
}

