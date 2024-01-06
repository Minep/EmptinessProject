// Fill out your copyright notice in the Description page of Project Settings.


#include "Cosmos/CelestialProximitySolver.h"
#include "GenericPlatform/GenericPlatformMath.h"

// Sets default values
UCelestialProximitySolver::UCelestialProximitySolver()
{
    RootSOIs = new UE::Geometry::FSparseDynamicOctree3();
}

ICelestialBody* UCelestialProximitySolver::GetCapture(ICelestialBody* const &self)
{
    TArray<int> possible_captures;
    FVector3d pos = self->AbsoluteSpatialState().Position;
    UE::Geometry::FAxisAlignedBox3d unitBound = UE::Geometry::FAxisAlignedBox3d(pos, 0.5);

    RootSOIs->RangeQuery(unitBound, possible_captures);

    double maxForce = 0;
    ICelestialBody* capture = nullptr;
    for(int& c_id : possible_captures) {
        ICelestialBody** obj = celestials.Find(c_id);

        if (!obj) {
            continue;
        }

        ICelestialBody* body = *obj;
        double F = body->GetMu();
        maxForce = FGenericPlatformMath::Max(maxForce, F);
        if (maxForce == F) {
            capture = body;
        }
    }

    return capture;
}

void UCelestialProximitySolver::AddCelestialBody(ICelestialBody* const &body)
{
    int id = body->GetCelestialID();

    if (celestials.Contains(id)) {
        return;
    }
    
    celestials.Emplace(id, body);

    FVector3d pos = body->AbsoluteSpatialState().Position;
    UE::Geometry::FAxisAlignedBox3d soi_aabb = UE::Geometry::FAxisAlignedBox3d(pos, body->GetSOIRadius());

    RootSOIs->InsertObject(id, soi_aabb);
}

void UCelestialProximitySolver::UpdateSpatialInfo(ICelestialBody* const &body)
{
    int id = body->GetCelestialID();
    FVector3d pos = body->AbsoluteSpatialState().Position;
    UE::Geometry::FAxisAlignedBox3d soi_aabb = UE::Geometry::FAxisAlignedBox3d(pos, body->GetSOIRadius());

    RootSOIs->ReinsertObject(id, soi_aabb);
}
