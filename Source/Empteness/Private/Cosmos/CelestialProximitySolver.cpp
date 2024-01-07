// Fill out your copyright notice in the Description page of Project Settings.


#include "Cosmos/CelestialProximitySolver.h"

#include "MyUtils.h"
#include "GenericPlatform/GenericPlatformMath.h"

// Sets default values
CelestialProximitySolver::CelestialProximitySolver()
{
    RootSOIs = new UE::Geometry::FSparseDynamicOctree3();
}

CelestialProximitySolver::~CelestialProximitySolver()
{
    if (RootSOIs) {
        delete RootSOIs;
    }
}


ICelestialBody* CelestialProximitySolver::GetCapture(ICelestialBody* const &self)
{
    TArray<int> possible_captures;
    const FVector3d Pos = self->AbsoluteSpatialState().Position;
    const UE::Geometry::FAxisAlignedBox3d UnitBound = UE::Geometry::FAxisAlignedBox3d(Pos, 0.5);

    RootSOIs->RangeQuery(UnitBound, possible_captures);

    double MinDist = 1e30;
    ICelestialBody* capture = nullptr;
    for(int& c_id : possible_captures) {
        auto obj = Celestials.Find(c_id);

        if (!obj || *obj == self) {
            continue;
        }

        const auto Body = *obj;
        double R = (Body->AbsoluteSpatialState().Position - Pos).SizeSquared();
        MinDist = std::min(MinDist, R);
        if (MinDist == R && Body->GetMu() > self->GetMu()) {
            capture = Body;
        }
    }

    return capture;
}

void CelestialProximitySolver::AddCelestialBody(ICelestialBody* const &body)
{
    int id = body->GetCelestialID();
    EM_LOG(Log, "Add Body: %d", id);

    if (Celestials.Contains(id)) {
        return;
    }
    
    Celestials.Emplace(id, body);

    FVector3d pos = body->AbsoluteSpatialState().Position;
    const UE::Geometry::FAxisAlignedBox3d Soi_AABB = UE::Geometry::FAxisAlignedBox3d(pos, body->GetSOIRadius());

    RootSOIs->InsertObject(id, Soi_AABB);
}

void CelestialProximitySolver::UpdateSpatialInfo(ICelestialBody* const &body)
{
    int id = body->GetCelestialID();
    FVector3d pos = body->AbsoluteSpatialState().Position;
    const UE::Geometry::FAxisAlignedBox3d Soi_AABB = UE::Geometry::FAxisAlignedBox3d(pos, body->GetSOIRadius());

    RootSOIs->ReinsertObject(id, Soi_AABB);
}
