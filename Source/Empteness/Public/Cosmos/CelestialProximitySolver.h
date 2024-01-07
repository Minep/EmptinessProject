// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Physics/Orbital/CelestialBody.h"
#include "GameFramework/Actor.h"
#include "Spatial/SparseDynamicOctree3.h"

class EMPTENESS_API CelestialProximitySolver
{

protected:
    UE::Geometry::FSparseDynamicOctree3* RootSOIs;

    UPROPERTY()
    TMap<int, ICelestialBody*> Celestials;

public:    
    // Sets default values for this actor's properties
    CelestialProximitySolver();
    virtual ~CelestialProximitySolver();

    virtual ICelestialBody* GetCapture(ICelestialBody* const &self);
    virtual void AddCelestialBody(ICelestialBody* const &body);
    virtual void UpdateSpatialInfo(ICelestialBody* const &body);
};
