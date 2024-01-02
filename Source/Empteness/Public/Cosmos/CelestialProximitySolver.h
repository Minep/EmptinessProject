// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Orbital/CelestialBody.h"
#include "GameFramework/Actor.h"
#include "Spatial/SparseDynamicOctree3.h"
#include "CelestialProximitySolver.generated.h"

UCLASS()
class EMPTENESS_API UCelestialProximitySolver : public UObject
{
	GENERATED_BODY()

protected:
	UE::Geometry::FSparseDynamicOctree3* RootSOIs;
	TMap<int, ICelestialBody*> celestials;

public:	
	// Sets default values for this actor's properties
	UCelestialProximitySolver();

	virtual ICelestialBody* GetCapture(ICelestialBody* const &self);
	virtual void AddCelestialBody(ICelestialBody* const &body);
	virtual void UpdateSpatialInfo(ICelestialBody* const &body);
};
