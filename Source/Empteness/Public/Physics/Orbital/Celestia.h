// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CosmosInstance.h"
#include "Physics/Orbital/CelestialBody.h"
#include "Math/MathFwd.h"
#include "Physics/Solver/IOrbitalMechanicSolver.h"

#include "Celestia.generated.h"

UCLASS()
class EMPTENESS_API ACelestia : public AActor, public ICelestialBody, public IInertialFrame
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ACelestia();

protected:
	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Orbital State"))
	FOrbitalState orbit;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "Rel.Mass"))
	// Relative mass compare to Earth's Mass
	double RelativeMass;

	UPROPERTY(EditAnywhere, Meta = (DisplayName = "SOI Radius"))
	// Relative mass compare to Earth's Mass
	double soi_radius;

	UPROPERTY(VisibleAnywhere, Meta = (DisplayName = "Global Spatial State"))
	FSpatialState GlobalSpatial;

	FVector3d MainAttraction;

protected:
	UCosmosInstance* CosmosInstance;

	IOrbitalMechanicSolver* Solver;

	double Mu;

	// Inverse transformation of osculating plane local space
	FQuat OspInv;

	ACelestia* Centric = nullptr;

	int id;

	bool bKinematicObject = false;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	virtual void PreInitializeComponents() override;

public:
	enum CelestialStateSource
	{
		ORBITAL_ELEMT = 0,
		SPATIAL_STATE = 1
	};

	virtual void LoadInitialState(CelestialStateSource source);

	virtual void UpdateOrbitalElement();

	virtual void Tick(float DeltaTime) override;

	virtual void AsyncPhysicsTickActor(float DeltaTime, float SimTime) override;

	///
	/// InertialFrame
	///
public:
	virtual FVector3d FrameLocalPosition(FVector3d global_pos) override
	{
		return global_pos - GlobalSpatial.Position;
	}

	virtual double GetObjectMass() override
	{
		return RelativeMass;
	}

	virtual FSpatialState ToAbsoluteState(const FSpatialState& sstate) override
	{
		return sstate + GlobalSpatial;
	}

	virtual FSpatialState ToFrameLocalState(const FSpatialState& sstate) override
	{
		return sstate - GlobalSpatial;
	}

	///
	/// ICelestialBody
	///
public:
	virtual double GetSOIRadius() const override { return soi_radius; }

	virtual int GetCelestialID() const override
	{
		return id;
	}

	virtual double GetMu() const override
	{
		return Mu;
	}

	virtual const FSpatialState& AbsoluteSpatialState() const override
	{
		return GlobalSpatial;
	}

	virtual void UpdatePhysicsObjectTransform(double DeltaTime, FTransform& Transform);
};
