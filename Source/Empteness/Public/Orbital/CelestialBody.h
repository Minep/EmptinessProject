// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Physical.h"
#include "UObject/Interface.h"
#include "CelestialBody.generated.h"

// This class does not need to be modified.
UINTERFACE()
class UCelestialBody : public UInterface
{
	GENERATED_BODY()
};

/**
 * 
 */
class EMPTENESS_API ICelestialBody
{
	GENERATED_BODY()

	// Add interface functions to this class. This is the class that will be inherited to implement this interface.
public:
	virtual int GetCelestialID() const;
	virtual double GetMu() const;
	virtual double GetSOIRadius() const;
	virtual const FSpatialState& AbsoluteSpatialState() const;
};
