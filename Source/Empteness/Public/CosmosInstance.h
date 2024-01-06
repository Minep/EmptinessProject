// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Cosmos/CelestialProximitySolver.h"
#include "Engine/GameInstance.h"
#include "CosmosInstance.generated.h"

/**
 * 
 */
UCLASS()
class EMPTENESS_API UCosmosInstance : public UGameInstance
{
    GENERATED_BODY()

public:
    UCelestialProximitySolver* ProximitySolver;
public:
    enum CosmosSubsystem
    {
        PROXIMITY_SOLVER
    };
public:
    UCosmosInstance();

    template <class T>
    T* SubsystemOf(CosmosSubsystem subsys)
    {
        switch (subsys) {
            case PROXIMITY_SOLVER:
                {
                    return ProximitySolver;
                }
        }

        return nullptr;
    }
};
