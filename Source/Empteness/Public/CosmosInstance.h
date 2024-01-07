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
    CelestialProximitySolver* ProximitySolver = nullptr;

public:
    enum CosmosSubsystem
    {
        PROXIMITY_SOLVER
    };
public:
    UCosmosInstance();
    ~UCosmosInstance();

    virtual void OnStart() override;



    template <class T>
    T* GetCosmosSubsystem(CosmosSubsystem subsys)
    {
        switch (subsys) {
            case PROXIMITY_SOLVER:
                {
                    return ProximitySolver;
                }
        }
        check(false);
    }
};
