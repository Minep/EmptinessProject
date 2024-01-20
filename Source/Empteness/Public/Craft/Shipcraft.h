// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "CraftAbility.h"
#include "Physics/RigidBody.h"
#include "Shipcraft.generated.h"

UCLASS()
class EMPTENESS_API AShipcraft : public ARigidBody
{
    GENERATED_BODY()

protected:
    TWeakObjectPtr<ICraftAbility<AShipcraft>> ShipComputerAbility;

    TArray<ICraftAbility<AShipcraft>*> Abilities; 

public:
    // Sets default values for this actor's properties
    AShipcraft();

protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

public:
    // Called every frame
    virtual void Tick(float DeltaTime) override;
};
