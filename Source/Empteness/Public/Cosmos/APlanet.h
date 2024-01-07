// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Physics/Orbital/Celestia.h"
#include "APlanet.generated.h"

UCLASS()
class EMPTENESS_API AAPlanet : public ACelestia
{
    GENERATED_BODY()

public:
    // Sets default values for this actor's properties
    AAPlanet();

protected:
    UPROPERTY(Category = StaticMeshActor, VisibleAnywhere, BlueprintReadOnly, meta = (ExposeFunctionCategories = "Mesh,Rendering,Physics,Components|StaticMesh", AllowPrivateAccess = "true"))
    TObjectPtr<UStaticMeshComponent> MeshComponent;
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

public:
    // Called every frame
    virtual void Tick(float DeltaTime) override;
};
