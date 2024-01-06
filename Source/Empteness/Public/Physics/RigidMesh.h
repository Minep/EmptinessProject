// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Physics/RigidBody.h"
#include "RigidMesh.generated.h"

UCLASS()
class EMPTENESS_API ARigidMesh : public ARigidBody
{
    GENERATED_BODY()

protected:
    UPROPERTY(Category = StaticMeshActor, VisibleAnywhere, BlueprintReadOnly, meta = (ExposeFunctionCategories = "Mesh,Rendering,Physics,Components|StaticMesh", AllowPrivateAccess = "true"))
    TObjectPtr<UStaticMeshComponent> MeshComponent;
    
public:
    // Sets default values for this actor's properties
    ARigidMesh();

protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

public:
    // Called every frame
    virtual void Tick(float DeltaTime) override;
};
