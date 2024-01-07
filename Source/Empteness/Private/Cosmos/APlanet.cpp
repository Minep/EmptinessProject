// Fill out your copyright notice in the Description page of Project Settings.


#include "Cosmos/APlanet.h"


// Sets default values
AAPlanet::AAPlanet()
{
    // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = false;

    MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>("DefaultMesh");
    MeshComponent->SetCollisionProfileName(UCollisionProfile::BlockAll_ProfileName);
    MeshComponent->Mobility = EComponentMobility::Movable;
    MeshComponent->SetGenerateOverlapEvents(false);
    
    MeshComponent->bUseDefaultCollision = true;
    MeshComponent->bTraceComplexOnMove = true;
    MeshComponent->SetAllUseCCD(true);
    
    RootComponent = MeshComponent;
}

// Called when the game starts or when spawned
void AAPlanet::BeginPlay()
{
    Super::BeginPlay();
    
}

// Called every frame
void AAPlanet::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

