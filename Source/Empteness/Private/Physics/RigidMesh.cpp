// Fill out your copyright notice in the Description page of Project Settings.


#include "Physics/RigidMesh.h"


// Sets default values
ARigidMesh::ARigidMesh()
{
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
void ARigidMesh::BeginPlay()
{
    Super::BeginPlay();
    
}

// Called every frame
void ARigidMesh::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

