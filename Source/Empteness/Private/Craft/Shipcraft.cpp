// Fill out your copyright notice in the Description page of Project Settings.


#include "Craft/Shipcraft.h"


// Sets default values
AShipcraft::AShipcraft()
{
    // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AShipcraft::BeginPlay()
{
    Super::BeginPlay();
    
}

// Called every frame
void AShipcraft::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

