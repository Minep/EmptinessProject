// Fill out your copyright notice in the Description page of Project Settings.


#include "CosmosInstance.h"

UCosmosInstance::UCosmosInstance()
{
    
}

UCosmosInstance::~UCosmosInstance()
{
    if (ProximitySolver) {
        delete ProximitySolver;
    }
}

void UCosmosInstance::OnStart()
{
    ProximitySolver = new CelestialProximitySolver();
}

