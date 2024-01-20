// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"

enum class ECraftAbilityType
{
    AttitudeStablizer
};

/**
 * 
 */
template <class T> class EMPTENESS_API ICraftAbility
{
    static_assert(std::is_base_of_v<AActor, T>, "ICraftAbility must act on AActor");

protected:
    ECraftAbilityType Ability;
    
protected:
    ICraftAbility(ECraftAbilityType Ability): Ability(Ability) { }
    
    ~ICraftAbility() = default;
    
public:
    virtual void InitializeAbility(T Host) = 0;
    
    virtual void InvokeAbility(T Host) = 0;

    virtual void InvokeAbilityAnon() = 0;
    
    virtual FString GetName() = 0;
};
