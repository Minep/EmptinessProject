// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Physical.h"
#include "Math/MathFwd.h"
/**
 * 
 */
class IInertialFrame
{
    public:
    /**
     * Convert an absolute frame position into current frame
     * @param global_pos position within absolute frame
     * @return position within current frame
     */
    virtual FVector3d FrameLocalPosition(const FVector3d global_pos);

    virtual FSpatialState ToAbsoluteState(const FSpatialState& sstate);
    
    virtual FSpatialState ToFrameLocalState(const FSpatialState& sstate);

    virtual double GetObjectMass();
};
