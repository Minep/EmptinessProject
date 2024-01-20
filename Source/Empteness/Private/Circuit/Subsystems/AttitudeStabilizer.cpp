#include "Circuit/Subsystems/AttitudeStabilizer.h"

void AttitudeStabilizer::OnClockTick(bool RaisingEdge)
{
    const FVector3d Cross = (CurrentOrientation ^ TargetOrientation).GetSafeNormal();
    const double RelAngle = CurrentOrientation | TargetOrientation;
    const double Correction = GetDeltaCorrection(RelAngle, Emulator.GetDesignatedPluseWidth());

    auto ControlVector = Correction * Cross;
    if (ControlVector.IsZero()) {
        return;
    }
    
    WriteToSocket(AttitudeControlVec, FVector3dPacket::NewPacket(ControlVector));
}
