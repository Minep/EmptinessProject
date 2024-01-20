#include "Circuit/CircuitElementBase.h"

class AttitudeStabilizer : CircuitElementBase
{
    double ErrorP = 0, ErrorI = 0, ErrorD = 0;
    double Kp, Ki, Kd;

    FVector3d CurrentOrientation;
    FVector3d TargetOrientation;

protected:
    enum SocketType
    {
        CurrentORNT = 0,
        TargetORNT = 1,
        AttitudeControlVec = 2,
    };

    using CircuitElementBase::Emulator;
    
    void ResetPIDState()
    {
        ErrorP = ErrorI = ErrorD = 0;
    }

    double GetDeltaCorrection(const double CurrentVal, const double DeltaT)
    {
        ErrorD = DeltaT * (CurrentVal - ErrorP);
        ErrorI += DeltaT * ErrorP;
        ErrorP = CurrentVal;

        return Kp * CurrentVal + Ki * ErrorI + Kd * ErrorD;
    }
    
public:
    explicit AttitudeStabilizer(CircuitSystemEmulator& Emulator)
        : CircuitElementBase(FString("CE_SAS"), Emulator)
    {
        AddInputSocket(CurrentORNT, "CE_SAS_SIn_CurORIENT", Vector3Type);
        AddInputSocket(TargetORNT, "CE_SAS_SIn_TgtORIENT", Vector3Type);

        AddOutputSocket(AttitudeControlVec, "CE_SAS_SOut_AttCntl", Vector3Type);
    }

    virtual void OnClockTick(bool RaisingEdge) override;

    virtual void HandleInboundPacket(FDataPacket<>* Packet) override
    {
        FVector3dPacket* VPacket = dynamic_cast<FVector3dPacket*>(Packet);

        const auto Vector = VPacket->GetData();
        const auto Gateway = VPacket->GetInboundGatewayId();
        if (Gateway == CurrentORNT) {
            CurrentOrientation.Set(Vector->X, Vector->Y, Vector->Z);
        }
        else if (Gateway == TargetORNT) {
            TargetOrientation.Set(Vector->X, Vector->Y, Vector->Z);
        }
    }
};