#pragma once

#include "DataFlow.h"
#include "DataPacketTypes.h"
#include "SystemEmulator.h"

class DataProcessorNode
{
protected:
    TQueue<FDataPacket<>*> PacketRxPending;
public:
    virtual ~DataProcessorNode() = default;

    virtual void AddIncomingPacket(FDataPacket<>* Packet)
    {
        Packet->Take();
        PacketRxPending.Enqueue(Packet);
    }
    
    virtual FDataPacket<>* GetCurrentPending()
    {
        const auto Pkt = PacketRxPending.Peek();
        return Pkt ? *Pkt : nullptr;
    }

    void NextPacket()
    {
        FDataPacket<>* Pkt;
        if (PacketRxPending.Dequeue(Pkt)) {
            Pkt->Put();
        }
    }
};

class FWiringSocket
{
    bool OutputSocket;
    int SocketId;
    FString Name;
    
    DataProcessorNode& Owner;
    FWiringSocket* ConnectedPeer = nullptr;
    DataPacketType AcceptedTypes;

public:
    explicit FWiringSocket(DataProcessorNode& MyOwner, FString& Name, DataPacketType AcceptedTypes, int SocketId, bool OutputSocket)
        : OutputSocket(OutputSocket), Name(Name), Owner(MyOwner), AcceptedTypes(AcceptedTypes), SocketId(SocketId)
    { }

    static FWiringSocket In(DataProcessorNode& MyOwner, FString& Name, DataPacketType AcceptedTypes, int SocketId)
    {
        return FWiringSocket(MyOwner, Name, AcceptedTypes, SocketId, false);
    }

    static FWiringSocket Out(DataProcessorNode& MyOwner, FString& Name, DataPacketType AcceptedTypes, int SocketId)
    {
        return FWiringSocket(MyOwner, Name, AcceptedTypes, SocketId, true);
    }

    bool CanConnect(const FWiringSocket* Socket) const
    {
        if (!Socket) {
            return false;
        }

        if (Socket->AcceptedTypes != AcceptedTypes) {
            return false;
        }

        if (std::addressof(Socket->Owner) == std::addressof(Owner)) {
            return false;
        }

        return Socket->OutputSocket ^ OutputSocket;
    }

    bool ConnectTo(FWiringSocket* Socket)
    {
        if (!CanConnect(Socket)) {
            return false;
        }

        ConnectedPeer = Socket;
        return true;
    }

    bool SendPacket(FDataPacket<>* Packet) const
    {
        if (!ensureMsgf(Packet->MatchType(AcceptedTypes),
                TEXT("Incompatible type 0x%x, expected 0x%x (%s)"),
                Packet->GetEnclaveType(), AcceptedTypes, *Name)) {
            return false;
        }
        
        if (!ConnectedPeer || !OutputSocket) {
            return false;
        }

        return ConnectedPeer->ReceivePacket(Packet);
    }

    bool ReceivePacket(FDataPacket<>* Packet) const
    {
        Packet->SetInboundGatewayId(SocketId);
        Owner.AddIncomingPacket(Packet);
        return true;
    }

    bool IsOutputSocket() const
    {
        return OutputSocket;
    }
    
};

class CircuitElementBase: DataProcessorNode
{
protected:
    TMap<int, FWiringSocket> OutputSocket;
    TMap<int, FWiringSocket> InputSocket;

    FString Name;
    CircuitSystemEmulator& Emulator;

    void AddOutputSocket(const int SocketId, const char* Name, DataPacketType AcceptedTypes)
    {
        auto NameFStr = FString(Name);
        OutputSocket.Add(SocketId, FWiringSocket::Out(*this, NameFStr, AcceptedTypes, SocketId));
    }

    void AddInputSocket(const int SocketId, const char* Name, DataPacketType AcceptedTypes)
    {
        auto NameFStr = FString(Name);
        InputSocket.Add(SocketId, FWiringSocket::In(*this, NameFStr, AcceptedTypes, SocketId));
    }

    template <class T>
    bool WriteToSocket(const int SocketId, T* PacketLike)
    {
        auto Socket = OutputSocket.Find(SocketId);

        if (!Socket) {
            return false;
        }

        return Socket->SendPacket(dynamic_cast<FDataPacket<>*>(PacketLike));
    }

    virtual void ProcessPacket()
    {
        FDataPacket<>* Pkt;
        while((Pkt = GetCurrentPending())) {
            HandleInboundPacket(Pkt);
            NextPacket();
        }
    }

public:
    virtual void OnBootUp() { }

    virtual void OnShutdown() { }
    
    virtual void OnClockTick(bool RaisingEdge) { }

    virtual void HandleInboundPacket(FDataPacket<>* Packet) { }

public:
    explicit CircuitElementBase(const FString& Name, CircuitSystemEmulator& Emulator)
        : Emulator(Emulator), Name(Name)
    { }
    
    void SystemClockTick(bool IsRaisingEdge)
    {
        ProcessPacket();
        OnClockTick(IsRaisingEdge);
    }
};
