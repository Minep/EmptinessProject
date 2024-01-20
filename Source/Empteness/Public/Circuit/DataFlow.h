#pragma once


template <class EnclaveType=void*, int EnclaveTypeIdentifier=-1>
class FDataPacket
{
protected:
    int RefCount;
    int Command;
    EnclaveType Data;
    int TypeIdentifier = EnclaveTypeIdentifier;
    int InboundGateway;

public:
    virtual ~FDataPacket() = default;
    explicit FDataPacket(): RefCount(1), Command(0) {}
    explicit FDataPacket(int Command, EnclaveType& Data): RefCount(1), Command(Command), Data(Data) {}
    
    static FDataPacket<EnclaveType>* NewPacket(EnclaveType& Data, int Command = 0)
    {
        return new FDataPacket<EnclaveType>(Command, Data);
    }

    virtual void PackData(EnclaveType& ToBePacked)
    {
        Data = ToBePacked;
    }

    virtual const EnclaveType* GetData() const
    {
        return &Data;
    }
    
    void Take() { RefCount++; }
    void Put()
    {
        RefCount--;
        if (RefCount <= 0) {
            delete this;
        }
        // must not access member
    }

    bool MatchType(int TypeId) const
    {
        return (TypeId & TypeIdentifier) == TypeId;
    }

    int GetEnclaveType() const
    {
        return TypeIdentifier;
    }

    void SetInboundGatewayId(int InboundGatewayId)
    {
        InboundGateway = InboundGatewayId;
    }

    int GetInboundGatewayId() const
    {
        return InboundGateway;
    }
};

template <class EnclaveType>
class FRefDataPacket : public FDataPacket<EnclaveType*>
{
protected:
    typedef FDataPacket<EnclaveType*> Super;
    int DataRefCount = 0;

    typedef void (*EnclaveDataRecycler)(void*);
    EnclaveDataRecycler FreeEnclaveData;

public:
    explicit FRefDataPacket() { }

    using FDataPacket<EnclaveType*>::PackData;
    virtual void PackData(EnclaveType*& ToBePacked, EnclaveDataRecycler Recycler = nullptr)
    {
        check(DataRefCount == 0);

        if (!FreeEnclaveData && !this->Data) {
            FreeEnclaveData(this->Data);
        }

        this->Data = ToBePacked;
        FreeEnclaveData = Recycler;
        DataRefCount++;
    }

    virtual const EnclaveType** GetData() const override
    {
        return nullptr;
    }

    EnclaveType** BorrowData()
    {
        DataRefCount++;
        return &this->Data;
    }

    void ReturnData(EnclaveType* ReturnedData)
    {
        check(ReturnedData == this->Data);
        
        DataRefCount--;
    }
};