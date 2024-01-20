#pragma once

#include "DataFlow.h"
#include "MatrixTypes.h"

enum DataPacketType
{
    AnyType = -1,
    IntegerType = 1,
    RealNumberType = 1 << 2,
    Vector3Type = 1 << 3,
    Matrix33Type = 1 << 4,
    StringType = 1 << 5,
    ArrayType = 1 << 6
};

using FIntegerPacket = FDataPacket<long, IntegerType>;
using FRealNumPacket = FDataPacket<double, RealNumberType>;

using FVector3dPacket = FDataPacket<FVector3d, Vector3Type>;
using FMatrix3dPacket = FDataPacket<UE::Geometry::FMatrix3d, Matrix33Type>;
using FStringPacket = FDataPacket<FString, StringType>;

template <class T>
class FArrayPacket<T> : FDataPacket<TArray<T>, ArrayType> { };