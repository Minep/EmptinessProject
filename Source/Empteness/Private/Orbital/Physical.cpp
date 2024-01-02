#include "Orbital/Physical.h"

FMatrix44d Physical::U2E = FMatrix44d(
    FPlane(-1,0,0,0),
    FPlane( 0,1,0,0),
    FPlane( 0,0,1,0),
    FPlane( 0,0,0,1)
);  // symmetric

FVector Physical::ToEquatorial(const FVector& vector) {
    return FVector(U2E.TransformPosition(vector));
}

FVector Physical::FromEquatorial(const FVector& vector) {
    return FVector(U2E.TransformPosition(vector));
}