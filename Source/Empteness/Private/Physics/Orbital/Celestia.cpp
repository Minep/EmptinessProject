// Fill out your copyright notice in the Description page of Project Settings.


#include "Physics/Orbital/Celestia.h"
#include "Physics/Physical.h"
#include "MyUtils.h"
#include "Physics/Solver/TwoBodyDynamicsSolver.h"


// Sets default values
ACelestia::ACelestia()
{
     // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = false;
    bAsyncPhysicsTickEnabled = true;

    Solver = new FTwoBodyDynamicsSolver();

    Tags.Push(ACTOR_TAG("Celestia"));

    id = GetUniqueID();
}

// Called when the game starts or when spawned
void ACelestia::BeginPlay()
{
    Super::BeginPlay();
    CosmosInstance = Cast<UCosmosInstance>(GetGameInstance());

    check(CosmosInstance);

    if (bKinematicObject) {
        return;
    }

    TArray<AActor*> actors;
    GetAttachedActors(actors, true);
    
    for (AActor*& actor: actors) {
        if (!actor->IsA<ACelestia>()) {
            continue;
        }

        ACelestia* c = Cast<ACelestia>(actor);
        c->Centric = this;
    }

    Mu = Physical::GetGParam(CelestialMass, 0);

    if (soi_radius <= 0) {
        soi_radius = Physical::GetSOIRadius(CelestialMass, 0.01);
    }

    UE_LOG(LogTemp, Warning, TEXT("%s"), *GetActorLabel());
    LoadInitialState(ORBITAL_ELEMT);

    auto ProximitySolver =
            CosmosInstance->GetCosmosSubsystem<CelestialProximitySolver>(UCosmosInstance::PROXIMITY_SOLVER);
    if (ProximitySolver) {
        ProximitySolver->AddCelestialBody(this);
    }
}

void ACelestia::PreInitializeComponents()
{
    Super::PreInitializeComponents();
}

// Called every frame
void ACelestia::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    
}

void ACelestia::UpdatePhysicsObjectTransform(double DeltaTime, FTransform& Transform)
{
    
    if (!Centric) {
        Solver->SimulationStep(0, DeltaTime, GlobalSpatial);
    } else {
        GlobalSpatial -= Centric->PrevFrameStateDelta;
        
        FSpatialState local = Centric->ToFrameLocalState(GlobalSpatial);

        PrevFrameStateDelta = local;
        
        Solver->SimulationStep(Centric->Mu, DeltaTime, local);

        PrevFrameStateDelta -= local;

        GlobalSpatial = Centric->ToAbsoluteState(local);
    }

    if (GlobalSpatial.Velocity.SizeSquared() != 0) {
        DrawDebugLine(GetWorld(), GlobalSpatial.Position * 1e3, (GlobalSpatial.Position + GlobalSpatial.Velocity) * 1e3, FColor::Blue,
                    false, -1, 10, 30);
    }
    
    Transform.SetTranslation(GlobalSpatial.Position * 1e3);
    
    UpdateOrbitalElement();

    if (CaptureUpdateTimer > 0.5) {
        const auto& ProximitySolver =
            CosmosInstance->GetCosmosSubsystem<CelestialProximitySolver>(UCosmosInstance::PROXIMITY_SOLVER);
    
        if (ProximitySolver) {
            ProximitySolver->UpdateSpatialInfo(this);
            auto Capture = ProximitySolver->GetCapture(this);
    
            if (Capture != this && Capture != Centric) {
                Centric = Cast<ACelestia>(Capture);
                if (Centric) {
                    EM_LOG(Log, "[%.3f] %s Capture: %s", SimTimeLocal, *GetActorLabel(), *Centric->GetActorLabel());
                }
            }
        }
        else {
            EM_LOG(Warning, "Ptr to ProximitySolver is not valid anymore");
        }
        
        CaptureUpdateTimer = 0;
    }
    CaptureUpdateTimer += DeltaTime;
    SimTimeLocal += DeltaTime;
}


void ACelestia::AsyncPhysicsTickActor(float DeltaTime, float SimTime) {
    Super::AsyncPhysicsTickActor(DeltaTime, SimTime);

    if (bKinematicObject) {
        return;
    }

    auto T = GetActorTransform();
    
    UpdatePhysicsObjectTransform(DeltaTime, T);
    
    SetActorTransform(T);

    DrawDebugPoint(GetWorld(), GlobalSpatial.Position * 1e3, 5, FColor::Red,
                    true, -1, 0);
}

void ACelestia::LoadInitialState(CelestialStateSource source)
{
    if (!Centric)
    {
        GlobalSpatial.Position = Physical::ToAstroScale(GetActorLocation(), Physical::Length);
        return;
    }
    
    if (source == ORBITAL_ELEMT)
    {
        FSpatialState local = Solver->SolveLocalSpatialState(Centric, orbit);
        GlobalSpatial = Centric->ToAbsoluteState(local);
    }
}

void ACelestia::UpdateOrbitalElement()
{
    if (!Centric) {
        return;
    }

    FSpatialState local = Centric->ToFrameLocalState(GlobalSpatial);
    Solver->SolveOrbitalElement(Centric->Mu, local, orbit);
}