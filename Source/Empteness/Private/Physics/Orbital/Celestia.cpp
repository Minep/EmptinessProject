// Fill out your copyright notice in the Description page of Project Settings.


#include "Physics/Orbital/Celestia.h"
#include "Physics/Physical.h"
#include "MyUtils.h"
#include "Physics/Solver/PatchedConicSolver.h"


// Sets default values
ACelestia::ACelestia()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
	bAsyncPhysicsTickEnabled = true;

	Solver = new PatchedConicSolver();

	Tags.Push(ACTOR_TAG("Celestia"));

	id = GetUniqueID();
}

// Called when the game starts or when spawned
void ACelestia::BeginPlay()
{
	Super::BeginPlay();
	CosmosInstance = Cast<UCosmosInstance>(GetGameInstance());

	ensure(CosmosInstance);

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

	Mu = Physical::GetGParam(RelativeMass, 0);

	if (soi_radius <= 0) {
		soi_radius = Physical::GetSOIRadius(RelativeMass, 0.01);
	}
	
	LoadInitialState(ORBITAL_ELEMT);
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
		FSpatialState local = Centric->ToFrameLocalState(GlobalSpatial);

		Solver->SimulationStep(Centric->Mu, DeltaTime, local);

		GlobalSpatial = Centric->ToAbsoluteState(local);
	}
	
	Transform.SetTranslation(GlobalSpatial.Position * 1e3);

	UCelestialProximitySolver* ProximitySolver =
		CosmosInstance->SubsystemOf<UCelestialProximitySolver>(UCosmosInstance::PROXIMITY_SOLVER);

	//ProximitySolver->UpdateSpatialInfo(this);
	UpdateOrbitalElement();

	// ICelestialBody* capture = ProximitySolver->GetCapture(this);
	// Centric = capture ? Cast<ACelestia>(capture) : nullptr;
}


void ACelestia::AsyncPhysicsTickActor(float DeltaTime, float SimTime) {
	Super::AsyncPhysicsTickActor(DeltaTime, SimTime);

	if (bKinematicObject) {
		return;
	}

	auto T = GetActorTransform();
	
	UpdatePhysicsObjectTransform(DeltaTime, T);
	
	SetActorTransform(T);
}

void ACelestia::LoadInitialState(CelestialStateSource source)
{
	if (!Centric)
	{
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