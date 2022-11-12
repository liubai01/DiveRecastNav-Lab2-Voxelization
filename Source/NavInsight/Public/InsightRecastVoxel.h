// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Navmesh/Public/Recast/Recast.h"
#include "ProceduralMeshComponent.h"
#include "InsightRecastVoxel.generated.h"

UCLASS()
class NAVINSIGHT_API AInsightRecastVoxel : public AActor
{
	GENERATED_BODY()
	
public:
	// Visualize the voxelization of a target mesh
	UPROPERTY(EditAnywhere, Category = "NavInsight")
	AActor* TargetMesh;

	UPROPERTY(EditAnywhere, Category = "NavInsight")
	UMaterial* VolMaterial;

	UPROPERTY(VisibleAnywhere, Category = "NavInsight")
	double BuildTime = 0.0f;

	// Sets default values for this actor's properties
	AInsightRecastVoxel();

	// Compute and visualize target mesh based on recast navigation
	UFUNCTION(CallInEditor, Category = "NavInsight")
	void ComputeVoxelOfTargetMesh();

	rcHeightfield* HeightField;

	void CreateNewHeightField(FBox& BBox);

private:
	float CellSize = 25.0f;

	float CellHeight = 50.0f;

	UProceduralMeshComponent* Mesh;

	void RasterizeMeshToHeightField();

	void LoadNavConfig();

	void VisualizeHeightField() const;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
