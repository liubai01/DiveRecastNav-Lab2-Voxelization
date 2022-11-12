// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Volume.h"
#include "InsightVoxelSpace.generated.h"

UCLASS()
class NAVINSIGHT_API AInsightVoxelSpace : public AVolume
{
	GENERATED_BODY()

	AInsightVoxelSpace();

	UPROPERTY(EditAnywhere, Category = "NavInsight")
	float CellSize = 20.0f;

	UPROPERTY(EditAnywhere, Category = "NavInsight")
	float CellHeight = 50.0f;

	UPROPERTY(EditAnywhere, Category = "NavInsight")
	AActor* StartPoint;

	UPROPERTY(EditAnywhere, Category = "NavInsight")
	AActor* EndPoint;
	
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UFUNCTION(CallInEditor)
	void VoxelizeInBox();

	UFUNCTION(CallInEditor)
	void FindPath();

private:
	TArray<char> VoxelsOccupied;

	FBox VoxelBBox;

	int VoxelXNum;
	int VoxelYNum;
	int VoxelZNum;

	void SetVoxelOccupied(int X, int Y, int Z, bool Flag);
	bool GetVoxelOccupied(int X, int Y, int Z);
	
	bool IsVoxelInside(int X, int Y, int Z);

	void RasterizeTriangle(FVector A, FVector B, FVector C);

	void VisualizeVoxelSpace();

	void InitializeVoxelSpace();

	bool IsStayableVoxel(int X, int Y, int Z);
	
	FIntVector ProbeVoxel(FVector& Position);
};
