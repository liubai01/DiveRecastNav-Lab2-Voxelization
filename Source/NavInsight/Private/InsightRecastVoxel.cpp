// Fill out your copyright notice in the Description page of Project Settings.


#include "InsightRecastVoxel.h"
#include "Engine/StaticMesh.h"
#include "NavMesh/RecastNavMeshGenerator.h"
#include "Navmesh/Public/Recast/Recast.h"
#include "NavMesh/RecastHelpers.h"
#include "DrawDebugHelpers.h"
#include "NavigationSystem.h"
#include "NavMesh/RecastNavMesh.h"

namespace InsightRecast
{
	struct FRecastGeometry
	{
		struct FHeader
		{
			FNavigationRelevantData::FCollisionDataHeader Validation;

			int32 NumVerts;
			int32 NumFaces;
			struct FWalkableSlopeOverride SlopeOverride;

			static uint32 StaticMagicNumber;
		};

		FHeader Header;

		/** recast coords of Vertices (size: NumVerts * 3) */
		float* Verts;

		/** vert indices for triangles (size: NumFaces * 3) */
		int32* Indices;

		FRecastGeometry() {}
		FRecastGeometry(const uint8* Memory);
	};

	FRecastGeometry::FRecastGeometry(const uint8* Memory)
	{
		Header = *((FHeader*)Memory);
		Verts = (float*)(Memory + sizeof(FRecastGeometry));
		Indices = (int32*)(Memory + sizeof(FRecastGeometry) + (sizeof(float) * Header.NumVerts * 3));
	}
}

void AInsightRecastVoxel::CreateNewHeightField(FBox& BBox)
{
	if (HeightField)
	{
		rcFreeHeightField(HeightField);
	}

	int HFWidth = 0;
	int HFHeight = 0;

	rcCalcGridSize(&BBox.Min.X, &BBox.Max.X, CellSize, &HFWidth, &HFHeight);

	HeightField = rcAllocHeightfield();
	rcCreateHeightfield(nullptr, *HeightField, HFWidth, HFHeight, &BBox.Min.X, &BBox.Max.X, CellSize, CellHeight);
}

void AInsightRecastVoxel::RasterizeMeshToHeightField()
{
	if (!TargetMesh)
	{
		return;
	}

	// Get StaticMesh Component
	UStaticMeshComponent* Comp = Cast<UStaticMeshComponent>(
		TargetMesh->GetComponentByClass(
			UStaticMeshComponent::StaticClass()
		)
		);

	if (!Comp)
	{
		return;
	}

	FNavigationRelevantData Data(*TargetMesh);

	FRecastNavMeshGenerator::ExportComponentGeometry(Comp, Data);
	const TNavStatArray<uint8>& RawCollisionCache = Data.CollisionData;
	if (RawCollisionCache.Num() == 0)
	{
		return;
	}

	InsightRecast::FRecastGeometry CollisionCache(RawCollisionCache.GetData());

	const int32 NumCoords = CollisionCache.Header.NumVerts * 3;
	const int32 NumIndices = CollisionCache.Header.NumFaces * 3;

	FBox BBox = Comp->GetNavigationBounds();
	BBox = Unreal2RecastBox(BBox);

	// Add Padding
	static float Padding = 10.0f;
	BBox.Min -= {Padding, Padding, Padding};
	BBox.Max += {Padding, Padding, Padding};

	rcContext Context;

	double TimeStart = FPlatformTime::Seconds();

	CreateNewHeightField(BBox);

	for (int i = 0; i < NumIndices; i += 3)
	{
		int32 ia = CollisionCache.Indices[i];
		int32 ib = CollisionCache.Indices[i + 1];
		int32 ic = CollisionCache.Indices[i + 2];

		FVector PosA(CollisionCache.Verts[ia * 3 + 0], CollisionCache.Verts[ia * 3 + 1], CollisionCache.Verts[ia * 3 + 2]);
		FVector PosB(CollisionCache.Verts[ib * 3 + 0], CollisionCache.Verts[ib * 3 + 1], CollisionCache.Verts[ib * 3 + 2]);
		FVector PosC(CollisionCache.Verts[ic * 3 + 0], CollisionCache.Verts[ic * 3 + 1], CollisionCache.Verts[ic * 3 + 2]);

		rcRasterizeTriangle(&Context, &PosA.X, &PosB.X, &PosC.X, 0, *HeightField);
	}

	double TimeEnd = FPlatformTime::Seconds();

	BuildTime = TimeEnd - TimeStart;
}

void AInsightRecastVoxel::LoadNavConfig()
{
	// Get Recast NavMesh to keep config consisten with navmesh (e.g.: Cell Size, Cell Height, etc.)
	const UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (!NavSys)
	{
		// Should not happen at most time
		return;
	}

	ANavigationData* NavData = NavSys->GetDefaultNavDataInstance();
	if (!NavSys)
	{
		// Reach here due to in-approriate config. in most time
		return;
	}

	ARecastNavMesh* RecastNavData = Cast<ARecastNavMesh>(NavData);
	CellSize = RecastNavData->CellSize;
	CellHeight = RecastNavData->CellHeight;
}

void AInsightRecastVoxel::VisualizeHeightField() const
{
	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UV0;
	TArray<FProcMeshTangent> Tangents;
	TArray<FLinearColor> VertexColors;

	FVector Position = GetActorLocation();

	// Visualize a span (with a procedural cube)
	auto DrawSpan = [&](rcSpan& Span, int x, int y) {
		static float BBoxPadding = 2.0f;

		FVector BBoxMin(HeightField->bmin[0], HeightField->bmin[1], HeightField->bmin[2]);

		BBoxMin.X += HeightField->cs * x + BBoxPadding;
		BBoxMin.Z += HeightField->cs * y + BBoxPadding;
		BBoxMin.Y += HeightField->ch * (Span.data.smin);

		FVector BBoxMax(
			BBoxMin.X + HeightField->cs - BBoxPadding * 2.0f,
			BBoxMin.Y + (Span.data.smax - Span.data.smin) * HeightField->ch,
			BBoxMin.Z + HeightField->cs - BBoxPadding * 2.0f
		);

		BBoxMin = Recast2UnrealPoint(BBoxMin) - Position;
		BBoxMax = Recast2UnrealPoint(BBoxMax) - Position;

		const float XMin = BBoxMin.X;
		const float XMax = BBoxMax.X;
		const float YMin = BBoxMin.Y;
		const float YMax = BBoxMax.Y;
		const float ZMin = BBoxMin.Z;
		const float ZMax = BBoxMax.Z;

		const int IndexBase = Vertices.Num();

		// Bottom
		Vertices.Add(FVector(XMin, YMin, ZMin));
		Vertices.Add(FVector(XMax, YMin, ZMin));
		Vertices.Add(FVector(XMax, YMax, ZMin));
		Vertices.Add(FVector(XMin, YMax, ZMin));
		// Front
		Vertices.Add(FVector(XMin, YMin, ZMin));
		Vertices.Add(FVector(XMax, YMin, ZMin));
		Vertices.Add(FVector(XMax, YMin, ZMax));
		Vertices.Add(FVector(XMin, YMin, ZMax));
		// Right
		Vertices.Add(FVector(XMax, YMin, ZMin));
		Vertices.Add(FVector(XMax, YMax, ZMin));
		Vertices.Add(FVector(XMax, YMin, ZMax));
		Vertices.Add(FVector(XMax, YMax, ZMax));
		// Left
		Vertices.Add(FVector(XMin, YMin, ZMin));
		Vertices.Add(FVector(XMin, YMax, ZMin));
		Vertices.Add(FVector(XMin, YMax, ZMax));
		Vertices.Add(FVector(XMin, YMin, ZMax));
		// Back
		Vertices.Add(FVector(XMin, YMax, ZMin));
		Vertices.Add(FVector(XMax, YMax, ZMin));
		Vertices.Add(FVector(XMax, YMax, ZMax));
		Vertices.Add(FVector(XMin, YMax, ZMax));
		// Top
		Vertices.Add(FVector(XMin, YMin, ZMax));
		Vertices.Add(FVector(XMax, YMin, ZMax));
		Vertices.Add(FVector(XMax, YMax, ZMax));
		Vertices.Add(FVector(XMin, YMax, ZMax));


		// Bottom
		Triangles.Add(IndexBase + 0);
		Triangles.Add(IndexBase + 1);
		Triangles.Add(IndexBase + 3);
		Triangles.Add(IndexBase + 1);
		Triangles.Add(IndexBase + 2);
		Triangles.Add(IndexBase + 3);
		// Front
		Triangles.Add(IndexBase + 4);
		Triangles.Add(IndexBase + 6);
		Triangles.Add(IndexBase + 5);
		Triangles.Add(IndexBase + 4);
		Triangles.Add(IndexBase + 7);
		Triangles.Add(IndexBase + 6);
		// Right
		Triangles.Add(IndexBase + 8);
		Triangles.Add(IndexBase + 10);
		Triangles.Add(IndexBase + 9);
		Triangles.Add(IndexBase + 9);
		Triangles.Add(IndexBase + 10);
		Triangles.Add(IndexBase + 11);
		// Left
		Triangles.Add(IndexBase + 12);
		Triangles.Add(IndexBase + 13);
		Triangles.Add(IndexBase + 14);
		Triangles.Add(IndexBase + 12);
		Triangles.Add(IndexBase + 14);
		Triangles.Add(IndexBase + 15);
		// Back
		Triangles.Add(IndexBase + 16);
		Triangles.Add(IndexBase + 17);
		Triangles.Add(IndexBase + 18);
		Triangles.Add(IndexBase + 16);
		Triangles.Add(IndexBase + 18);
		Triangles.Add(IndexBase + 19);
		// Top
		Triangles.Add(IndexBase + 22);
		Triangles.Add(IndexBase + 21);
		Triangles.Add(IndexBase + 20);
		Triangles.Add(IndexBase + 20);
		Triangles.Add(IndexBase + 23);
		Triangles.Add(IndexBase + 22);

		// Bottom
		Normals.Add(FVector(0, 0, -1));
		Normals.Add(FVector(0, 0, -1));
		Normals.Add(FVector(0, 0, -1));
		Normals.Add(FVector(0, 0, -1));
		// Front
		Normals.Add(FVector(0, -1, 0));
		Normals.Add(FVector(0, -1, 0));
		Normals.Add(FVector(0, -1, 0));
		Normals.Add(FVector(0, -1, 0));
		// Right
		Normals.Add(FVector(1, 0, 0));
		Normals.Add(FVector(1, 0, 0));
		Normals.Add(FVector(1, 0, 0));
		Normals.Add(FVector(1, 0, 0));
		// Left
		Normals.Add(FVector(-1, 0, 0));
		Normals.Add(FVector(-1, 0, 0));
		Normals.Add(FVector(-1, 0, 0));
		Normals.Add(FVector(-1, 0, 0));
		// Back
		Normals.Add(FVector(0, 1, 0));
		Normals.Add(FVector(0, 1, 0));
		Normals.Add(FVector(0, 1, 0));
		Normals.Add(FVector(0, 1, 0));
		// Top
		Normals.Add(FVector(0, 0, 1));
		Normals.Add(FVector(0, 0, 1));
		Normals.Add(FVector(0, 0, 1));
		Normals.Add(FVector(0, 0, 1));

		// Bottom
		Tangents.Add(FProcMeshTangent(1, 0, 0));
		Tangents.Add(FProcMeshTangent(1, 0, 0));
		Tangents.Add(FProcMeshTangent(1, 0, 0));
		Tangents.Add(FProcMeshTangent(1, 0, 0));
		// Front
		Tangents.Add(FProcMeshTangent(0, 0, 1));
		Tangents.Add(FProcMeshTangent(0, 0, 1));
		Tangents.Add(FProcMeshTangent(0, 0, 1));
		Tangents.Add(FProcMeshTangent(0, 0, 1));
		// Right
		Tangents.Add(FProcMeshTangent(0, 0, 1));
		Tangents.Add(FProcMeshTangent(0, 0, 1));
		Tangents.Add(FProcMeshTangent(0, 0, 1));
		Tangents.Add(FProcMeshTangent(0, 0, 1));
		// Left
		Tangents.Add(FProcMeshTangent(0, 1, 0));
		Tangents.Add(FProcMeshTangent(0, 1, 0));
		Tangents.Add(FProcMeshTangent(0, 1, 0));
		Tangents.Add(FProcMeshTangent(0, 1, 0));
		// Back
		Tangents.Add(FProcMeshTangent(0, 0, 1));
		Tangents.Add(FProcMeshTangent(0, 0, 1));
		Tangents.Add(FProcMeshTangent(0, 0, 1));
		Tangents.Add(FProcMeshTangent(0, 0, 1));
		// Top
		Tangents.Add(FProcMeshTangent(1, 0, 0));
		Tangents.Add(FProcMeshTangent(1, 0, 0));
		Tangents.Add(FProcMeshTangent(1, 0, 0));
		Tangents.Add(FProcMeshTangent(1, 0, 0));

		const FLinearColor Color = { 1.0, 1.0, 0.0, 1.0 };
		for (int i = 0; i < 24; i++)
		{
			VertexColors.Add(Color);
		}
	};

	// Traverse all the span to visualize
	for (int x = 0; x < HeightField->width; x++)
	{
		for (int y = 0; y < HeightField->height; y++)
		{
			int idx = x + y * HeightField->width;
			rcSpan* nowSpan = HeightField->spans[idx];

			while (nowSpan)
			{
				DrawSpan(*nowSpan, x, y);

				nowSpan = nowSpan->next;
			}
		}
	}
	Mesh->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, UV0, VertexColors, Tangents, false);
	if (VolMaterial)
	{
		Mesh->SetMaterial(0, VolMaterial);
	}
}

void AInsightRecastVoxel::ComputeVoxelOfTargetMesh()
{
	// Load CellSize and CellHeight consistent with nav system config
	LoadNavConfig();

	// Rasterize Mesh To the Height Field
	RasterizeMeshToHeightField();

	// Visualize the Height Field
	VisualizeHeightField();
}

// Sets default values
AInsightRecastVoxel::AInsightRecastVoxel()
{
	Mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("VoxelMesh"));
	RootComponent = Mesh;

 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AInsightRecastVoxel::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AInsightRecastVoxel::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

