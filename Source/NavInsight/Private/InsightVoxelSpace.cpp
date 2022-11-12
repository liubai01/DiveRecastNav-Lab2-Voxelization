// Fill out your copyright notice in the Description page of Project Settings.

#include "InsightVoxelSpace.h"
#include "EngineUtils.h"
#include "Engine/StaticMesh.h"
#include "NavMesh/RecastNavMeshGenerator.h"
#include "Navmesh/Public/Recast/Recast.h"
#include "NavMesh/RecastHelpers.h"
#include "DrawDebugHelpers.h"
#include "Algo/Reverse.h"

// Sets default values
AInsightVoxelSpace::AInsightVoxelSpace()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AInsightVoxelSpace::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AInsightVoxelSpace::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

struct FInsightGeometryExport
{
	TNavStatArray<FVector> VertexBuffer;
	TNavStatArray<int32> IndexBuffer;
};

void ExportComponentGeo(UActorComponent* Component, FInsightGeometryExport& GeomExport)
{
	UPrimitiveComponent* PrimComp = Cast<UPrimitiveComponent>(Component);
	if (PrimComp && PrimComp->IsNavigationRelevant())
	{

		UBodySetup* BodySetup = PrimComp->GetBodySetup();
		if (BodySetup)
		{
			FRecastNavMeshGenerator::ExportRigidBodyGeometry(*BodySetup, GeomExport.VertexBuffer, GeomExport.IndexBuffer, PrimComp->GetComponentTransform());
		}
	}
}

void AInsightVoxelSpace::InitializeVoxelSpace()
{
	VoxelBBox = GetBounds().GetBox();

	VoxelXNum = static_cast<int>(
		(VoxelBBox.Max.X - VoxelBBox.Min.X) / CellSize + 0.5f
	);
	VoxelYNum = static_cast<int>(
		(VoxelBBox.Max.Y - VoxelBBox.Min.Y) / CellSize + 0.5f
	);
	VoxelZNum = static_cast<int>(
		(VoxelBBox.Max.Z - VoxelBBox.Min.Z) / CellHeight + 0.5f
	);

	const int VoxelNum = VoxelXNum * VoxelYNum * VoxelZNum;
	const int ArrSize = static_cast<int>(0.5f + VoxelNum / 8) + 1; // safe margin 
	
	VoxelsOccupied.SetNum(ArrSize, true);

	for (int i = 0; i < ArrSize; ++i)
	{
		VoxelsOccupied[i] = 0;
	}
	

	FlushPersistentDebugLines(GetWorld());
}

void AInsightVoxelSpace::SetVoxelOccupied(int X, int Y, int Z, bool Flag)
{
	const int NumBit = X * VoxelYNum * VoxelZNum + Y * VoxelZNum + Z;
	const int NumByte = NumBit / 8;
	const int NumBitLeftOver = NumBit - NumByte * 8;

	if (Flag)
	{
		VoxelsOccupied[NumByte] = VoxelsOccupied[NumByte] | (0x1 << NumBitLeftOver);
	}
	else {
		VoxelsOccupied[NumByte] = VoxelsOccupied[NumByte] & ~(0x1 << NumBitLeftOver);
	}
}

bool AInsightVoxelSpace::GetVoxelOccupied(int X, int Y, int Z)
{
	const int NumBit = X * VoxelYNum * VoxelZNum + Y * VoxelZNum + Z;
	const int NumByte = NumBit / 8;
	const int NumBitLeftOver = NumBit - NumByte * 8;

	return (VoxelsOccupied[NumByte] >> NumBitLeftOver) & 0x1;
}

bool AInsightVoxelSpace::IsVoxelInside(int X, int Y, int Z)
{
	if (X < 0 || X >= VoxelXNum)
	{
		return false;
	}
	if (Y < 0 || Y >= VoxelYNum)
	{
		return false;
	}
	if (Z < 0 || Z >= VoxelZNum)
	{
		return false;
	}

	return true;
}


static void DividePoly(FVector* InPoly, int NIn,
					  FVector* OutPolyLeft, int& NOutLeft,
					  FVector* OutPolyRight, int& NOutRight,
					  float X, int Axis)
{
	// max poly edge (corner): 1 * 2 (axis-aligned) + 2 * 3 (triangle edge intersect.) + 4
	float Diff[12];
	for (int i = 0; i < NIn; ++i)
	{
		Diff[i] = X - InPoly[i][Axis];
	}

	NOutLeft = 0;
	NOutRight = 0;

	// Iterate over all vertex indexed by I
	int IIdxPrev = NIn - 1;
	for (int IIdx = 0; IIdx < NIn; ++IIdx)
	{
		// Current line seg. is <I-, I>
		const bool bLeftIPrev = Diff[IIdxPrev] >= 0;
		const bool bLeftI = Diff[IIdx] >= 0;
		// I-, I is at separate line different side
		if (bLeftI != bLeftIPrev)
		{
			// Compute <I-, I> intersection point with X-axis by linear interpolation
			// Add intersection point to Left & Right Polygon
			/*if (Diff[IIdxPrev] - Diff[IIdx] < 1e-3)
			{
				OutPolyLeft[NOutLeft++] = (InPoly[IIdx] + InPoly[IIdxPrev]) / 2;
				OutPolyRight[NOutRight++] = OutPolyLeft[NOutLeft - 1];
			} else {*/
			float s = Diff[IIdxPrev] / (Diff[IIdxPrev] - Diff[IIdx]);
			OutPolyLeft[NOutLeft++] = InPoly[IIdxPrev] + (InPoly[IIdx] - InPoly[IIdxPrev]) * s;
			OutPolyRight[NOutRight++] = OutPolyLeft[NOutLeft - 1];
			//}


			// Add I to corresponding left / right polygon if exactly falls in LHS / RHS
			if (Diff[IIdx] > 0)
			{
				OutPolyLeft[NOutLeft++] = InPoly[IIdx];
			} else if (Diff[IIdx] < 0)
			{
				OutPolyRight[NOutRight++] = InPoly[IIdx];
			}
			
		} else {
			// <I-, I> are both at LHS or RHS of axis
			// At I to corresponding polygon
			
			if (bLeftI)
			{
				// I is at LHS (I- is also at LHS, which is supposed to be added when visiting it)
				OutPolyLeft[NOutLeft++] = InPoly[IIdx];
				
				// Exactly on the axis (add to both polygon)
				if (Diff[IIdx] != 0)
				{
					IIdxPrev = IIdx;
					continue;
				}
			}

			// I is at RHS or exactly on the edge
			OutPolyRight[NOutRight++] = InPoly[IIdx];
		}
		
		IIdxPrev = IIdx;
	}
}
/*template<class T> inline void MySwap(T& a, T& b) { T t = a; a = b; b = t; }*/

void DrawDebugPoly(UWorld* World, FVector* Verts, int N)
{
	int IPrevIdx = N - 1;
	static FColor Color = {0, 0, 255};
	for (int IIdx = 0; IIdx < N; ++IIdx)
	{
		FVector A = Verts[IPrevIdx];
		A.Z += 20.0f;
		FVector B = Verts[IIdx];
		B.Z += 20.0f;
		
		DrawDebugLine(World, A, B, Color,  true);

		IPrevIdx = IIdx;
	}
}

void AInsightVoxelSpace::RasterizeTriangle(FVector A, FVector B, FVector C)
{
	FBox TrBBox;
	TrBBox.Min = C.ComponentMin(A.ComponentMin(B));
	TrBBox.Max = C.ComponentMax(A.ComponentMax(B));

	if (!TrBBox.Intersect(VoxelBBox))
	{
		return;
	}
	// Get voxel Y index
	int Y0 = FMath::FloorToInt((TrBBox.Min[1] - VoxelBBox.Min[1]) / CellSize);
	int Y1 = FMath::CeilToInt((TrBBox.Max[1] - VoxelBBox.Min[1]) / CellSize);
	Y0 = FMath::Clamp(Y0, 0, VoxelYNum - 1);
	Y1 = FMath::Clamp(Y1, 0, VoxelYNum - 1);

	// Clip the triangle into all grid cells it touches.
	// Initialize four polygon positions (<= 7 corners)
	FVector Buf[7 * 4];
	
	FVector* In     = Buf;
	FVector* InRow  = In + 7;
	FVector* P1     = InRow + 7;
	FVector* P2     = P1 + 7;

	In[0] = A;
	In[1] = B;
	In[2] = C;

	int NRow = 3;
	int NIn = 3;

	// Cut along Y-Axis into Rows
	for (int Y = Y0; Y <= Y1; ++Y)
	{
		// Clip polygon to row. Store the remaining polygon as well
		const float ClipY = VoxelBBox.Min.Y + Y * CellSize;

		DividePoly(In, NIn, InRow, NRow, P1, NIn, ClipY, 1);
		Swap(In, P1);

 		if (NRow < 3)
		{
			// Nothing left
			continue;
		}

		float MinX = InRow[0].X;
		float MaxX = InRow[0].X;
		for (int i = 0; i < NRow; ++i)
		{
			if (MinX > InRow[i].X) MinX = InRow[i].X;
			if (MaxX < InRow[i].X) MaxX = InRow[i].X;
		}

		int X0 = FMath::FloorToInt((MinX - VoxelBBox.Min.X) / CellSize);
		int X1 = FMath::CeilToInt((MaxX - VoxelBBox.Min.X) / CellSize);
		X0 = FMath::Clamp(X0, 0, VoxelXNum - 1);
		X1 = FMath::Clamp(X1, 0, VoxelXNum - 1);

		int N, N2 = NRow;

		for (int X = X0; X <= X1; ++X)
		{
			const float CX = VoxelBBox.Min.X + X * CellSize;
			DividePoly(InRow, N2, // cut the row polygon (align with z) along the x-axis
				P1, N, 
				P2, N2, 
				CX + CellSize, 
				0
			);
			Swap(InRow, P2);
			if (N < 3) continue;

			// Calculate min and max of the span.
			//   Remark: the height
			float smin = P1[0].Z, smax = P1[0].Z;
			for (int i = 1; i < N; ++i)
			{
				smin = FMath::Min(smin, P1[i].Z);
				smax = FMath::Max(smax, P1[i].Z);
			}

			// Skip the span if it is outside the bbox
			if (smax < VoxelBBox.Min.Z) continue;
			if (smin > VoxelBBox.Max.Z) continue;
			// Clamp the span to the bbox.
			if (smin < VoxelBBox.Min.Z) smin = VoxelBBox.Min.Z;
			if (smax > VoxelBBox.Max.Z) smax = VoxelBBox.Max.Z;

			smin -= VoxelBBox.Min.Z;
			smax -= VoxelBBox.Min.Z;

			const int ZMin = FMath::Clamp(FMath::FloorToInt(smin / CellHeight), 0, VoxelZNum - 1);
			const int ZMax = FMath::Clamp(FMath::CeilToInt(smax / CellHeight), 0,  VoxelZNum - 1);

			for (int Z = ZMin; Z < ZMax; ++Z)
			{
				SetVoxelOccupied(X, Y, Z, true);
			}
		}
	}
}

void AInsightVoxelSpace::VoxelizeInBox()
{
	InitializeVoxelSpace();

	for (TActorIterator<AActor> ActorItr(GetWorld()); ActorItr; ++ActorItr)
	{

		if (*ActorItr == StartPoint || *ActorItr == EndPoint)
		{
			continue;
		}

		// Get StaticMesh Component
		UStaticMeshComponent* Comp = Cast<UStaticMeshComponent>(
			ActorItr->GetComponentByClass(
				UStaticMeshComponent::StaticClass()
			)
		);

		if (!Comp)
		{
			continue;
		}

		FBox BBoxGeo = Comp->GetNavigationBounds();
		
		if (!(BBoxGeo.Intersect(VoxelBBox)))
		{
			continue;
		}

		FNavigationRelevantData Data(*Comp);
		FInsightGeometryExport GeoExport;

		ExportComponentGeo(Comp, GeoExport);
		
		if (GeoExport.VertexBuffer.Num() == 0)
		{
			continue;
		}
		
		for (int IIdx = 0; IIdx < GeoExport.IndexBuffer.Num() / 3; ++IIdx)
		{
			FVector PosA = GeoExport.VertexBuffer[GeoExport.IndexBuffer[IIdx * 3 + 0]];
			FVector PosB = GeoExport.VertexBuffer[GeoExport.IndexBuffer[IIdx * 3 + 1]];
			FVector PosC = GeoExport.VertexBuffer[GeoExport.IndexBuffer[IIdx * 3 + 2]];
			FColor Color = {0, 255, 0};
			
			RasterizeTriangle(PosA, PosB, PosC);
		}


	}

	VisualizeVoxelSpace();
}

FIntVector AInsightVoxelSpace::ProbeVoxel(FVector& Position)
{
	int X = (Position.X - VoxelBBox.Min.X) / CellSize + 0.5f;
	int Y = (Position.Y - VoxelBBox.Min.Y) / CellSize + 0.5f;
	int Z = (Position.Z - VoxelBBox.Min.Z) / CellHeight + 0.5f;


	if (GetVoxelOccupied(X, Y, Z))
	{
		return {-1, -1, -1};
	}

	int ProbeDist = 3;
	while (!GetVoxelOccupied(X, Y, Z - 1))
	{
		Z -= 1;

		if (--ProbeDist == 0 || Z == -1)
		{
			return {-1, -1, -1};
		}
	}

	return {X, Y, Z};
}


void AInsightVoxelSpace::FindPath()
{
	if (!StartPoint)
	{
		return;
	}

	if (!EndPoint)
	{
		return;
	}

	FVector StartPos = StartPoint->GetActorLocation();
	FVector EndPos = EndPoint->GetActorLocation();

	FIntVector StartIdx = ProbeVoxel(StartPos);
	FIntVector EndIdx = ProbeVoxel(EndPos);

	if (StartIdx.X == -1 && EndIdx.X == -1)
	{
		return;
	}
	
	// BFS
	static int DNum = 6;
	static int Dx[] = {-1, 0, 1, 0, 0, 0};
	static int Dy[] = {0, 1, 0, -1, 0, 0};
	static int Dz[] = {0, 0, 0, 0, -1, 1};
	
	TQueue<FIntVector> Frontier;
	TMap<FIntVector, FIntVector> Prev;
	Frontier.Enqueue(StartIdx);
	Prev.Add(StartIdx, {-1, -1, -1});
	bool Found = false;
	
	while (!Frontier.IsEmpty())
	{
		FIntVector NowIdx;
		Frontier.Dequeue(NowIdx);

		for (int DIdx = 0; DIdx < DNum; ++DIdx)
		{
			FIntVector NextIdx = {
				NowIdx.X + Dx[DIdx],
				NowIdx.Y + Dy[DIdx],
				NowIdx.Z + Dz[DIdx]
			};

			if (!Prev.Find(NextIdx) && IsVoxelInside(NextIdx.X, NextIdx.Y, NextIdx.Z) && IsStayableVoxel(NextIdx.X, NextIdx.Y, NextIdx.Z))
			{
				Frontier.Enqueue(NextIdx);
				Prev.Add(NextIdx, NowIdx);
			}

			if (NextIdx == EndIdx)
			{
				Found = true;
				break;
			}
		}
	}

	if (!Found)
	{
		return;
	}
	
	TArray<FIntVector> Path;
	FIntVector NowIdx = EndIdx;
	while (NowIdx != StartIdx)
	{
		Path.Push(NowIdx);
		NowIdx = Prev[NowIdx];
	}
	Path.Push(StartIdx);
	Algo::Reverse(Path);

	int j = 0;
	FColor PathColor = {0, 255, 255};
	for (int i = 1; i < Path.Num(); ++i)
	{
		FVector PointPosA = {
			Path[i].X * CellSize + VoxelBBox.Min.X,
			Path[i].Y * CellSize + VoxelBBox.Min.Y,
			Path[i].Z * CellHeight + VoxelBBox.Min.Z + CellHeight
		};
		FVector PointPosB = {
			Path[j].X * CellSize + VoxelBBox.Min.X,
			Path[j].Y * CellSize + VoxelBBox.Min.Y,
			Path[j].Z * CellHeight + VoxelBBox.Min.Z + CellHeight
		};
		DrawDebugLine(GetWorld(), PointPosA, PointPosB, PathColor, true);
		j = i;
	}
	
}

bool AInsightVoxelSpace::IsStayableVoxel(int X, int Y, int Z)
{
	if (GetVoxelOccupied(X, Y, Z))
	{
		return false;
	}

	for (int DX = -1; DX <= 1; ++DX)
	{
		for (int Dy = -1; Dy <= 1; ++Dy)
		{
			for (int Dz = -1; Dz <= 0; ++Dz)
			{
				if (DX == 0 && Dy == 0 && Dz == 0)
				{
					continue;
				}
				if (IsVoxelInside(X + DX, Y + Dy, Z + Dz) && GetVoxelOccupied(X + DX, Y + Dy, Z + Dz))
				{
					return true;
				}
			}
		}
	}
	
	return false;
}

void AInsightVoxelSpace::VisualizeVoxelSpace()
{
	
	for (int X = 0; X < VoxelXNum; ++X)
	{
		for (int Y = 0; Y < VoxelYNum; ++Y)
		{
			for (int Z = 0; Z < VoxelZNum; ++Z)
			{
				FVector Center = {
					(X + 0.5f) * CellSize + VoxelBBox.Min.X,
					(Y + 0.5f) * CellSize + VoxelBBox.Min.Y,
					(Z + 0.5f) * CellHeight + VoxelBBox.Min.Z
				};
				FVector Extent = {CellSize, CellSize, CellHeight};
				FColor Color = {255, 0, 0};

				if (GetVoxelOccupied(X, Y, Z))
				{
					DrawDebugBox(GetWorld(), Center, Extent, Color, true, -1);
				}
				
			}
		}
	}
}