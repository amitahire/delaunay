#pragma once

#include "math/math.hh"
#include "ptr.hh"
#include <vector>

////////////////////////////////////////////////////////////////////////////////
class TriSoup;

////////////////////////////////////////////////////////////////////////////////
// SignedDistanceField
// - sparse voxelized distances of a mesh. Can be computed using fast sweep
//   or fast marching.
class SignedDistanceField
{
public:
	////////////////////////////////////////////////////////////////////////////////	
	class Iterator;

	////////////////////////////////////////////////////////////////////////////////	
	SignedDistanceField(const TriSoup& triSoup, float resolution);
	~SignedDistanceField();

	void Compute();
	void ComputeTri(int i);
	int NumTris() const;

	const AABB& GetBounds() const { return m_bounds; }
	const AABB GetAlignedBounds() const;
	void GridCoordsFromVec(Vec3_arg pos, int& ix, int& iy, int &iz) const;
	Vec3 CellCenterFromGridCoords(int ix, int iy, int iz) const;
	
	Iterator GetFirst() ;	
private:
	// non-copyable for now
	SignedDistanceField(const SignedDistanceField& other);
	SignedDistanceField& operator=(const SignedDistanceField& other);

	////////////////////////////////////////////////////////////////////////////////	
	struct VoxelGroup;
	struct VoxelLeafBlock;
	struct VoxelOctNode;
	
	////////////////////////////////////////////////////////////////////////////////	
	void ComputeTriangleDistances(int tri, const Vec3 (&verts)[3]);
	void SplatVoxelGroup(
			VoxelGroup *grp,
			int ix, int iy, int iz, int tri,
			const Vec3 (&verts)[3], 
			const Vec3 (&diff)[3],
			const float (&curPlanes)[3], 
			const float (&planeIncX)[3],
			const float (&planeIncY)[3],
			const float (&planeIncZ)[3],
			const float (&distances)[3]);
	VoxelGroup* FindVoxelGroup(int ix, int iy, int iz, VoxelOctNode*& cache);
	
	////////////////////////////////////////////////////////////////////////////////	
	ScopedPtr<TriSoup>::Type m_triSoup;
	AABB m_bounds;
	int m_gridBoundsMin[3];
	int m_gridBoundsMax[3];
	float m_resolution;
	float m_invResolution;
	//std::vector<VoxelLeafBlock*> m_blocks;
	VoxelOctNode *m_root;
};

////////////////////////////////////////////////////////////////////////////////	
class SignedDistanceField::Iterator
{
	friend class SignedDistanceField;
public:
	Iterator(const Iterator& other);
	Iterator& operator=(const Iterator& other);

	bool Valid() const;
	bool NextVoxel();
	bool NextNode();

	Vec3 GetCenter() const;
	float GetDistance() const;
	int GetClosestTri() const;
private:
	////////////////////////////////////////////////////////////////////////////////	
	explicit Iterator(SignedDistanceField *field, VoxelOctNode* top);
	int IndexInParent(VoxelOctNode* node) const;
	VoxelOctNode* FindNextLeaf(VoxelOctNode* top);

	////////////////////////////////////////////////////////////////////////////////	
	SignedDistanceField *m_field;
	VoxelOctNode* m_node;
	int m_pos[3];
};
