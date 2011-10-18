#pragma once

#include "math/vec3.hh"
#include "ptr.hh"
#include <vector>
#include <set>

////////////////////////////////////////////////////////////////////////////////
// TriMesh
// - triangle mesh with some topology information.
// - The current state of this class: It's great for topologically correct meshes, 
//   but the hole filling just doesnt' work correctly. Since I'm not attempting
//   to solve that problem for the time being, I'm going to move on to other methods.
class TriMesh
{
public:
	////////////////////////////////////////////////////////////////////////////////	
	enum FaceFlags
	{
		FACE_NONMANIFOLD = (1 << 0),
		FACE_IGNORE_LOOP0 = (1 << 1),
		FACE_IGNORE_LOOP1 = (1 << 2),
		FACE_IGNORE_LOOP2 = (1 << 3),

		FACE_HOLE_FILLER = (1 << 4),

		FACE_IGNORE_ANY_LOOP = 0x0E
	};

	enum VertexFlags
	{
		VERTEX_BOUNDARY = (1 << 0)
	};

	////////////////////////////////////////////////////////////////////////////////	
	TriMesh();
	~TriMesh();
	TriMesh(const TriMesh& other);
	TriMesh& operator=(const TriMesh& other);

	void Clear();

	// Creation functions 
	int AddVertex(const Vec3& pos, int payloadSize = 0);
	int AddVertexUnique(const Vec3& pos, int payloadSize = 0);
	int AddFace(int v0, int v1, int v2, int payloadSize = 0);

	// Modification Functions
	void DeleteFace(int index);
	int CleanBoundaries();
	int FillHoles();

	// Accessor Functions
	const Vec3& GetVertexPos(int vertex) const;
	
	void GetFace(int face, int (&indices)[3]) const;
	int NumFaces() const { return m_faces.size(); }
	int NumVertices() const { return m_vertices.size(); }

	int GetVertexFlags(int index) const;
	int GetFaceFlags(int faceIndex) const;

	char * GetVertexData(int index) ;
	const char * GetVertexData(int index) const ;
	char * GetFaceData(int index) ;
	const char * GetFaceData(int index) const ;

	// Traversal functions
	int GetNeighborFaceIndex(int face, int neighborFace) const;	// returns 0..2, -1 on error
	int GetVertexNumInFace(int face, int vertexIdx) const; // returns 0..2, -1 on error
	int GetVertexValency(int vertex) const; 
	int GetOneRing(int vertexIndex, int* ring, int max) const ;
	int GetNextFace(int faceIdx, int vertexFrom) const;
	int GetPrevFace(int faceIdx, int vertexFrom) const;
	int GetFaceNeighbor(int faceIdx, int fnum) const;
private:
	
	////////////////////////////////////////////////////////////////////////////////	
	struct Edge
	{
		Edge(int from, int to) ;
		bool operator<(const Edge& other) const;

		int m_from;
		int m_to;

		int m_firstFaceEdge;
		int m_faces[2];
	};

	////////////////////////////////////////////////////////////////////////////////	
	struct Vertex;
	struct Face;

	////////////////////////////////////////////////////////////////////////////////	
	int AllocateVertex();
	int AllocateFace();
	bool AddEdge(int start, int end);
	void AttemptCreateEdges(int faceIdx, bool markNonManifold, bool &nonManifold, bool &wrongFacing);
	void MoveFace(int destIndex, int srcIndex);
	int GetBoundaryLoop(int faceIdx, int vertexIdx, int* boundaryLoop, int maxLoopSize) const;
	bool FillHole(int* boundaryLoop, int loopSize);
	bool FaceAdjacent(int face, int adjVert) const;
	int RemoveProblemTriangles();
	int RemoveUnreachableTriangles();

	////////////////////////////////////////////////////////////////////////////////	
	std::set<Edge> m_edges;
	std::vector<Vertex*> m_vertices;
	std::vector<Face*> m_faces;
};

