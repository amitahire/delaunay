#pragma once

#include "math/vec3.hh"
#include "ptr.hh"
#include <vector>
#include <set>

////////////////////////////////////////////////////////////////////////////////
// TriMesh
// - triangle mesh with some topology information.
class TriMesh
{
public:
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

	// Accessor Functions
	const Vec3& GetVertexPos(int vertex) const;
	
	void GetFace(int face, int (&indices)[3]) const;
	int NumFaces() const { return m_faces.size(); }
	int NumVertices() const { return m_vertices.size(); }

	char * GetVertexData(int index) ;
	const char * GetVertexData(int index) const ;
	char * GetFaceData(int index) ;
	const char * GetFaceData(int index) const ;
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
	void DeleteLastFace();
	bool AddEdge(int start, int end);

	////////////////////////////////////////////////////////////////////////////////	
	std::set<Edge> m_edges;
	std::vector<Vertex*> m_vertices;
	std::vector<Face*> m_faces;
};

