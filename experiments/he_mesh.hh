#pragma once

#include "math/vec3.hh"
#include "ptr.hh"
#include <vector>

// HalfEdge Mesh
class HEMesh
{
	struct Vertex;
	struct Face;
	struct HalfEdge;

	struct HalfEdge
	{
		HalfEdge() ;

		int m_dual;		
		int m_face;		// face defined by CCW vertex ordering
		int m_from;
		int m_to;
		int m_next;		// CW next edge around a vertex
		int m_prev;		// CW prev edge around a vertex
	};

	struct Face
	{
		Face() : m_payload(0), m_edge(-1) {}

		ScopedPtrAry<char>::Type m_payload;
		int m_edge;

		int IndexOfVert(const std::vector<HalfEdge*>& edges, int vert) const;
		int IndexOfEdge(const std::vector<HalfEdge*>& edges, int edge) const;
	private:
		Face(const Face&);
		Face& operator=(const Face&);
	};

	struct Vertex
	{
		Vertex() : m_pos(), m_payload(0), m_firstEdge(-1) {}

		Vertex(Vec3_arg pos, char * payload) : m_pos(pos), m_payload(payload), m_firstEdge(-1) {}

		Vec3 m_pos;
		ScopedPtrAry<char>::Type m_payload;
		int m_firstEdge;

		int IndexOfEdge(const std::vector<HalfEdge*>& edges, int edge) const ;
	private:
		Vertex(const Vertex&);
		Vertex& operator=(const Vertex&);
	};

	int m_flags;
	std::vector<Vertex*> m_vertices;
	std::vector<Face*> m_faces;
	std::vector<HalfEdge*> m_edges;
public:
	enum OptFlags {
		OPT_TRYMATCH = (1 << 0),
		OPT_INITPAYLOAD = (1 << 1)
	};
	explicit HEMesh(int flags = OPT_INITPAYLOAD);
	~HEMesh();

	// Creation functions
	int AddVertex(const Vec3& pos, int payloadSize = 0);
	int AddVertexUnique(const Vec3& pos, int payloadSize = 0);
	int AddFace(int* verts, int num, int payloadSize = 0);

	// Modification functions
	void DeleteFace(int index);

	// reading functions
	const Vec3& GetVertexPos(int vertex) const;
	int GetFace(int face, int * indices, int maxIndices) const;
	int NumFaces() const { return m_faces.size(); }
	int NumVertices() const { return m_vertices.size(); }
	char * GetVertexData(int index) ;
	const char * GetVertexData(int index) const ;
	char * GetFaceData(int index) ;
	const char * GetFaceData(int index) const ;

private:
	HEMesh(const HEMesh&);
	HEMesh& operator=(const HEMesh&);

	int AddEdge(int faceIdx, int start, int end);
	void FindEdgeInsert(int vertex, int edge, int& insertAfter) const;
	void InsertEdge(int vertexIdx, int edge, int edgeInsert);
	bool EdgeExistsWithFace(int from, int to) const;
	int FindEdge(int from, int to) const;
	Vec3 GetWindingNormal(int edgeIdx) const;

	bool EdgeHasFaces(int edgeIndex) const;
	void RemoveEdge(int edgeIndex);
	void MoveEdge(int newIndex, int oldIndex);
	void MoveFace(int newIndex, int oldIndex);
	
	void DebugVerify();
};

