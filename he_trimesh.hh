#pragma once

#include "math/vec3.hh"
#include <vector>

// HalfEdge TriMesh 
class HETriMesh
{
	struct Vertex;
	struct Face;
	struct HalfEdge;

	struct HalfEdge
	{
		HalfEdge() 
			: m_dual(-1)
			, m_face(-1)
			, m_from(-1)
			, m_to(-1)
		{}

		int m_dual;	
		int m_face;
		int m_from;
		int m_to;
	};

	struct Face
	{
		Face() : m_payload(0)
		{
			for(int i = 0; i < 3; ++i)
				m_vertices[i] = m_edges[i] = -1;
		}

		~Face() {
			delete[] m_payload;
		}

		char * m_payload;
		int m_vertices[3];
		int m_edges[3];

		int IndexOfVert(int vert) const {
			for(int i = 0; i < 3; ++i) {
				if(vert == m_vertices[i])
					return i;
			}
			return -1;
		}

		int IndexOfEdge(int edge) const {
			for(int i = 0; i < 3; ++i) {
				if(edge == m_edges[i])
					return i;
			}
			return -1;
		}
	};

	struct Vertex
	{
		Vertex() : m_payload(0)
		{}
		~Vertex() { delete[] m_payload; }

		Vertex(Vec3_arg pos, char * payload) : m_pos(pos), m_payload(payload) {}

		Vec3 m_pos;
		char * m_payload;
		std::vector<int> m_edges;

		bool RemoveEdge(int edgeIdx);
		int IndexOfEdge(int edge) const {
			for(int i = 0, c = m_edges.size(); i < c; ++i)
				if(m_edges[i] == edge)
					return i;
			return -1;
		}
	};

	int m_flags;
	std::vector<Vertex*> m_vertices;
	std::vector<Face*> m_faces;
	std::vector<HalfEdge*> m_edges;
public:
	enum OptFlags {
		OPT_TRYMATCH = (1 << 0),
		OPT_INITPAYLOAD = (1 << 1),
		OPT_REPLACE_SMALLER_ON_CONFLICT = (1 << 2),
		OPT_IGNORE_SMALL = (1 << 3),
	};
	explicit HETriMesh(int flags = 0);
	~HETriMesh();

	// Creation functions
	int AddVertex(const Vec3& pos, int payloadSize = 0);
	int AddVertexUnique(const Vec3& pos, int payloadSize = 0);
	int AddFace(int v0, int v1, int v2, int payloadSize = 0);

	// Modification functions
	void DeleteFace(int index);

	// reading functions
	const Vec3& GetVertexPos(int vertex) const;
	void GetFace(int face, int (&indices)[3]) const;
	int NumFaces() const { return m_faces.size(); }
	int NumVertices() const { return m_vertices.size(); }
	char * GetVertexData(int index) ;
	const char * GetVertexData(int index) const ;
	char * GetFaceData(int index) ;
	const char * GetFaceData(int index) const ;

private:
	HETriMesh(const HETriMesh&);
	HETriMesh& operator=(const HETriMesh&);

	int AddEdge(int face, int start, int end);
	bool ConnectEdge(int vertex, int edge);
	bool EdgeExistsWithFace(int from, int to) const;
	int FindEdge(int from, int to) const;

	void RemoveEdge(int edgeIndex);
	void MoveEdge(int newIndex, int oldIndex);
	void MoveFace(int newIndex, int oldIndex);
	
	void DebugVerify();
};

