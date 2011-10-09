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
		Face()
		{
			for(int i = 0; i < 3; ++i)
				m_vertices[i] = m_edges[i] = -1;
		}

		int m_vertices[3];
		int m_edges[3];

		int IndexOfVert(int vert) const {
			for(int i = 0; i < 3; ++i) {
				if(vert == m_vertices[i])
					return i;
			}
			return -1;
		}
	};

	struct Vertex
	{
		Vertex()
		{}

		Vertex(Vec3_arg pos) : m_pos(pos) {}

		Vec3 m_pos;
		std::vector<int> m_edges;
	};

	int m_flags;
	std::vector<Vertex*> m_vertices;
	std::vector<Face*> m_faces;
	std::vector<HalfEdge*> m_edges;
public:
	enum OptFlags {
		OPT_TRYMATCH = (1 << 0),
	};
	explicit HETriMesh(int flags);
	~HETriMesh();

	// Creation functions
	int AddVertex(const Vec3& pos);
	int AddVertexUnique(const Vec3& pos);
	int AddFace(int v0, int v1, int v2);

	// reading functions
	const Vec3& GetVertexPos(int vertex) const;
	void GetFace(int face, int (&indices)[3]) const;
	int NumFaces() const { return m_faces.size(); }
	int NumVertices() const { return m_vertices.size(); }

private:
	HETriMesh(const HETriMesh&);
	HETriMesh& operator=(const HETriMesh&);

	int AddEdge(int face, int start, int end);
	bool ConnectEdge(int vertex, int edge);
	bool EdgeExistsWithFace(int from, int to) const;
};

