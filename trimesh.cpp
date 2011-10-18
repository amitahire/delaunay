#include "common.hh"
#include "trimesh.hh"
#include "math/math.hh"
#include <set>

////////////////////////////////////////////////////////////////////////////////	
TriMesh::Edge::Edge(int from, int to)
	: m_from(-1)
	, m_to(-1)
	, m_firstFaceEdge(-1)
{
	m_from = Min(from, to);
	m_to = Max(from, to);
	m_faces[1] = m_faces[0] = -1;
}
		
bool TriMesh::Edge::operator<(const Edge& other) const
{
	if(m_from == other.m_from)
		return m_to < other.m_to;
	else return m_from < other.m_from;
}

////////////////////////////////////////////////////////////////////////////////	
struct TriMesh::Vertex
{
	Vertex() : m_pos(), m_payload(0), m_payloadSize(0), m_firstFace(-1) {}
	Vertex(const Vertex& other)
		: m_pos(other.m_pos)
		, m_payload( other.m_payloadSize > 0 ? new char[other.m_payloadSize] : 0)
		, m_payloadSize(other.m_payloadSize)
		, m_firstFace(other.m_firstFace)
	{
		if(m_payloadSize > 0)
			memcpy(m_payload, other.m_payload, m_payloadSize);
	}

	Vertex& operator=(const Vertex& other)
	{	
		if(this != &other)
		{
			m_pos = other.m_pos;
			m_payload = 0;
			m_payloadSize = other.m_payloadSize;
			m_firstFace = other.m_firstFace;

			if(m_payloadSize > 0)
			{
				m_payload = new char[m_payloadSize];
				memcpy(m_payload, other.m_payload, m_payloadSize);
			}
		}

		return *this;
	}

	Vec3 m_pos;
	ScopedPtrAry<char>::Type m_payload;
	int m_payloadSize;
	int m_firstFace;
};

////////////////////////////////////////////////////////////////////////////////	
enum FaceFlags
{
	FACE_NONMANIFOLD = (1 << 0)
};

struct TriMesh::Face
{
	Face() 
		: m_payload(0)
		, m_payloadSize(0)
		, m_flags(0)
	{
		for(int i = 0; i < 3; ++i)
		{
			m_vertices[i] = -1;
			m_faces[i] = -1;
		}
	}

	Face(const Face& other)
		: m_payload(other.m_payloadSize > 0 ? new char[other.m_payloadSize] : 0)
		, m_payloadSize(other.m_payloadSize)
		, m_flags(other.m_flags)
	{
		if(m_payloadSize > 0)
			memcpy(m_payload, other.m_payload, m_payloadSize);
	}

	Face& operator=(const Face& other)
	{
		if(this != &other)
		{
			m_payload = 0;
			m_payloadSize = other.m_payloadSize;
			m_flags = other.m_flags;
			memcpy(m_vertices, other.m_vertices, sizeof(m_vertices));
			memcpy(m_faces, other.m_faces, sizeof(m_faces));

			if(m_payloadSize > 0)
			{
				m_payload = new char[m_payloadSize];
				memcpy(m_payload, other.m_payload, m_payloadSize);
			}
		}
		return *this;
	}

	ScopedPtrAry<char>::Type m_payload;
	int m_payloadSize;
	int m_flags;
	int m_vertices[3];
	int m_faces[3];
};


////////////////////////////////////////////////////////////////////////////////	
TriMesh::TriMesh()
	: m_edges()
	, m_vertices()
	, m_faces()
{
}

TriMesh::~TriMesh()
{
	Clear();
}

void TriMesh::Clear()
{
	m_edges.clear();

	for(int i = 0, c = m_vertices.size(); i < c; ++i)
		delete m_vertices[i];
	
	for(int i = 0, c = m_faces.size(); i < c; ++i)
		delete m_faces[i];

	m_vertices.clear();
	m_faces.clear();
}

TriMesh::TriMesh(const TriMesh& other)
	: m_edges(other.m_edges)
	, m_vertices(other.m_vertices.size())
	, m_faces(other.m_faces.size())
{
	for(int i = 0, c = m_vertices.size(); i < c; ++i)
		m_vertices[i] = new Vertex(*other.m_vertices[i]);

	for(int i = 0, c = m_faces.size(); i < c; ++i)
		m_faces[i] = new Face(*other.m_faces[i]);
}

TriMesh& TriMesh::operator=(const TriMesh& other)
{
	if(this != &other)
	{
		Clear();
		m_edges = other.m_edges;
		m_vertices.resize(other.m_vertices.size());
		m_faces.resize(other.m_faces.size());
		
		for(int i = 0, c = m_vertices.size(); i < c; ++i)
			m_vertices[i] = new Vertex(*other.m_vertices[i]);

		for(int i = 0, c = m_faces.size(); i < c; ++i)
			m_faces[i] = new Face(*other.m_faces[i]);
	}
	return *this;
}


int TriMesh::AllocateVertex()
{
	int idx = m_vertices.size();
	m_vertices.push_back(new Vertex);
	return idx;
}

int TriMesh::AllocateFace()
{
	int idx = m_faces.size();
	m_faces.push_back(new Face);
	return idx;
}

void TriMesh::DeleteLastFace()
{
	delete m_faces[m_faces.size() - 1];
	m_faces.pop_back();
}

int TriMesh::AddVertex(const Vec3& pos, int payloadSize)
{
	int idx = AllocateVertex();
	Vertex* vertex = m_vertices[idx];
	vertex->m_pos = pos;
	if(payloadSize > 0)
	{
		vertex->m_payload = new char[payloadSize];
		vertex->m_payloadSize = payloadSize;
		memset(vertex->m_payload, 0, payloadSize);
	}
	return idx;
}

int TriMesh::AddVertexUnique(const Vec3& pos, int payloadSize)
{
	for(int i = 0, c = m_vertices.size(); i < c; ++i)
	{
		if(m_vertices[i]->m_pos == pos)
			return i;
	}
	return AddVertex(pos, payloadSize);
}

int TriMesh::AddFace(int v0, int v1, int v2, int payloadSize)
{
	int verts[] = { v0, v1, v2 };
	int faceIdx = AllocateFace();
	Face* face = m_faces[faceIdx];

	// Fixup face Verts
	for(int i = 0; i < 3; ++i)
	{
		face->m_vertices[i] = verts[i];
	}

	// Fixup edges
	for(int i = 0; i < 3; ++i)
	{
		Edge edge(face->m_vertices[i], face->m_vertices[(i+1)%3]);
		std::set<Edge>::iterator iter = m_edges.find(edge);
		if(m_edges.end() == iter)
		{
			edge.m_firstFaceEdge = i;
			edge.m_faces[0] = faceIdx;
			m_edges.insert(edge);
		}
		else
		{
			ASSERT(iter->m_faces[0] >= 0);
			if(iter->m_faces[1] >= 0)
			{
				// The solution is going to be to simple remove multiple incident faces, and
				// fill resulting holes with generated triangles. So, mark both faces
				// as having manifold problems, but refuse to add this face.
				Face* face0 = m_faces[iter->m_faces[0]];
				Face* face1 = m_faces[iter->m_faces[1]];
				face0->m_flags |= FACE_NONMANIFOLD;
				face1->m_flags |= FACE_NONMANIFOLD;

				DeleteLastFace();

				return -1;
			}
			else
			{
				edge = *iter;
				edge.m_faces[1] = faceIdx;
				Face* otherFace = m_faces[edge.m_faces[0]];
				// Connect the faces to each other on the corresponding edge.
				otherFace->m_faces[edge.m_firstFaceEdge] = faceIdx;
				face->m_faces[i] = edge.m_faces[0];
				m_edges.erase(edge);
				m_edges.insert(edge);
			}
		}
	}

	// Fixup verts in the mesh (requires the previous edge section to complete)
	for(int i = 0; i < 3; ++i)
	{
		Vertex* vertex = m_vertices[verts[i]];
		vertex->m_firstFace = faceIdx;
	}

	// Fixup payload
	if(payloadSize > 0)
	{
		face->m_payload = new char[payloadSize];
		face->m_payloadSize = payloadSize;
	}
	return faceIdx;
}

const Vec3& TriMesh::GetVertexPos(int vertex) const
{
	return m_vertices[vertex]->m_pos;
}

void TriMesh::GetFace(int faceIdx, int (&indices)[3]) const
{
	Face* face = m_faces[faceIdx];
	for(int i = 0; i < 3; ++i)
		indices[i] = face->m_vertices[i];
}

char * TriMesh::GetVertexData(int index) 
{
	Vertex* vertex = m_vertices[index];
	return vertex->m_payload;
}

const char * TriMesh::GetVertexData(int index) const 
{
	const Vertex* vertex = m_vertices[index];
	return vertex->m_payload;
}

const char * TriMesh::GetFaceData(int index) const 
{
	const Face* face = m_faces[index];
	return face->m_payload;
}

char * TriMesh::GetFaceData(int index) 
{
	Face* face = m_faces[index];
	return face->m_payload;
}
