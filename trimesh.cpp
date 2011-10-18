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
	Vertex() : m_pos(), m_payload(0), m_payloadSize(0), m_firstFace(-1), m_boundary(false) {}
	Vertex(const Vertex& other)
		: m_pos(other.m_pos)
		, m_payload( other.m_payloadSize > 0 ? new char[other.m_payloadSize] : 0)
		, m_payloadSize(other.m_payloadSize)
		, m_firstFace(other.m_firstFace)
		, m_boundary(other.m_boundary)
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
	bool m_boundary;
};

////////////////////////////////////////////////////////////////////////////////	

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

void TriMesh::AttemptCreateEdges(int faceIdx, bool &nonManifold, bool &wrongFacing)
{
	Face* face = m_faces[faceIdx];
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

				nonManifold = true;
			}
			else
			{
				edge = *iter;
				edge.m_faces[1] = faceIdx;
				Face* otherFace = m_faces[edge.m_faces[0]];
				// Connect the faces to each other on the corresponding edge.
				otherFace->m_faces[edge.m_firstFaceEdge] = faceIdx;
				face->m_faces[i] = edge.m_faces[0];

				// Before reinserting, check that the neighboring face doesn't have this edge in the same order.
				if(face->m_vertices[(i+1)%3] == otherFace->m_vertices[(edge.m_firstFaceEdge+1)%3])
				{
					wrongFacing = true;
				}
				m_edges.erase(edge);
				m_edges.insert(edge);
			}
		}
	}

}

int TriMesh::AddFace(int v0, int v1, int v2, int payloadSize)
{
	int verts[] = { v0, v1, v2 };
	int faceIdx = AllocateFace();
	Face* face = m_faces[faceIdx];

	// Fixup face Verts
	for(int i = 0; i < 3; ++i)
		face->m_vertices[i] = verts[i];
	
	bool nonManifold = false;
	bool wrongFacing = false;
	AttemptCreateEdges(faceIdx, nonManifold, wrongFacing);
	if(nonManifold)
	{
		DeleteFace(faceIdx);
		return -1;
	}
	if(wrongFacing)
	{
		DeleteFace(faceIdx);
		faceIdx = AllocateFace();
		face = m_faces[faceIdx];

		Swap(verts[1], verts[2]);
		// Fixup face Verts
		for(int i = 0; i < 3; ++i)
			face->m_vertices[i] = verts[i];

		AttemptCreateEdges(faceIdx, nonManifold, wrongFacing);
		if(nonManifold || wrongFacing)
		{
			DeleteFace(faceIdx);
			return -1;
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

	// Update vertex boundary info
	for(int i = 0; i < 3; ++i)
	{
		Vertex* vertex = m_vertices[verts[i]];
		const int firstFace = vertex->m_firstFace;
		int faceIdx = firstFace;
		do
		{
			faceIdx = GetNextFace(faceIdx, verts[i]);
		} while (faceIdx != -1 && faceIdx != firstFace);
		vertex->m_boundary = (faceIdx == -1);
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
	
int TriMesh::GetFaceFlags(int faceIndex) const
{
	const Face* face = m_faces[faceIndex];
	return face->m_flags;
}
	
int TriMesh::GetNeighborFaceIndex(int faceIndex, int neighborFace) const
{
	const Face* face = m_faces[faceIndex];
	for(int i = 0; i < 3; ++i)
	{
		if(face->m_faces[i] == neighborFace)
			return i;
	}
	return -1;
}

int TriMesh::GetVertexNumInFace(int faceIndex, int vertexIdx) const
{
	const Face* face = m_faces[faceIndex];
	for(int i = 0; i < 3; ++i)
	{
		if(face->m_vertices[i] == vertexIdx)
			return i;
	}
	return -1;
}
	
bool TriMesh::GetOneRing(int vertexIndex, int* ring, int max) const 
{
	int valency = GetVertexValency(vertexIndex);
	if(max < valency)
		return false;
	const Vertex* vertex = m_vertices[vertexIndex];
	const int firstFace = vertex->m_firstFace;
	int curFace = firstFace;
	if(vertex->m_boundary)
	{
		int curFace = firstFace;
		do {
			int prevFace = GetPrevFace(curFace, vertexIndex);
			if(prevFace == -1)
				break;
			curFace = prevFace;
		} while(true);
	}
	int ringIndex = 0;
	do {
		ASSERT(ringIndex < max);
		int nextVertexIdx = GetVertexNumInFace(vertexIndex, curFace);
		if(nextVertexIdx < 0) return false;
		nextVertexIdx = (nextVertexIdx + 1) % 3;
		const Face* face = m_faces[curFace];
		ring[ringIndex++] = face->m_vertices[nextVertexIdx];

		curFace = GetNextFace(curFace, vertexIndex);
	} while(curFace != -1 && curFace != firstFace);
	return true;	
}
	
int TriMesh::GetNextFace(int faceIdx, int vertexFrom) const
{
	int vnum = GetVertexNumInFace(faceIdx, vertexFrom);
	if(vnum < 0) return -1;
	const Face* face = m_faces[faceIdx];
	return face->m_faces[vnum];
}

int TriMesh::GetPrevFace(int faceIdx, int vertexFrom) const
{
	int vnum = GetVertexNumInFace(faceIdx, vertexFrom);
	if(vnum < 0) return -1;
	const Face* face = m_faces[faceIdx];
	return face->m_faces[(vnum + 2)%3];
}
	
int TriMesh::GetVertexValency(int vertexIdx) const
{
	const Vertex* vertex = m_vertices[vertexIdx];
	const int firstFace = vertex->m_firstFace;
	if(firstFace < 0)
		return 0;
	int count = 1;
	if(vertex->m_boundary)
	{
		int curFace = firstFace;
		while((curFace = GetNextFace(curFace, vertexIdx)) != -1)
			++count;
		curFace = firstFace;
		while((curFace = GetPrevFace(curFace, vertexIdx)) != -1)
			++count;
	}
	else
	{
		int curFace = firstFace;
		while((curFace = GetNextFace(curFace, vertexIdx)) != firstFace)
			++count;
	}
	return count;
}

void TriMesh::DeleteFace(int faceIndex)
{
	Face* face = m_faces[faceIndex];
	// Repair edge set so we can insert a new face where this one was.
	for(int i = 0; i < 3; ++i)
	{
		int v0 = face->m_vertices[i];
		int v1 = face->m_vertices[(i+1)%3];
	
		Edge edge(v0,v1);
		std::set<Edge>::iterator iter = m_edges.find(edge);
		if(iter != m_edges.end())
		{
			edge = *iter;
			if(faceIndex == edge.m_faces[0])
			{
				if(edge.m_faces[1] == -1)
				{
					m_edges.erase(edge);
				}
				else
				{
					edge.m_faces[0] = edge.m_faces[1];
					edge.m_faces[1] = -1;
					// Need to update m_firstFaceEdge... so which neighbor was faceIndex?
					int neighborIndex = GetNeighborFaceIndex(edge.m_faces[0], faceIndex);
					ASSERT(neighborIndex >= 0);
					edge.m_firstFaceEdge = neighborIndex;
					m_edges.erase(edge);
					m_edges.insert(edge);
				}
			}
			else if(faceIndex == edge.m_faces[1])
			{
				edge.m_faces[1] = -1;
				m_edges.erase(edge);
				m_edges.insert(edge);
			}
		}
	}

	// Update verts so they start on a valid face.
	for(int i = 0; i < 3; ++i)
	{
		int vertexIdx = face->m_vertices[i];
		Vertex* vertex = m_vertices[vertexIdx];

		if(vertex->m_firstFace == faceIndex)
		{
			int newFirstFace = GetNextFace(faceIndex, vertexIdx);
			if(newFirstFace < 0)
				newFirstFace = GetPrevFace(faceIndex, vertexIdx);
			// WARNING: this can create an isolated vertex that IS in a face, but cannot iterate
			// over the face. Example is:
			//  -----
			//  \   /
			//   \ /
			//    .
			//   / \			.	
			//  /   \			.
			//  -----
			vertex->m_firstFace = newFirstFace;
		}
	}
	
	// Update face neighbors and boundary edges
	for(int i = 0; i < 3; ++i)
	{
		if(face->m_faces[i] >= 0)
		{	
			Face* faceNeighbor = m_faces[ face->m_faces[i] ];
			int fnum = GetNeighborFaceIndex(face->m_faces[i], faceIndex);
			faceNeighbor->m_faces[fnum] = -1;
			m_vertices[faceNeighbor->m_vertices[fnum]]->m_boundary = true;
			m_vertices[faceNeighbor->m_vertices[(fnum+1)%3]]->m_boundary = true;
		}
	}

	delete face;
	m_faces[faceIndex] = 0;

	MoveFace(faceIndex, m_faces.size() - 1);
	m_faces.pop_back();
}

void TriMesh::MoveFace(int destIndex, int srcIndex)
{
	if(destIndex == srcIndex)
		return;

	ASSERT(m_faces[destIndex] == 0);
	Face* face = m_faces[srcIndex];
	m_faces[srcIndex] = 0;
	m_faces[destIndex] = face;

	// Update vertex first faces
	for(int i = 0; i <3 ; ++i)
	{
		Vertex* vertex = m_vertices[face->m_vertices[i]];
		vertex->m_firstFace = destIndex;
	}
	
	// Update neighbor references
	for(int i = 0; i < 3; ++i)
	{
		if(face->m_faces[i] >= 0)
		{
			Face* faceNeighbor = m_faces[ face->m_faces[i] ];
			int fnum = GetNeighborFaceIndex(face->m_faces[i], srcIndex);
			faceNeighbor->m_faces[fnum] = destIndex;
		}
	}
	
	// Update pending edges
	for(int i = 0; i < 3; ++i)
	{
		int v0 = face->m_vertices[i];
		int v1 = face->m_vertices[(i+1)%3];
	
		Edge edge(v0,v1);
		std::set<Edge>::iterator iter = m_edges.find(edge);
		if(iter != m_edges.end())
		{
			edge = *iter;
			if(srcIndex == edge.m_faces[0])
			{
				edge.m_faces[0] = destIndex;
			}
			else if(srcIndex == edge.m_faces[1])
			{
				edge.m_faces[1] = destIndex;
			}

			m_edges.erase(edge);
			m_edges.insert(edge);
		}
	}
}

int TriMesh::RemoveProblemTriangles()
{
	int count = 0;
	for(int i = m_faces.size() - 1; i >= 0; --i)
	{
		Face* face = m_faces[i];
		if(face->m_flags & FACE_NONMANIFOLD)
		{
			DeleteFace(i);
			++count;
		}
	}
	return count;
}

int TriMesh::FillHoles()
{

	return 0;
}

