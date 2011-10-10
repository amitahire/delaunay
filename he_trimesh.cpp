#include "common.hh"
#include "he_trimesh.hh"
#include "math/math.hh"

HETriMesh::HETriMesh(int flags)
	: m_flags(flags)
{
}

HETriMesh::~HETriMesh()
{
	for(int i = 0, c = m_vertices.size(); i < c; ++i)
		delete m_vertices[i];
	for(int i = 0, c = m_faces.size(); i < c; ++i)
		delete m_faces[i];
	for(int i = 0, c = m_edges.size(); i < c; ++i)
		delete m_edges[i];
}

int HETriMesh::AddVertex(const Vec3& pos, int payloadSize)
{
	// assume pos is unique
	int idx = m_vertices.size();
	char * payload = 0;
	if(payloadSize > 0)
	{
		payload = new char[payloadSize];
		if(m_flags & OPT_INITPAYLOAD)
			memset(payload, 0, payloadSize);
	}
	m_vertices.push_back(new Vertex(pos, payload));
	return idx;
}

int HETriMesh::AddVertexUnique(const Vec3& pos, int payloadSize)
{
	for(int i = 0, c = m_vertices.size(); i < c; ++i)
	{
		if(m_vertices[i]->m_pos == pos)
			return i;
	}
	return AddVertex(pos, payloadSize);
}

int HETriMesh::AddFace(int v0, int v1, int v2, int payloadSize)
{
	if(v0 < 0 || v1 < 0 || v2 < 0)
		return -1;

	if( (m_flags & OPT_REPLACE_SMALLER_ON_CONFLICT) || (m_flags & OPT_IGNORE_SMALL) )
	{
		const Vec3& pt0 = m_vertices[v0]->m_pos;
		const Vec3& pt1 = m_vertices[v1]->m_pos;
		const Vec3& pt2 = m_vertices[v2]->m_pos;
		float parallelogramAreaSqNew = magnitude_squared(cross(pt1 - pt0, pt2 - pt0));
		
		if(m_flags & OPT_IGNORE_SMALL)
		{
			if(parallelogramAreaSqNew < EPSILON)
				return -1;
		}

		if(m_flags & OPT_REPLACE_SMALLER_ON_CONFLICT) 
		{
			int verts[3];
			verts[0] = v0;
			verts[1] = v1;
			verts[2] = v2;
			for(int i = 0; i < 3; ++i)
			{
				int edgeIndex = FindEdge(verts[i], verts[(i+1)%3]);
				if(edgeIndex >= 0)
				{
					HalfEdge *halfEdge = m_edges[edgeIndex];
					if(halfEdge->m_face >= 0)
					{
						Face* existingFace = m_faces[halfEdge->m_face];
						const Vec3& npt0 = m_vertices[existingFace->m_vertices[0]]->m_pos;
						const Vec3& npt1 = m_vertices[existingFace->m_vertices[1]]->m_pos;
						const Vec3& npt2 = m_vertices[existingFace->m_vertices[2]]->m_pos;
						float parallelogramAreaSqExisting = magnitude_squared(cross(npt1 - npt0, npt2 - npt0));

						if(parallelogramAreaSqExisting < parallelogramAreaSqNew) 
							DeleteFace(halfEdge->m_face);
					}
				}
			}
		}
	}
	
	if((EdgeExistsWithFace(v0, v1) || EdgeExistsWithFace(v1, v2) || EdgeExistsWithFace(v2, v0)))
	{
		if(m_flags & OPT_TRYMATCH)
			Swap(v1, v2);
		else return -1;
	}

	int faceIdx = m_faces.size();
	Face* face = new Face();
	m_faces.push_back(face);

	face->m_vertices[0] = v0;
	face->m_vertices[1] = v1;
	face->m_vertices[2] = v2;

	face->m_edges[0] = AddEdge(faceIdx, v0, v1);
	face->m_edges[1] = AddEdge(faceIdx, v1, v2);
	face->m_edges[2] = AddEdge(faceIdx, v2, v0);

	if(face->m_edges[0] < 0 || face->m_edges[1] < 0 || face->m_edges[2] < 0)
	{
		for(int i = 0; i < 3; ++i)
			if(face->m_edges[i] >= 0)
				RemoveEdge(face->m_edges[i]);
		m_faces.pop_back();
		delete face;
		return -1;
	}

	if(payloadSize > 0)
	{
		face->m_payload = new char[payloadSize];
		if(m_flags & OPT_INITPAYLOAD)
			memset(face->m_payload, 0, payloadSize);
	}

	//DebugVerify();
	return faceIdx;
}

bool HETriMesh::Vertex::RemoveEdge(int edgeIdx)
{
	for(int i = 0, c = m_edges.size(); i < c; ++i)
	{
		if(m_edges[i] == edgeIdx)
		{
			for(int j = i; j < c - 1; ++j)
			{
				m_edges[j] = m_edges[j+1];
			}
			m_edges.pop_back();
			return true;
		}
	}
	return false;
}

void HETriMesh::DeleteFace(int index)
{
	Face* face = m_faces[index];
	for(int i = 0; i < 3; ++i)
	{
		int oldEdge = face->m_edges[i];
		face->m_edges[i] = -1;

		HalfEdge* edge = m_edges[oldEdge];
		edge->m_face = -1;

		int oldVert = edge->m_from;
		if(edge->m_dual >= 0)
		{
			HalfEdge* dual = m_edges[edge->m_dual];
			dual->m_dual = -1;
		}

		Vertex* vertex = m_vertices[oldVert];
		VERIFY(vertex->RemoveEdge(oldEdge));

		delete edge;
		if(!m_edges.empty()) MoveEdge(oldEdge, m_edges.size() - 1);
		m_edges.pop_back();
	}

	delete face;
	if(!m_faces.empty()) MoveFace(index, m_faces.size() - 1);
	m_faces.pop_back();

	//DebugVerify();
}

	
int HETriMesh::AddEdge(int face, int start, int end)
{
	int edgeIdx = m_edges.size();
	HalfEdge* halfEdge = new HalfEdge();
	m_edges.push_back(halfEdge);

	halfEdge->m_face = face;
	halfEdge->m_from = start;
	halfEdge->m_to = end;

	if(!ConnectEdge(start, edgeIdx))
	{
		m_edges.pop_back();
		delete halfEdge;
		return -1;
	}

	return edgeIdx;
}
	
bool HETriMesh::ConnectEdge(int vertexIdx, int edge)
{
	HalfEdge* halfEdge = m_edges[edge];
	Vertex* vertex = m_vertices[vertexIdx];

	// Sanity check. 
	for(int i = 0, c = vertex->m_edges.size(); i < c; ++i)
	{
		int vertEdge = vertex->m_edges[i];
		HalfEdge* vertHalfEdge = m_edges[vertEdge];
		if(vertHalfEdge->m_from == halfEdge->m_from &&
			vertHalfEdge->m_to == halfEdge->m_to)
		{
			//printf("WARNING: vertex already has an edge %d from %d to %d.\n", vertEdge, 
			//	vertHalfEdge->m_from,
			//	vertHalfEdge->m_to);
			return false;
		}
	}

	// Find the dual
	Vertex* destVertex = m_vertices[halfEdge->m_to];
	for(int i = 0, c = destVertex->m_edges.size(); i < c; ++i)
	{
		int vertEdge = destVertex->m_edges[i];
		HalfEdge* vertHalfEdge = m_edges[vertEdge];

		if(vertHalfEdge->m_from == halfEdge->m_to &&
			vertHalfEdge->m_to == halfEdge->m_from)
		{
			if(vertHalfEdge->m_dual != -1)
			{
			//	printf("WARNING: non-manifold mesh detected (edge %d has a dual)\n",
			//		vertEdge);
				return false;
			}

			vertHalfEdge->m_dual = edge;
			halfEdge->m_dual = vertEdge;
			break;
		}
	}

	vertex->m_edges.push_back(edge);
	// Sort counter clockwise around 'outside' direction of this face
	if(vertex->m_edges.size() > 1)
	{
		Vec3 toFirst = normalize(m_vertices[ m_edges[vertex->m_edges[0]]->m_to ]->m_pos - vertex->m_pos);

		int faceIdx = halfEdge->m_face;
		Face* face = m_faces[faceIdx];

		int vertNum = face->IndexOfVert(vertexIdx);
		int secondVert = (vertNum + 2) % 3;

		Vec3 windingVec = normalize(m_vertices[ secondVert ]->m_pos - vertex->m_pos);
		Vec3 upVec = cross(toFirst, windingVec);
		Vec3 perpVec = normalize(cross(upVec, toFirst));

		// Sort as if toFirst is 'x' and perpVec is 'y' for the purposes of atan2.
		Vec3 toNew = normalize(m_vertices[ halfEdge->m_to ]->m_pos - vertex->m_pos);
		float angleNew = AngleWrap(atan2(dot(perpVec, toNew), dot(toFirst, toNew)));
		int indexOfNew = vertex->m_edges.size() - 1;

		for(int i = vertex->m_edges.size() - 2; i >= 0; --i)
		{
			Vec3 toNext = normalize(m_vertices[ m_edges[vertex->m_edges[i]]->m_to ]->m_pos - vertex->m_pos);
			float angleNext = AngleWrap(atan2(dot(perpVec, toNext), dot(toFirst, toNext)));
			if(angleNext > angleNew)
			{
				Swap(vertex->m_edges[indexOfNew], vertex->m_edges[i]);
				indexOfNew = i;
			}
		}
	}

	return true;
}

bool HETriMesh::EdgeExistsWithFace(int from, int to) const
{
	int edgeIdx = FindEdge(from, to);
	if(edgeIdx >= 0)
	{
		HalfEdge* edge = m_edges[edgeIdx];
		if(edge->m_dual != -1)
			return true;
	}
	return false;
}
	
int HETriMesh::FindEdge(int from, int to) const
{
	Vertex* vertex = m_vertices[from];
	for(int i = 0, c = vertex->m_edges.size(); i < c; ++i)
	{
		HalfEdge* edge = m_edges[vertex->m_edges[i]];
		if(edge->m_to == to)
			return vertex->m_edges[i];
	}

	return -1;
}

const Vec3& HETriMesh::GetVertexPos(int vertexIdx) const
{
	Vertex* vertex = m_vertices[vertexIdx];
	return vertex->m_pos;
}

void HETriMesh::GetFace(int faceIdx, int (&indices)[3]) const
{
	Face* face = m_faces[faceIdx];
	indices[0] = face->m_vertices[0];
	indices[1] = face->m_vertices[1];
	indices[2] = face->m_vertices[2];
}
	
char * HETriMesh::GetVertexData(int index) 
{
	Vertex* vertex = m_vertices[index];
	return vertex->m_payload;
}

const char * HETriMesh::GetVertexData(int index) const 
{
	const Vertex* vertex = m_vertices[index];
	return vertex->m_payload;
}

const char * HETriMesh::GetFaceData(int index) const 
{
	const Face* face = m_faces[index];
	return face->m_payload;
}

char * HETriMesh::GetFaceData(int index) 
{
	Face* face = m_faces[index];
	return face->m_payload;
}
	
void HETriMesh::RemoveEdge(int edgeIndex)
{
	HalfEdge* edge = m_edges[edgeIndex];
	Vertex* vtx = m_vertices[edge->m_from];
	vtx->RemoveEdge(edgeIndex);

	if(edge->m_dual >= 0)
	{
		HalfEdge* dual = m_edges[edge->m_dual];
		dual->m_dual = -1;
	}
	
	delete edge;
	if(!m_edges.empty()) MoveEdge(edgeIndex, m_edges.size() - 1);
	m_edges.pop_back();
}

void HETriMesh::MoveEdge(int newIndex, int oldIndex)
{
	if(newIndex == oldIndex)
	{
		m_edges[oldIndex] = 0;
		return;
	}
	HalfEdge* edge = m_edges[oldIndex];
	m_edges[newIndex] = edge;
	m_edges[oldIndex] = 0;

	// patch indexes referring to this edge.
	Face* face = m_faces[edge->m_face];
	int i = face->IndexOfEdge(oldIndex);
	VERIFY(i != -1);
	face->m_edges[i] = newIndex;

	ASSERT(edge->m_face == m_edges[face->m_edges[i]]->m_face);

	if(edge->m_dual >= 0)
	{
		HalfEdge* dual = m_edges[edge->m_dual];
		dual->m_dual = newIndex;
	}

	Vertex* vertex = m_vertices[edge->m_from];
	i = vertex->IndexOfEdge(oldIndex);
	VERIFY(i != -1);
	vertex->m_edges[i] = newIndex;
}

void HETriMesh::MoveFace(int newIndex, int oldIndex)
{	
	if(newIndex == oldIndex)
	{
		m_faces[oldIndex] = 0;
		return;
	}
	Face* face = m_faces[oldIndex];
	m_faces[newIndex] = face;
	m_faces[oldIndex] = 0;

	for(int i = 0; i < 3; ++i)
	{
		HalfEdge *edge = m_edges[face->m_edges[i]];
		ASSERT(-1 != face->IndexOfVert(edge->m_from));
		ASSERT(-1 != face->IndexOfVert(edge->m_to));
		edge->m_face = newIndex;
	}
}	

void HETriMesh::DebugVerify()
{
	for(int i = 0, c = m_faces.size(); i < c; ++i)
	{
		Face * face = m_faces[i];
		if(!face) continue;
		for(int j = 0; j < 3; ++j)
		{
			VERIFY(m_edges[face->m_edges[j]]->m_face == i);
			int edgeIdx = FindEdge(face->m_vertices[j], face->m_vertices[ (j+1)%3 ]);
			VERIFY(-1 != edgeIdx);
			VERIFY(face->IndexOfEdge(edgeIdx) != -1);
		}
	}

	for(int i = 0, c = m_edges.size(); i < c; ++i)
	{
		HalfEdge * edge = m_edges[i];
		if(!edge) continue;
		if(edge->m_dual >= 0)
			VERIFY(m_edges[edge->m_dual]->m_dual == i);
		Vertex * vtx = m_vertices[edge->m_from];

		VERIFY(-1 != vtx->IndexOfEdge(i));
	}
}

