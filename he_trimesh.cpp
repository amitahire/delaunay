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

int HETriMesh::AddVertex(const Vec3& pos)
{
	// assume pos is unique
	int idx = m_vertices.size();
	m_vertices.push_back(new Vertex(pos));
	return idx;
}

int HETriMesh::AddVertexUnique(const Vec3& pos)
{
	for(int i = 0, c = m_vertices.size(); i < c; ++i)
	{
		if(m_vertices[i]->m_pos == pos)
			return i;
	}
	return AddVertex(pos);
}

int HETriMesh::AddFace(int v0, int v1, int v2)
{
	if(v0 < 0 || v1 < 0 || v2 < 0)
		return -1;

	int faceIdx = m_faces.size();
	Face* face = new Face();
	m_faces.push_back(face);
	
	if((m_flags & OPT_TRYMATCH) && 
		(EdgeExistsWithFace(v0, v1) || EdgeExistsWithFace(v1, v2) || EdgeExistsWithFace(v2, v0)))
		Swap(v1, v2);

	face->m_vertices[0] = v0;
	face->m_vertices[1] = v1;
	face->m_vertices[2] = v2;

	face->m_edges[0] = AddEdge(faceIdx, v0, v1);
	face->m_edges[1] = AddEdge(faceIdx, v1, v2);
	face->m_edges[2] = AddEdge(faceIdx, v2, v0);

	if(face->m_edges[0] < 0 || face->m_edges[1] < 0 || face->m_edges[2] < 0)
	{
		m_faces.pop_back();
		delete face;
		return -1;
	}

	return faceIdx;
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
			printf("WARNING: vertex already has an edge %d from %d to %d.\n", vertEdge, 
				vertHalfEdge->m_from,
				vertHalfEdge->m_to);
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
				printf("WARNING: non-manifold mesh detected (edge %d has a dual)\n",
					vertEdge);
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
	Vertex* vertex = m_vertices[from];
	for(int i = 0, c = vertex->m_edges.size(); i < c; ++i)
	{
		HalfEdge* edge = m_edges[vertex->m_edges[i]];
		if(edge->m_to == to && edge->m_dual != -1)
			return true;
	}

	return false;
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

