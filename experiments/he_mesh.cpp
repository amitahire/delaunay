#include "common.hh"
#include "he_mesh.hh"
#include "math/math.hh"
#include "ptr.hh"
#include "debugdraw.hh"

// It's very likely that if any loops are 1024 big, that we have a problem.
static const int kSanityIterations = 1024;
////////////////////////////////////////////////////////////////////////////////
// HEMesh::HalfEdge
HEMesh::HalfEdge::HalfEdge() 
: m_dual(-1)
, m_face(-1)
, m_from(-1)
, m_to(-1)
, m_next(-1)
, m_prev(-1)
{}
		
////////////////////////////////////////////////////////////////////////////////
// HEMesh::Face
int HEMesh::Face::IndexOfVert(const std::vector<HalfEdge*>& edges, int vert) const
{
	if(m_edge == -1) return -1;
	const HalfEdge* first = edges[m_edge];
	const HalfEdge* cur = first;
	int idx = 0;
	do
	{
		VERIFY(idx < kSanityIterations);
		if(cur->m_from == vert)
			return idx;
		cur = edges[cur->m_next];
		++idx;
	}
	while(cur != first);
	return -1;
}
		
int HEMesh::Face::IndexOfEdge(const std::vector<HalfEdge*>& edges, int edge) const
{
	if(m_edge == -1) return -1;
	const HalfEdge* first = edges[m_edge];
	const HalfEdge* cur = first;
	const HalfEdge* const seekEdge = edges[edge];
	int idx = 0;
	do
	{
		VERIFY(idx < kSanityIterations);
		if(cur == seekEdge)
			return idx;
		cur = edges[cur->m_next];
		++idx;
	}
	while(cur != first);
	return -1;
}

////////////////////////////////////////////////////////////////////////////////
// HEMesh::Vertex
int HEMesh::Vertex::IndexOfEdge(const std::vector<HalfEdge*>& edges, int edge) const 
{
	if(m_firstEdge == -1) return -1;
	const HalfEdge* const seekEdge = edges[edge];
	const HalfEdge* first = edges[m_firstEdge];
	const HalfEdge* cur = first;
	int idx = 0;
	do
	{
		VERIFY(idx < kSanityIterations);
		if(cur == seekEdge)
			return idx;
		const HalfEdge* dual = edges[cur->m_dual];
		cur = edges[dual->m_next];
	}
	while(cur != first);
	return -1;
}

////////////////////////////////////////////////////////////////////////////////
// HEMesh
HEMesh::HEMesh(int flags)
	: m_flags(flags)
	, m_vertices()
	, m_faces()
	, m_edges()
{
}

HEMesh::~HEMesh()
{
	for(int i = 0, c = m_vertices.size(); i < c; ++i)
		delete m_vertices[i];
	for(int i = 0, c = m_faces.size(); i < c; ++i)
		delete m_faces[i];
	for(int i = 0, c = m_edges.size(); i < c; ++i)
		delete m_edges[i];
}

int HEMesh::AddVertex(const Vec3& pos, int payloadSize)
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

int HEMesh::AddVertexUnique(const Vec3& pos, int payloadSize)
{
	for(int i = 0, c = m_vertices.size(); i < c; ++i)
	{
		if(m_vertices[i]->m_pos == pos)
			return i;
	}
	return AddVertex(pos, payloadSize);
}

int HEMesh::AddFace(int* verts, int num, int payloadSize)
{
	for(int i = 0; i < num; ++i)
		if(verts[i] < 0)
			return -1;

	for(int i = 0; i < num; ++i)
	{
		if(EdgeExistsWithFace(verts[i], verts[(i+1)%num]))
		{
			if(m_flags & OPT_TRYMATCH)
			{
				for(int j = 0; j < num / 2; ++j)
					Swap(verts[j], verts[num - j - 1]);
			}
			else return -1;
		}
	}

	int faceIdx = m_faces.size();
	Face* face = new Face();
	m_faces.push_back(face);

	ScopedPtrAry<int>::Type edges(new int[num]);
	for(int i = 0; i < num; ++i)
		edges[i] = -1;

	int curEdge;
	for(curEdge = 0; curEdge < num; ++curEdge)
	{
		edges[curEdge] = AddEdge(faceIdx, verts[curEdge], verts[(curEdge+1)%num]);
		if(edges[curEdge] < 0) break;
	}

	if(curEdge != num)
	{
		for(int i = 0; i < curEdge; ++i)
		{
			int edgeIndex = edges[i];
			int dualIndex = m_edges[edgeIndex]->m_dual;
			m_edges[edgeIndex]->m_face = -1;

			if(!EdgeHasFaces(edges[i]))
			{	
				RemoveEdge(edges[i]);
				// these indexes change as we remove the edges.
				for(int j = i + 1; j < curEdge; ++j)
				{
					if(edges[j] == (int)m_edges.size() + 1)
					{
						edges[j] = edgeIndex;
					}
					else if(edges[j] == (int)m_edges.size())
					{
						edges[j] = dualIndex;
					}
				}
			}
		}
		m_faces.pop_back();
		delete face;
		DebugVerify();
		return -1;
	}

	face->m_edge = edges[0];
	printf("new face: %d: ", faceIdx);	
	for(int i = 0; i < num; ++i)
	{
		printf("%d -> %d(%d), ", m_edges[edges[i]]->m_from, edges[i], m_edges[edges[i]]->m_dual);
	}
	printf("\n");
	for(int i = 0; i < num; ++i)
	{
		printf("%d has next %d, prev %d\n", edges[i], m_edges[edges[i]]->m_next, m_edges[edges[i]]->m_prev);
		printf("%d has next %d, prev %d\n", m_edges[edges[i]]->m_dual, 
			m_edges[m_edges[edges[i]]->m_dual]->m_next, 
			m_edges[m_edges[edges[i]]->m_dual]->m_prev);
	}

	for(int i = 0; i < num; ++i)
	{
		ASSERT(m_edges[m_edges[edges[i]]->m_next]->m_face == faceIdx);
		ASSERT(m_edges[m_edges[edges[i]]->m_prev]->m_face == faceIdx);
	}

	if(payloadSize > 0)
	{
		face->m_payload = new char[payloadSize];
		if(m_flags & OPT_INITPAYLOAD)
			memset(face->m_payload, 0, payloadSize);
	}

	DebugVerify();
	return faceIdx;
}

void HEMesh::DeleteFace(int index)
{
	Face* face = m_faces[index];
	HalfEdge* firstEdge = m_edges[face->m_edge];

	int numEdges = 0;
	// Remove face references
	HalfEdge* cur = firstEdge;
	do
	{
		VERIFY(numEdges < kSanityIterations);
		cur->m_face = -1;
		cur = m_edges[cur->m_next];
		++numEdges;
	} while(cur != firstEdge);

	// Remove edges where the duals have no faces.
	ScopedPtrAry<int>::Type edgesToDelete = new int[numEdges];
	int numToDelete = 0;
	cur = firstEdge;
	int curIdx = face->m_edge;
	do
	{
		ASSERT(numToDelete < numEdges);
		if(m_edges[cur->m_dual]->m_face == -1)
			edgesToDelete[numToDelete++] = curIdx;
		curIdx = cur->m_next;
		cur = m_edges[cur->m_next];
	} while(cur != firstEdge);

	for(int i = 0; i < numToDelete; ++i)
	{
		if(!EdgeHasFaces(edgesToDelete[i]))
		{ 
			int edgeIndex = edgesToDelete[i];
			int dualIndex = m_edges[edgeIndex]->m_dual;
			RemoveEdge(edgesToDelete[i]);
			for(int j = i + 1; j < numToDelete; ++j)
			{
				if(edgesToDelete[j] == (int)m_edges.size() + 1)
				{
					edgesToDelete[j] = edgeIndex;
				}
				else if(edgesToDelete[j] == (int)m_edges.size())
				{
					edgesToDelete[j] = dualIndex;
				}
			}
		}
	}

	delete face;
	m_faces[index] = 0;
	if(!m_faces.empty()) MoveFace(index, m_faces.size() - 1);
	m_faces.pop_back();

	DebugVerify();
}

	
int HEMesh::AddEdge(int faceIdx, int start, int end)
{
	// If an edge exists that has a free face, use that. 
	int edgeIdx = FindEdge(start, end);
	int dualIdx = -1;
	if(edgeIdx != -1)
	{
		HalfEdge* halfEdge = m_edges[edgeIdx];
		if(halfEdge->m_face >= 0)
		{
			printf("Face %d already exists for edge from %d to %d\n", halfEdge->m_face, start, end);
			return -1;
		}
		halfEdge->m_face = faceIdx;
		dualIdx = halfEdge->m_dual;
	}
	else
	{
		edgeIdx = m_edges.size();
		HalfEdge* halfEdge = new HalfEdge();
		m_edges.push_back(halfEdge);

		halfEdge->m_from = start;
		halfEdge->m_to = end;
		halfEdge->m_face = faceIdx;

		dualIdx = m_edges.size();
		HalfEdge* dualEdge = new HalfEdge();
		m_edges.push_back(dualEdge);

		dualEdge->m_from = end;
		dualEdge->m_to = start;

		halfEdge->m_dual = dualIdx;
		dualEdge->m_dual = edgeIdx;

		int edgeInsert, dualInsert;
		FindEdgeInsert(start, edgeIdx, edgeInsert);
		FindEdgeInsert(end, dualIdx, dualInsert);

		int edgeFace = m_edges[edgeInsert]->m_face;
		int edgePrevFace = (edgeInsert == edgeIdx ? -1 : m_edges[m_edges[edgeInsert]->m_prev]->m_face);
		int dualEdgeFace = m_edges[dualInsert]->m_face;
		int dualEdgePrevFace = (dualInsert == dualIdx ? -1 : m_edges[m_edges[dualInsert]->m_prev]->m_face);

		if( (edgeFace >= 0 && edgeFace != faceIdx) ||
			(edgePrevFace >= 0 && edgePrevFace != faceIdx) ||
			(dualEdgeFace >= 0 && dualEdgeFace != faceIdx) ||
			(dualEdgePrevFace >= 0 && dualEdgePrevFace != faceIdx) )
		{
			printf("Cannot split face, would result in non-manifold mesh.\n"
				"edge %d inserted after %d splits face (%d || %d)\n"
				"edge %d inserted after %d splits face (%d || %d)\n",
				edgeIdx, edgeInsert,
				edgeFace, edgePrevFace,
				dualIdx, dualInsert,
				dualEdgeFace, dualEdgePrevFace);

			delete halfEdge;
			delete dualEdge;
			m_edges.pop_back();
			m_edges.pop_back();
			return -1;
		}
		else
		{
			InsertEdge(start, edgeIdx, edgeInsert);
			InsertEdge(end, dualIdx, dualInsert);

			ASSERT(halfEdge->m_next >= 0);
			ASSERT(halfEdge->m_prev >= 0);
			ASSERT(dualEdge->m_next >= 0);
			ASSERT(dualEdge->m_prev >= 0);
		}
	}

	return edgeIdx;
}

Vec3 HEMesh::GetWindingNormal(int edgeIdx) const
{
	const HalfEdge* startEdge = m_edges[edgeIdx];
	const HalfEdge* edge = startEdge;
	Vec3 avgNorm(0,0,0);
	int numFacesAdj = 0;
	do
	{
		if(edge->m_face >= 0)
		{
			const HalfEdge* nextEdge = m_edges[edge->m_next];
			ASSERT(nextEdge->m_dual != edgeIdx);

			Vec3 center = m_vertices[edge->m_to]->m_pos;
			Vec3 v0 = m_vertices[edge->m_from]->m_pos;
			Vec3 v1 = m_vertices[nextEdge->m_to]->m_pos;

			Vec3 n = normalize(cross(v1 - center, v0 - center));
			avgNorm += n;
			++numFacesAdj;
		}

		edge = m_edges[m_edges[edge->m_prev]->m_dual];

	} while (edge != startEdge);
	ASSERT(numFacesAdj > 0);
	if(numFacesAdj == 0) numFacesAdj = 1;
	avgNorm /= (float)numFacesAdj;
	return avgNorm;
}

void HEMesh::InsertEdge(int vertexIdx, int edge, int edgeInsert)
{
	printf("Inserting edge %d after %d\n", edge, edgeInsert);

	HalfEdge* halfEdge = m_edges[edge];
	HalfEdge* dualEdge = m_edges[halfEdge->m_dual];
	ASSERT(halfEdge->m_prev < 0);
	ASSERT(dualEdge->m_next < 0);
	Vertex* vertex = m_vertices[vertexIdx];
	if(vertex->m_firstEdge < 0)
	{
		vertex->m_firstEdge = edge;
		halfEdge->m_prev = halfEdge->m_dual;	
		dualEdge->m_next = edge;
		return;
	}

	ASSERT(edge != edgeInsert);

	int edgeNext = edgeInsert;
	int edgePrev = m_edges[edgeNext]->m_prev;

	halfEdge->m_prev = edgePrev;
	dualEdge->m_next = edgeNext;

	m_edges[edgeNext]->m_prev = halfEdge->m_dual;
	m_edges[edgePrev]->m_next = edge;

	if(vertex->m_firstEdge < 0)
		vertex->m_firstEdge	= edge;

	{
		const HalfEdge* nextEdge = m_edges[edgeNext];
		const HalfEdge* prevEdge = m_edges[edgePrev];

		DebugDrawLine(m_vertices[nextEdge->m_from]->m_pos,
			m_vertices[nextEdge->m_to]->m_pos, 1.f, 0.f, 0.f, 1.f);

		DebugDrawLine(m_vertices[prevEdge->m_from]->m_pos,
			m_vertices[prevEdge->m_to]->m_pos, 0.f, 1.f, 0.f, 1.f);
		
		DebugDrawLine(m_vertices[halfEdge->m_from]->m_pos,
			m_vertices[halfEdge->m_to]->m_pos, 1.f, 1.f, 0.f, 1.f);
	}
}

void HEMesh::FindEdgeInsert(int vertexIdx, int edge, int &insertAfter) const
{
	const HalfEdge* halfEdge = m_edges[edge];
	Vertex* vertex = m_vertices[vertexIdx];

	// Sort counter clockwise around 'outside' direction of this face
	if(vertex->m_firstEdge >= 0)
	{
		const int firstEdge = vertex->m_firstEdge;
		const int firstEdgeDual = m_edges[firstEdge]->m_dual;
		if( m_edges[firstEdge]->m_prev != firstEdgeDual)
		{
			const HalfEdge* firstEdgeData = m_edges[vertex->m_firstEdge];
			Vec3 toFirst = normalize(m_vertices[ firstEdgeData->m_to ]->m_pos - vertex->m_pos);
			Vec3 upVec = GetWindingNormal(vertex->m_firstEdge);
			Vec3 perpVec = normalize(cross(upVec, toFirst));

			// Sort as if toFirst is 'x' and perpVec is 'y' for the purposes of atan2.

			Vec3 toNew = normalize(ProjectN(m_vertices[ halfEdge->m_to ]->m_pos - vertex->m_pos, upVec));
			float angleNew = AngleWrap(atan2(dot(perpVec, toNew), dot(toFirst, toNew)));
			printf("angle for edge (%d) = %f, (%f/%f)\n", edge, angleNew, dot(perpVec, toNew), dot(toFirst, toNew));

			int iterCount = 0;
			int curEdge = firstEdge;
			const HalfEdge *curEdgeData = m_edges[curEdge];
			int nextEdge = m_edges[curEdgeData->m_prev]->m_dual;
			while(nextEdge != firstEdge)
			{
				VERIFY(iterCount < kSanityIterations);

				const HalfEdge* nextEdgeData = m_edges[nextEdge];

				Vec3 toNext = normalize(ProjectN(m_vertices[nextEdgeData->m_to]->m_pos - vertex->m_pos, upVec));
				float angleNext = AngleWrap(atan2(dot(perpVec, toNext), dot(toFirst, toNext)));
				printf("%d: %f > %f = %d\n", nextEdge, angleNext, angleNew, int(angleNext > angleNew));
				if(angleNext > angleNew)
					break;
			
				curEdge = nextEdge;
				curEdgeData = m_edges[curEdge];
				nextEdge = m_edges[curEdgeData->m_prev]->m_dual;

				++iterCount;
			}

			insertAfter = curEdge;
			printf("insertAfter = %d\n", insertAfter);
		}
		else
			insertAfter = firstEdge;
	}
	else
	{
		insertAfter = edge;
	}
}

bool HEMesh::EdgeExistsWithFace(int from, int to) const
{
	int edgeIdx = FindEdge(from, to);
	if(edgeIdx >= 0)
	{
		HalfEdge* edge = m_edges[edgeIdx];
		if(edge->m_face != -1)
			return true;
	}
	return false;
}
	
int HEMesh::FindEdge(int from, int to) const
{
	Vertex* vertex = m_vertices[from];
	if(vertex->m_firstEdge == -1)
		return -1;
	const HalfEdge* firstEdge = m_edges[vertex->m_firstEdge];
	const HalfEdge* cur = firstEdge;
	int edgeIdx = vertex->m_firstEdge;
	int iterCount = 0;
	do
	{
		VERIFY(iterCount < kSanityIterations);
		ASSERT(cur->m_from == from);
		if(cur->m_to == to)
		{
			return edgeIdx;
		}
		edgeIdx = m_edges[cur->m_dual]->m_next;
		cur = m_edges[edgeIdx];
		++iterCount;
	} while (cur != firstEdge);
	return -1;
}

const Vec3& HEMesh::GetVertexPos(int vertexIdx) const
{
	Vertex* vertex = m_vertices[vertexIdx];
	return vertex->m_pos;
}

int HEMesh::GetFace(int faceIdx, int * indices, int maxIndices) const
{
	Face* face = m_faces[faceIdx];
	if(face->m_edge < 0) return 0;
	if(maxIndices < 1) return 0;
	const HalfEdge* firstEdge = m_edges[face->m_edge];
	const HalfEdge* cur = firstEdge;
	int idx = 0;
	do
	{
		if(idx == maxIndices)
			return 0;
		indices[idx++] = cur->m_from;
		cur = m_edges[cur->m_next];
	} while (cur != firstEdge);
	return idx;
}
	
char * HEMesh::GetVertexData(int index) 
{
	Vertex* vertex = m_vertices[index];
	return vertex->m_payload;
}

const char * HEMesh::GetVertexData(int index) const 
{
	const Vertex* vertex = m_vertices[index];
	return vertex->m_payload;
}

const char * HEMesh::GetFaceData(int index) const 
{
	const Face* face = m_faces[index];
	return face->m_payload;
}

char * HEMesh::GetFaceData(int index) 
{
	Face* face = m_faces[index];
	return face->m_payload;
}
	
bool HEMesh::EdgeHasFaces(int edgeIndex) const
{
	const HalfEdge* edge = m_edges[edgeIndex];
	int dualIndex = edge->m_dual;
	const HalfEdge* dual = m_edges[dualIndex];

	return edge->m_face >= 0 || dual->m_face >= 0;
}

void HEMesh::RemoveEdge(int edgeIndex)
{
	ASSERT(!EdgeHasFaces(edgeIndex));
	HalfEdge* edge = m_edges[edgeIndex];
	int dualIndex = edge->m_dual;
	HalfEdge* dual = m_edges[dualIndex];

	HalfEdge* edgeNext = m_edges[edge->m_next];
	HalfEdge* edgePrev = m_edges[edge->m_prev];
	HalfEdge* dualNext = m_edges[dual->m_next];
	HalfEdge* dualPrev = m_edges[dual->m_prev];
	Vertex* vertFrom = m_vertices[edge->m_from];
	Vertex* vertTo = m_vertices[edge->m_to];

	if(edgeNext != dual)
	{
		edgeNext->m_prev = dual->m_prev;
		dualPrev->m_next = edge->m_next;

		if(vertTo->m_firstEdge == dualIndex)
			vertTo->m_firstEdge = dualPrev->m_dual;
	}
	else
	{
		vertTo->m_firstEdge = -1;
	}

	if(edgePrev != dual)
	{
		dualNext->m_prev = edge->m_prev;
		edgePrev->m_next = dual->m_next;

		if(vertFrom->m_firstEdge == edgeIndex)
			vertFrom->m_firstEdge = edgePrev->m_dual;
	}
	else
	{
		vertFrom->m_firstEdge = -1;
	}

	edge->m_dual = dual->m_dual = -1;
	edge->m_next = -1;
	dual->m_prev = -1;
	edge->m_prev = -1;
	dual->m_next = -1;

	delete edge;
	ASSERT(edge == m_edges[edgeIndex]);
	m_edges[edgeIndex] = 0;
	if(!m_edges.empty()) 
	{
		MoveEdge(edgeIndex, m_edges.size() - 1);
		if(dualIndex == (int)m_edges.size() - 1)
			dualIndex = edgeIndex;
		m_edges.pop_back();
	}
	
	delete dual;
	ASSERT(dual == m_edges[dualIndex]);
	m_edges[dualIndex] = 0;
	if(!m_edges.empty())
	{
		MoveEdge(dualIndex, m_edges.size() - 1);
		m_edges.pop_back();
	}
}

void HEMesh::MoveEdge(int newIndex, int oldIndex)
{
	if(newIndex == oldIndex)
		return;

	HalfEdge* edge = m_edges[oldIndex];
	m_edges[newIndex] = edge;
	m_edges[oldIndex] = 0;

	// patch indexes referring to this edge.
	if(edge->m_face >= 0)
	{
		Face* face = m_faces[edge->m_face];
		if(face->m_edge == oldIndex)
			face->m_edge = newIndex;
	}

	if(edge->m_dual >= 0)
	{
		HalfEdge* dual = m_edges[edge->m_dual];
		dual->m_dual = newIndex;
	}

	Vertex* vertex = m_vertices[edge->m_from];
	if(vertex->m_firstEdge == oldIndex)
		vertex->m_firstEdge = newIndex;

	if(edge->m_prev >= 0)
	{	
		HalfEdge* prevEdge = m_edges[edge->m_prev];
		prevEdge->m_next = newIndex;
	}

	if(edge->m_next >= 0)
	{
		HalfEdge* nextEdge = m_edges[edge->m_next];
		nextEdge->m_prev = newIndex;
	}
}

void HEMesh::MoveFace(int newIndex, int oldIndex)
{	
	if(newIndex == oldIndex)
		return;
	
	Face* face = m_faces[oldIndex];
	m_faces[newIndex] = face;
	m_faces[oldIndex] = 0;

	HalfEdge* firstEdge = m_edges[face->m_edge];
	HalfEdge* cur = firstEdge;
	
	int iterCount = 0;
	do
	{
		VERIFY(iterCount < kSanityIterations);
		cur->m_face = newIndex;
		cur = m_edges[cur->m_next];
		++iterCount;
	} while(cur != firstEdge) ;
}	

void HEMesh::DebugVerify()
{
	for(int i = 0, c = m_faces.size(); i < c; ++i)
	{
		Face* face = m_faces[i];
		if(!face) continue;
		VERIFY(face->m_edge >= 0);
		HalfEdge* firstEdge = m_edges[face->m_edge];
		HalfEdge* cur = firstEdge;

		int iterCount = 0;
		do
		{
			VERIFY(iterCount < kSanityIterations);
			VERIFY(cur->m_face == i);
			cur = m_edges[cur->m_next];
			++iterCount;
		} while(cur != firstEdge) ;
	}

	for(int i = 0, c = m_edges.size(); i < c; ++i)
	{
		HalfEdge* edge = m_edges[i];
		if(!edge) continue;
		
		VERIFY(m_edges[edge->m_dual]->m_dual == i);
		Vertex* vtx = m_vertices[edge->m_from];
		VERIFY(-1 != vtx->IndexOfEdge(m_edges, i));
	}
}

