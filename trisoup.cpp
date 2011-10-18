#include "common.hh"
#include "trisoup.hh"
#include <cstring>

////////////////////////////////////////////////////////////////////////////////
TriSoup::Vertex::Vertex()
	: m_pos()
	, m_payload()
	, m_payloadSize(0)
{
}

TriSoup::Vertex::Vertex(const Vertex& other)
	: m_pos(other.m_pos)
	, m_payload(0)
	, m_payloadSize(0)
{
	CopyData(other);
}

TriSoup::Vertex& TriSoup::Vertex::operator=(const Vertex& other)
{
	if(this != &other)
	{
		m_pos = other.m_pos;
		CopyData(other);
	}
	return *this;
}

void TriSoup::Vertex::CopyData(const Vertex& other)
{
	m_payload = 0;
	m_payloadSize = other.m_payloadSize;
	if(m_payloadSize > 0)
	{
		m_payload = new char[m_payloadSize];
		memcpy(m_payload, other.m_payload, m_payloadSize);
	}
}

////////////////////////////////////////////////////////////////////////////////
TriSoup::Face::Face()
	: m_payload()
	, m_payloadSize(0)
{
	for(int i = 0; i < 3; ++i)
		m_vertices[i] = -1;
}

TriSoup::Face::Face(const Face& other)
	: m_payload()
	, m_payloadSize(0)
{
	for(int i = 0; i < 3; ++i)
		m_vertices[i] = other.m_vertices[i];
	CopyData(other);
}

TriSoup::Face& TriSoup::Face::operator=(const Face& other)
{
	if(this != &other)
	{
		for(int i = 0; i < 3; ++i)
			m_vertices[i] = other.m_vertices[i];
		CopyData(other);
	}
	return *this;	
}

void TriSoup::Face::CopyData(const Face& other)
{
	m_payload = 0;
	m_payloadSize = other.m_payloadSize;
	if(m_payloadSize > 0)
	{
		m_payload = new char[m_payloadSize];
		memcpy(m_payload, other.m_payload, m_payloadSize);
	}
}

////////////////////////////////////////////////////////////////////////////////
TriSoup::TriSoup()
	: m_faces()
	, m_vertices()
{
}
	
void TriSoup::Clear()
{
	for(int i = 0, c = m_faces.size(); i < c; ++i)
		delete m_faces[i];
	for(int i = 0, c = m_faces.size(); i < c; ++i)
		delete m_vertices[i];

	m_faces.clear();
	m_vertices.clear();
}

TriSoup::~TriSoup()
{
	Clear();
}

TriSoup::TriSoup(const TriSoup& other)
	: m_faces(other.m_faces.size())
	, m_vertices(other.m_vertices.size())
{
	for(int i = 0, c = m_faces.size(); i < c; ++i)
		m_faces[i] = new Face(*other.m_faces[i]);
	for(int i = 0, c = m_vertices.size(); i <c; ++i)
		m_vertices[i] = new Vertex(*other.m_vertices[i]);
}

TriSoup& TriSoup::operator=(const TriSoup& other)
{
	if(this != &other)
	{
		Clear();
		for(int i = 0, c = m_faces.size(); i < c; ++i)
			m_faces[i] = new Face(*other.m_faces[i]);
		for(int i = 0, c = m_vertices.size(); i <c; ++i)
			m_vertices[i] = new Vertex(*other.m_vertices[i]);
	}
	return *this;
}

int TriSoup::AddVertex(const Vec3& pos, int payloadSize)
{
	int idx = m_vertices.size();
	m_vertices.push_back(new Vertex());
	Vertex* vertex = m_vertices.back();
	vertex->m_pos = pos;
	if(payloadSize > 0)
	{
		vertex->m_payloadSize = payloadSize;
		vertex->m_payload = new char[payloadSize];
		memset(vertex->m_payload, 0, payloadSize);
	}
	return idx;
}

int TriSoup::AddVertexUnique(const Vec3& pos, int payloadSize)
{
	for(int i = 0, c = m_vertices.size(); i < c; ++i)
	{
		if(m_vertices[i]->m_pos == pos)
			return i;
	}
	return AddVertex(pos, payloadSize);
}

int TriSoup::AddFace(int v0, int v1, int v2, int payloadSize)
{
	int idx = m_faces.size();
	m_faces.push_back(new Face());
	Face* face = m_faces.back();
	face->m_vertices[0] = v0;
	face->m_vertices[1] = v1;
	face->m_vertices[2] = v2;
	
	if(payloadSize > 0)
	{
		face->m_payloadSize = payloadSize;
		face->m_payload = new char[payloadSize];
		memset(face->m_payload, 0, payloadSize);
	}

	return idx;
}

void TriSoup::DeleteFace(int index)
{
	delete m_faces[index];
	m_faces[index] = 0;
	MoveFace(index, m_faces.size() - 1);
	m_faces.pop_back();
}

void TriSoup::MoveFace(int destIdx, int srcIdx)
{
	ASSERT(m_faces[destIdx] == 0);
	m_faces[destIdx] = m_faces[srcIdx];
	m_faces[srcIdx] = 0;
}

char * TriSoup::GetVertexData(int index) 
{
	return m_vertices[index]->m_payload;
}

const char * TriSoup::GetVertexData(int index) const 
{
	return m_vertices[index]->m_payload;
}

char * TriSoup::GetFaceData(int index) 
{
	return m_faces[index]->m_payload;
}

const char * TriSoup::GetFaceData(int index) const 
{
	return m_faces[index]->m_payload;
}

const Vec3& TriSoup::GetVertexPos(int index) const
{
	Vertex* vertex = m_vertices[index];
	return vertex->m_pos;
}

void TriSoup::GetFace(int index, int (&indices)[3]) const
{
	Face* face = m_faces[index];
	for(int i = 0; i < 3; ++i)
		indices[i] = face->m_vertices[i];
}

