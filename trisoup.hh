#pragma once

#include "math/vec3.hh"
#include "ptr.hh"
#include <vector>

////////////////////////////////////////////////////////////////////////////////
// TriSoup
//  - triangle mesh with no topology info or guarantees. Just a bunch of tris.
class TriSoup
{
public:
	////////////////////////////////////////////////////////////////////////////////	
	TriSoup();
	~TriSoup();
	TriSoup(const TriSoup& other);
	TriSoup& operator=(const TriSoup& other);

	// Creation functions
	int AddVertex(const Vec3& pos, int payloadSize = 0);
	int AddVertexUnique(const Vec3& pos, int payloadSize = 0);
	int AddFace(int v0, int v1, int v2, int payloadSize = 0);

	// Modification functions
	void Clear();
	void DeleteFace(int index);

	// Accessor functions
	int NumFaces() const { return m_faces.size(); }
	int NumVertices() const { return m_vertices.size(); }

	char * GetVertexData(int index) ;
	const char * GetVertexData(int index) const ;
	char * GetFaceData(int index) ;
	const char * GetFaceData(int index) const ;

	const Vec3& GetVertexPos(int index) const;
	void GetFace(int index, int (&indices)[3]) const;

private:
	////////////////////////////////////////////////////////////////////////////////	
	struct Face;
	struct Vertex;

	////////////////////////////////////////////////////////////////////////////////	
	void MoveFace(int destIdx, int srcIdx);
	
	////////////////////////////////////////////////////////////////////////////////	
	std::vector<Face*> m_faces;
	std::vector<Vertex*> m_vertices;
};

