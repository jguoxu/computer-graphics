/**
 * @file project.hpp
 * @brief Geometry project
 *
 * @author H. Q. Bovik (hqbovik)
 * @bug Unimplemented
 */

#ifndef _462_GEOMETRY_PROJECT_HPP_
#define _462_GEOMETRY_PROJECT_HPP_

#include "math/camera.hpp"
#include "glew/GL/glew.h"

/*
   A namespace declaration. All project files use this namespace.
   Add this declaration (and its closing) to all source/headers you create.
   Note that all #includes should be BEFORE the namespace declaration.
 */
namespace _462 {

struct Triangle
{
    // index into vertex list of the 3 vertices of this triangle
    unsigned int vertices[3];
};

struct Vertex
{
    // the position of the vertex
    Vector3 position;
    // the normal of the vertex
    Vector3 normal;
    // the texture coordinate of the vertex
    Vector2 texture_coord;
};

struct MeshData
{
    // array of vertices
    Vertex* vertices;
    // size of vertex array
    size_t num_vertices;

    // array of triangles
    Triangle* triangles;
    // size of triangle array
    size_t num_triangles;
};



struct Edge
{
	unsigned int headVertexIdx, tailVertexIdx;

	Edge *rightPreEdge, *rightNextEdge;
	Triangle *rightFace;
	unsigned int rightVertexIdx;

	Edge *leftPreEdge, *leftNextEdge;
	Triangle *leftFace;
	unsigned int leftVertexIdx;

	int edgeNewVertexIdx;
	Edge *nextEdgeInArr;
	bool reversed;

	Edge()
	{
		rightPreEdge = NULL;
		rightNextEdge = NULL;
		rightFace = NULL;

		leftPreEdge = NULL;
		leftNextEdge = NULL;
		leftFace = NULL;

		nextEdgeInArr = NULL;

		edgeNewVertexIdx = -1;

		rightVertexIdx = -1;
		leftVertexIdx = -1;

		reversed = false;
	}

	~Edge()
	{
		delete rightPreEdge;
		delete rightNextEdge;
		delete rightFace;

		delete leftPreEdge;
		delete leftNextEdge;
		delete leftFace;
	}
};

struct WE_Vertex
{
	Edge *connectedEdge;
	bool isReversed;
	WE_Vertex()
	{
		connectedEdge = NULL;
		isReversed = false;
	}
	~WE_Vertex()
	{
		delete connectedEdge;
	}
};

enum EdgeType
{
	exist = 0,
	reverse = 1,
	unexist = 2
};

class GeometryProject
{
public:

    // constructor, invoked when object is created
    GeometryProject();
    // destructor, invoked when object is destroyed
    ~GeometryProject();

    // more detailed specifications for each function are in project.cpp.

    // Initialize the project, loading the mesh from the given filename.
    // Returns true on success.
    bool initialize( const Camera* camera, const MeshData* mesh, const char* texture_filename );
    // Clean up the project, free any memory, etc.
    void destroy();
    // Render the mesh using the given camera.
    void render( const Camera* camera );
    // Subdivide the mesh
    void subdivide();

	// initialize winged edge array
	void initializeEdgeArr(Triangle triangle);
	// get edge by using head index and tail index
	Edge* getEdge(unsigned int headVertexIdx, unsigned int tailVertexIdx);
	//initialize odd vertices array
	void initOddVerticesArr();
	//initialize even vertices array
	void initEvenVerteicesArr();
	//initialize new triangle array
	void initNewTriangleArr();
	//get odd vertiecs index from new verteices array
	int getEdgeNewVertexIdx(unsigned int headIdx, unsigned int tailIdx);
	//set triangle by using 3 indices
	Triangle setTriangle(unsigned int a, unsigned int b, unsigned int c);

private:

    MeshData mesh;

    // TODO add any other private members/functions here.
	Edge *edgeArr;
    // since this has no meaningful assignment/copy, prevent the compiler from
    // automatically generating those functions
    GeometryProject( const GeometryProject& );
    GeometryProject& operator=( const GeometryProject& );

};

} /* _462 */

#endif /* _462_GEOMETRY_PROJECT_HPP_ */

