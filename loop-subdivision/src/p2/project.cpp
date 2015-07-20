/**
 * @file project.cpp
 * @brief Geometry project
 *
 * @author H. Q. Bovik (hqbovik)
 * @bug Unimplemented
 */

#include "project.hpp"
#include "application/opengl.hpp"
#include "libpng12/png.h"
#include "application/imageio.hpp"
/*
   A namespace declaration. All project files use this namespace.
   Add this declaration (and its closing) to all source/headers you create.
   Note that all #includes should be BEFORE the namespace declaration.
 */

namespace _462 {

// definitions of functions for the GeometryProject class

// constructor, invoked when object is allocated
GeometryProject::GeometryProject() { }

// destructor, invoked when object is de-allocated
GeometryProject::~GeometryProject() { }

/**
 * Initialize the project, doing any necessary opengl initialization.
 * @param camera An already-initialized camera.
 * @param mesh The mesh to be rendered and subdivided.
 * @param texture_filename The filename of the texture to use with the mesh.
 *  Is null if there is no texture data with the mesh or no texture filename
 *  was passed in the arguments, in which case textures should not be used.
 * @return true on success, false on error.
 */



GLuint vertBufferId, idxBufferId;

GLfloat LightAmbient[] =	{ 1.0f, 1.0f, 1.0f, 1.0f };		//set up ambient light color
GLfloat LightDiffuse[] =	{ 1.0f, 1.0f, 1.0f, 1.0f };		//set up diffuse light color
GLfloat LightPosition[] =   { 0.0f, 10.0f, 0.0f };			//set up light position

GLfloat modelMat[]= { 1.0f ,1.0f ,1.0f , 1.0f };


int edgeArrCounter = 0;

Vertex *vertexArr;
Triangle *triangleArr;
int newVerticeArrCounter;
WE_Vertex *weVertexArr;
GLuint texture_id;

bool GeometryProject::initialize( const Camera* camera, const MeshData* mesh, const char* texture_filename )
{
    this->mesh = *mesh;
    // TODO opengl initialization code

	// set up matrix mode, camera, enable depth and normalize
	glMatrixMode(GL_PROJECTION);
	gluPerspective(camera->get_fov_degrees(), camera->get_aspect_ratio(),
		camera->get_near_clip(),camera->get_far_clip());
	glMatrixMode(GL_MODELVIEW);
	glEnable( GL_DEPTH_TEST );
	glEnable(GL_NORMALIZE);
	//set up lighting
	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);	 
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);	 	
	glLightfv(GL_LIGHT0, GL_POSITION,LightPosition); 
	glEnable(GL_LIGHT0);
	
	//load texture
	int height, width;
	unsigned char* pointer;
	if (texture_filename!=NULL)
	{
		pointer=imageio_load_image(texture_filename,&width,&height);
		//enable textures
		glEnable(GL_TEXTURE_2D);
		glGenTextures(1, &texture_id);
		glBindTexture(GL_TEXTURE_2D, texture_id);
		//specify texture parameters
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_NEAREST);

		//set the active texture
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pointer);
	}
	return true;
}

/**
 * initialize the winged edge array. based on traverse the triangle edges
 */
void GeometryProject::initializeEdgeArr(Triangle triangle)
{
	Edge *tempEdge[3];

	tempEdge[0] = getEdge(triangle.vertices[0], triangle.vertices[1]);
	tempEdge[1] = getEdge(triangle.vertices[1], triangle.vertices[2]);
	tempEdge[2] = getEdge(triangle.vertices[2], triangle.vertices[0]);	

	for (int i = 0; i < 3; i++)
	{
		if(tempEdge[i]->rightNextEdge == NULL)
		{

			tempEdge[i]->rightNextEdge = tempEdge[(i+1)%3];
			tempEdge[i]->rightPreEdge = tempEdge[(i+2)%3];
	
			tempEdge[i]->rightVertexIdx = triangle.vertices[(i+2)%3];
			tempEdge[i]->rightFace = &triangle;

		}
		else
		{
			tempEdge[i]->leftNextEdge = tempEdge[(i+1)%3];
			tempEdge[i]->leftPreEdge = tempEdge[(i+2)%3];

			tempEdge[i]->leftVertexIdx = triangle.vertices[(i+2)%3];
			tempEdge[i]->leftFace = &triangle;
		}
	}
}

/**
 * get edge pointer based on head and tail index. if the edge is NULL, then create it.
 */
Edge* GeometryProject::getEdge(unsigned int headVertexIdx, unsigned int tailVertexIdx)
{
	int temp;
	bool reversed = false;
	if(tailVertexIdx < headVertexIdx)
	{
		temp = tailVertexIdx;
		tailVertexIdx = headVertexIdx;
		headVertexIdx = temp;
		reversed = true;
	}

	if (edgeArr[headVertexIdx].rightVertexIdx == -1)
	{
		edgeArr[headVertexIdx].headVertexIdx = headVertexIdx;
		edgeArr[headVertexIdx].tailVertexIdx = tailVertexIdx;
		edgeArr[headVertexIdx].rightVertexIdx = 0;
		if (weVertexArr[headVertexIdx].connectedEdge==NULL)
		{
			weVertexArr[headVertexIdx].connectedEdge = &edgeArr[headVertexIdx];
		}
		if (weVertexArr[tailVertexIdx].connectedEdge==NULL)
		{
			weVertexArr[tailVertexIdx].connectedEdge = &edgeArr[headVertexIdx];
			weVertexArr[tailVertexIdx].isReversed = true;
		}
		return &edgeArr[headVertexIdx];
	}
	else
	{
		Edge *tempE = &edgeArr[headVertexIdx];

		while (tempE != NULL)
		{
			if (tempE->tailVertexIdx == tailVertexIdx)
			{
				return tempE;
			}
			if (tempE->nextEdgeInArr != NULL)
			{
				tempE = tempE->nextEdgeInArr;
			}
			else
			{
				break;
			}
		}
		Edge *e = new Edge;
		e->headVertexIdx = headVertexIdx;
		e->tailVertexIdx = tailVertexIdx;
		tempE->nextEdgeInArr = e;
		if (weVertexArr[headVertexIdx].connectedEdge==NULL)
		{
			weVertexArr[headVertexIdx].connectedEdge = e;
		}
		if(weVertexArr[tailVertexIdx].connectedEdge==NULL)
		{
			weVertexArr[tailVertexIdx].connectedEdge = e;
			weVertexArr[tailVertexIdx].isReversed = true;
		}
		return e;
	}
}

/**
 * Clean up the project. Free any memory, etc.
 */
void GeometryProject::destroy()
{
  // TODO any cleanup code
}

/**
 * Clear the screen, then render the mesh using the given camera.
 * @param camera The logical camera to use.
 * @see scene/camera.hpp
 */
void GeometryProject::render( const Camera* camera )
{
  // TODO render code
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	// Clear Screen And Depth Buffer
	glLoadIdentity();

	Vector3 pos = camera->get_position();
	Vector3 vec = camera->get_up();
	Vector3 dir = camera->get_direction() + camera->get_position();
	gluLookAt(pos.x , pos.y , pos.z, dir.x , dir.y , dir.z, vec.x , vec.y , vec.z);

	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, modelMat);
	
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < mesh.num_triangles; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			int vertex0 = mesh.triangles[i].vertices[j];
			glNormal3f(mesh.vertices[vertex0].normal.x, mesh.vertices[vertex0].normal.y, mesh.vertices[vertex0].normal.z);
			glTexCoord2f(mesh.vertices[vertex0].texture_coord.x, mesh.vertices[vertex0].texture_coord.y);
			glVertex3d(mesh.vertices[vertex0].position.x, mesh.vertices[vertex0].position.y, mesh.vertices[vertex0].position.z);
		}

	}
	glEnd();	
}

/**
 * Subdivide the mesh that we are rendering using Loop subdivison.
 */
void GeometryProject::subdivide()
{
    // TODO perform a single subdivision.
	// reset the edge array's counter
	edgeArrCounter = 0;
	// allocate a new edge array
	edgeArr = new Edge[mesh.num_triangles*3];
	// allocate a new wing edge vertex array,
	//this array used for store the edge pointer connected to vertex.
	weVertexArr = new WE_Vertex[mesh.num_vertices];

	//traverse all triangles and initialize winged edge array
	for (size_t i = 0; i < mesh.num_triangles; i++)
	{
		initializeEdgeArr(mesh.triangles[i]);
	}

	//allocate new vertice and triangle array
	vertexArr = new Vertex[mesh.num_triangles*6];
	triangleArr = new Triangle[mesh.num_triangles*4];

	// set new vertice array counter to the number of vertices
	// in the new vertices array, the even vertices store in the old position
	// the odd vertices store after even vertices, so the array counter should start
	// from the number of old vertices.
	newVerticeArrCounter = mesh.num_vertices;

	// initialize the odd vertices, even vertices, and triangle array
	initOddVerticesArr();
	initEvenVerteicesArr();
	initNewTriangleArr();

	// set the new mesh
	mesh.num_triangles = mesh.num_triangles*4;
	mesh.num_vertices = newVerticeArrCounter;
	mesh.triangles = triangleArr;
	mesh.vertices = vertexArr;

	printf("%d", mesh.num_vertices);
}

/**
 * initialize the odd vertices array by traverse the edge array list
 */
void GeometryProject::initOddVerticesArr()
{
	for (size_t i = 0; i < (mesh.num_vertices); i++)
	{
		Edge *temp = &edgeArr[i];
		while (temp != NULL && temp->rightFace!=NULL)
		{
			Vertex *v = new Vertex;
			if (edgeArr[i].leftFace!=NULL && temp->leftVertexIdx != -1)
			{
				//if(temp->leftVertexIdx >= mesh.num_vertices) printf("%d %d\n", temp->leftVertexIdx, mesh.num_vertices);
				v->position = 3*(mesh.vertices[temp->headVertexIdx].position)/8 +
						3*(mesh.vertices[temp->tailVertexIdx].position)/8 +
						(mesh.vertices[temp->leftVertexIdx].position)/8 +
						(mesh.vertices[temp->rightVertexIdx].position)/8;

				v->normal = 3*(mesh.vertices[temp->headVertexIdx].normal)/8 +
						3*(mesh.vertices[temp->tailVertexIdx].normal)/8 +
						(mesh.vertices[temp->leftVertexIdx].normal)/8 +
						(mesh.vertices[temp->rightVertexIdx].normal)/8;

				v->texture_coord = 3*(mesh.vertices[temp->headVertexIdx].texture_coord)/8 +
						3*(mesh.vertices[temp->tailVertexIdx].texture_coord)/8 +
						(mesh.vertices[temp->leftVertexIdx].texture_coord)/8 +
						(mesh.vertices[temp->rightVertexIdx].texture_coord)/8;

				vertexArr[newVerticeArrCounter] = *v;
				temp->edgeNewVertexIdx = newVerticeArrCounter;
				newVerticeArrCounter++;
			}
			else
			{
				v->position = mesh.vertices[temp->headVertexIdx].position/2 +
							mesh.vertices[temp->tailVertexIdx].position/2;

				v->normal = mesh.vertices[temp->headVertexIdx].normal/2 +
							mesh.vertices[temp->tailVertexIdx].normal/2;

				v->texture_coord = mesh.vertices[temp->headVertexIdx].texture_coord/2 +
							mesh.vertices[temp->tailVertexIdx].texture_coord/2;

				vertexArr[newVerticeArrCounter] = *v;
				temp->edgeNewVertexIdx = newVerticeArrCounter;
				newVerticeArrCounter++;
			}

			delete v;
			temp = temp->nextEdgeInArr;
		}
	}
}

/**
 * initialize the even vertices array by traverse the edge array list
 */
void GeometryProject::initEvenVerteicesArr()
{
	Vertex *evenVertexArr = new Vertex[mesh.num_vertices];
	for (int i = 0; i < mesh.num_vertices; i++)
	{
		evenVertexArr[i] = mesh.vertices[i];
	}
	
	for (int i = 0; i < mesh.num_vertices; i++)
	{
		Edge *tempEdge = weVertexArr[i].connectedEdge;
		int headIdx;
		bool isBoundry = false;
		Vector3 position = Vector3::Zero;
		int counter = 0;

		if (!weVertexArr[i].isReversed)
			headIdx = weVertexArr[i].connectedEdge->headVertexIdx;
		else
			headIdx = weVertexArr[i].connectedEdge->tailVertexIdx;

		while (tempEdge!=NULL)
		{

			if (tempEdge->leftPreEdge==NULL || tempEdge->rightPreEdge==NULL)
			{
				Vector3 tempPosition = Vector3::Zero;
				if (tempEdge->headVertexIdx == headIdx)
				{
					position = mesh.vertices[tempEdge->tailVertexIdx].position;
				}
				else
				{
					position = mesh.vertices[tempEdge->headVertexIdx].position;
				}
				while (tempEdge->rightNextEdge!=NULL && tempEdge->leftNextEdge!=NULL)
				{
					if (tempEdge->rightNextEdge->headVertexIdx == headIdx)
					{
						tempPosition = mesh.vertices[tempEdge->rightNextEdge->headVertexIdx].position;
						tempEdge = tempEdge->rightNextEdge;
					}
					else if (tempEdge->rightNextEdge->tailVertexIdx == headIdx)
					{
						tempPosition = mesh.vertices[tempEdge->rightNextEdge->tailVertexIdx].position;
						tempEdge = tempEdge->rightNextEdge;
					}
					else if (tempEdge->leftNextEdge->headVertexIdx == headIdx)
					{
						tempPosition = mesh.vertices[tempEdge->leftNextEdge->headVertexIdx].position;
						tempEdge = tempEdge->leftNextEdge;
					}
					else if (tempEdge->leftNextEdge->tailVertexIdx == headIdx)
					{
						tempPosition = mesh.vertices[tempEdge->leftNextEdge->tailVertexIdx].position;
						tempEdge = tempEdge->leftNextEdge;
					}
				}
				position += tempPosition;
				isBoundry = true;				
				break;
			}

			else
			{
				if (tempEdge->rightPreEdge->headVertexIdx == headIdx)
				{
					position += mesh.vertices[tempEdge->rightPreEdge->tailVertexIdx].position;
					tempEdge = tempEdge->rightPreEdge;
					counter++;					
					if (tempEdge == weVertexArr[i].connectedEdge)
					{
						break;
					}
					continue;
				}
				else if (tempEdge->rightPreEdge->tailVertexIdx == headIdx)
				{
					position += mesh.vertices[tempEdge->rightPreEdge->headVertexIdx].position;
					tempEdge = tempEdge->rightPreEdge;
					counter++;
					if (tempEdge == weVertexArr[i].connectedEdge)
					{
						break;
					}
					continue;
				}
				else if (tempEdge->leftPreEdge->headVertexIdx == headIdx)
				{
					position += mesh.vertices[tempEdge->leftPreEdge->tailVertexIdx].position;
					tempEdge = tempEdge->leftPreEdge;
					counter++;
					if (tempEdge == weVertexArr[i].connectedEdge)
					{
						break;
					}
					continue;
				}
				else if (tempEdge->leftPreEdge->tailVertexIdx == headIdx)
				{
					position += mesh.vertices[tempEdge->leftPreEdge->headVertexIdx].position;
					tempEdge = tempEdge->leftPreEdge;
					counter++;
					if (tempEdge == weVertexArr[i].connectedEdge)
					{
						break;
					}
					continue;
				}	
			}
		}
	
		if (!isBoundry)
		{
			float beta=1.0/counter*(5.0/8.0-(3.0/8.0+1.0/4.0*cos(2.0*PI/counter))*(3.0/8.0+1.0/4.0*cos(2.0*PI/counter)));
			evenVertexArr[i].position = (1-beta*counter)*(evenVertexArr[i].position) + beta*(position);
		}
		else
		{
			evenVertexArr[i].position = 3*evenVertexArr[i].position/4 + 2*(position)/8;
		}
		
	}

	for (int i = 0; i < mesh.num_vertices; i++)
	{		
		vertexArr[i] = evenVertexArr[i];
	}
}

/**
 * initialize the new triangle array.
 */
void GeometryProject::initNewTriangleArr()
{
	for (int i = 0; i < mesh.num_triangles; i++)
	{
		int idx0, idx1, idx2, idx3, idx4, idx5;
		idx0 = mesh.triangles[i].vertices[0];
		idx2 = mesh.triangles[i].vertices[1];
		idx5 = mesh.triangles[i].vertices[2];

		idx1 = getEdgeNewVertexIdx(idx0,idx2);
		idx3 = getEdgeNewVertexIdx(idx0,idx5);
		idx4 = getEdgeNewVertexIdx(idx2,idx5);

		triangleArr[i*4+0] = setTriangle(idx0, idx1, idx3);
		triangleArr[i*4+1] = setTriangle(idx1, idx4, idx3);
		triangleArr[i*4+2] = setTriangle(idx1, idx2, idx4);
		triangleArr[i*4+3] = setTriangle(idx3, idx4, idx5);
	}
}

/**
 * get the odd vertices index in new vertex array, by using head and tail index
 */
int GeometryProject::getEdgeNewVertexIdx(unsigned int headIdx, unsigned int tailIdx)
{
	int tempIdx;
	if(tailIdx < headIdx)
	{
		tempIdx = tailIdx;
		tailIdx = headIdx;
		headIdx = tempIdx;
	}
	Edge *temp = &edgeArr[headIdx];
	while (temp != NULL)
	{
		if (tailIdx == temp->tailVertexIdx)
		{
			return temp->edgeNewVertexIdx;
		}

		temp = temp->nextEdgeInArr;
	}
	return -1;
}

/**
 * create and return a triangle based on indices.
 */
Triangle GeometryProject::setTriangle(unsigned int a, unsigned int b, unsigned int c)
{
	
	Triangle *triangle = new Triangle;
	triangle->vertices[0] = a;
	triangle->vertices[1] = b;
	triangle->vertices[2] = c;
	
	return *triangle;
}

} /* _462 */

