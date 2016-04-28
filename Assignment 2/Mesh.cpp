#include "Mesh.h"


using namespace std;

/*

 std::vector< Vector3f > bindVertices;
 std::vector< Tuple3u > faces;
 
 */

void Mesh::load( const char* filename )
{
	// 2.1.1. load() should populate bindVertices, currentVertices, and faces

	// Add your code here.
    
    ifstream infile(filename);
    float v1,v2, v3;
    int f1,f2,f3;
    char type[1];
    
    while (infile >> type){
        if (type[0] == 'v'){
            infile >> v1 >> v2 >> v3;
            bindVertices.push_back(Vector3f(v1,v2,v3));
            cout << "v Found!" << endl;
            
        } else if (type[0] == 'f'){
            infile >> f1 >> f2 >> f3;
            faces.push_back(Tuple3u(f1,f2,f3));
            cout << "f Found!" << endl;
        }
    }

	// make a copy of the bind vertices as the current vertices
	currentVertices = bindVertices;
}

void Mesh::draw()
{
	// Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".
    
    
    glBegin(GL_TRIANGLES);
    for (int i=0 ; i<faces.size() ; i++){
        // Tuple3u faces[i] contains 3 vertice index
        // a, d, g are vertice indices
        // c, f, i are respective normals
        
        int a = faces[i][0];
        int d = faces[i][1];
        int g = faces[i][2];
        
        Vector3f normal = Vector3f::cross((currentVertices[faces[i][1]-1]-currentVertices[faces[i][0]-1]), (currentVertices[faces[i][2]-1]-currentVertices[faces[i][0]-1])).normalized();
        
        glNormal3f(normal[0], normal[1], normal[2]);

        glVertex3f(currentVertices[a-1][0], currentVertices[a-1][1], currentVertices[a-1][2]);
        glVertex3f(currentVertices[d-1][0], currentVertices[d-1][1], currentVertices[d-1][2]);
        glVertex3f(currentVertices[g-1][0], currentVertices[g-1][1], currentVertices[g-1][2]);
        
    }
    glEnd();
    
}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 2.2. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments
    // std::vector< std::vector< float > > attachments;
    ifstream infile(filename);
    float holder;
    
    
    for (int i=0 ; i<currentVertices.size() ; i++){
        vector <float> line;
        for (int j=0 ; j<numJoints-1 ; j++){
            infile >> holder;
            line.push_back(holder);
//            cout << " " <<holder;
        }
//        cout << endl;
        attachments.push_back(line);
    }
}
