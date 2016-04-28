#include "SkeletalModel.h"

#include <FL/Fl.H>
#include <fstream>

using namespace std;

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
	loadSkeleton(skeletonFile);

	m_mesh.load(meshFile);
	m_mesh.loadAttachments(attachmentsFile, m_joints.size());

	computeBindWorldToJointTransforms();
	updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(Matrix4f cameraMatrix, bool skeletonVisible)
{
	// draw() gets called whenever a redraw is required
	// (after an update() occurs, when the camera moves, the window is resized, etc)

	m_matrixStack.clear();
	m_matrixStack.push(cameraMatrix);

	if( skeletonVisible )
	{
		drawJoints();

		drawSkeleton();
	}
	else
	{
		// Clear out any weird matrix we may have been using for drawing the bones and revert to the camera matrix.
		glLoadMatrixf(m_matrixStack.top());

		// Tell the mesh to draw itself.
		m_mesh.draw();
	}
}

// This method should compute m_rootJoint and populate m_joints.
void SkeletalModel::loadSkeleton( const char* filename )
{
	// Load the skeleton from file here.
    ifstream infile(filename);
    float f1,f2, f3;
    int num;
    while (infile >> f1 >> f2 >> f3 >> num){
        cout << f1 << "\t"
        << f2 << "\t"
        << f3 << "\t"
        << num << endl;
        
        Joint *joint = new Joint;
        joint->transform = Matrix4f::translation(f1, f2, f3);
        m_joints.push_back(joint);
        
        if (num == -1 ){
            m_rootJoint = joint;
        } else {
            m_joints[num]->children.push_back(joint);
        }
        
    }
}


void SkeletalModel::drawjoint(Joint* joint){
//    cout << "draw away!!" << endl;
    m_matrixStack.push( joint->transform );
    
    for (int i=0 ; i<(joint->children.size()); i++){
//        cout << "num of children: " << joint->children.size() << endl;
        drawjoint(joint->children[i]);
    }
    
    glLoadMatrixf(m_matrixStack.top());
    glutSolidSphere( 0.025f, 12, 12 );
    
    m_matrixStack.pop();
    
}

void SkeletalModel::drawJoints( )
{
	// Draw a sphere at each joint. You will need to add a recursive helper function to traverse the joint hierarchy.
	//
	// We recommend using glutSolidSphere( 0.025f, 12, 12 )
	// to draw a sphere of reasonable size.
	//
	// You are *not* permitted to use the OpenGL matrix stack commands
	// (glPushMatrix, glPopMatrix, glMultMatrix).
	// You should use your MatrixStack class
	// and use glLoadMatrix() before your drawing call.
    
    SkeletalModel::drawjoint(m_rootJoint);
    
    
    
}




void SkeletalModel::drawSkeleton( )
{
	// Draw boxes between the joints. You will need to add a recursive helper function to traverse the joint hierarchy.
    
    SkeletalModel::drawBones(m_rootJoint, true);
    
}

void SkeletalModel::drawBones(Joint* joint, bool root){
    // DFS
    m_matrixStack.push( joint->transform );
    
    for (int i=0 ; i<joint->children.size() ; i++){
        SkeletalModel::drawBones(joint->children[i]);
    }
    
    m_matrixStack.pop();
    
    if (!root){
        
        Matrix4f translate_z = Matrix4f::translation(0, 0, 0.5);
        float dist = Vector3f(joint->transform(0,3), joint->transform(1,3), joint->transform(2,3)).abs();
        Matrix4f scale = Matrix4f::scaling(0.05f, 0.05f, dist);
        
        Vector3f z = Vector3f(joint->transform(0,3), joint->transform(1,3), joint->transform(2,3)).normalized();
        Vector3f rnd = Vector3f(0.0f, 0.0f, 1.0f);
        Vector3f y = Vector3f::cross(z, rnd).normalized();
        Vector3f x = Vector3f::cross(y, z).normalized();
        Matrix3f tempRot = Matrix3f(x, y, z);
        Matrix4f rot = Matrix4f::identity();
        rot.setSubmatrix3x3(0, 0, tempRot);
        
        m_matrixStack.push(rot);
        m_matrixStack.push(scale);
        m_matrixStack.push(translate_z);
        
        glLoadMatrixf(m_matrixStack.top());
        glutSolidCube(1.0f);
        
        m_matrixStack.pop();
        m_matrixStack.pop();
        m_matrixStack.pop();
    }
    
}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
	// Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
    Matrix4f rotX = Matrix4f::rotateX(rX);
    Matrix4f rotY = Matrix4f::rotateX(rY);
    Matrix4f rotZ = Matrix4f::rotateX(rZ);
    Matrix4f rotation = rotX*rotY*rotZ;
    
    m_joints[jointIndex]->transform.setSubmatrix3x3(0, 0, rotation.getSubmatrix3x3(0, 0));
    
}


void SkeletalModel::computeBindWorldToJointTransforms()
{
	// 2.3.1. Implement this method to compute a per-joint transform from
	// world-space to joint space in the BIND POSE.
	//
	// Note that this needs to be computed only once since there is only
	// a single bind pose.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.

    m_matrixStack.clear();
    BindWorldToJointTransform( m_rootJoint );
    
}

void SkeletalModel::BindWorldToJointTransform(Joint* joint){
    
    m_matrixStack.push( joint->transform );
    
    joint->bindWorldToJointTransform = m_matrixStack.top().inverse();
    
    for (int i=0 ; i<(joint->children.size()); i++){
        BindWorldToJointTransform(joint->children[i]);
    }
    
    
    
    m_matrixStack.pop();
    
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
	// 2.3.2. Implement this method to compute a per-joint transform from
	// joint space to world space in the CURRENT POSE.
	//
	// The current pose is defined by the rotations you've applied to the
	// joints and hence needs to be *updated* every time the joint angles change.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
    
    m_matrixStack.clear();
    CurrentWorldToJointTransform(m_rootJoint);
}

void SkeletalModel::CurrentWorldToJointTransform(Joint* joint){
    
    m_matrixStack.push( joint->transform );
    joint->currentJointToWorldTransform = m_matrixStack.top();
    
    
    for (int i=0 ; i<(joint->children.size()); i++){
        CurrentWorldToJointTransform(joint->children[i]);
    }
    
    m_matrixStack.pop();
    
}

void SkeletalModel::updateMesh()
{
	// 2.3.2. This is the core of SSD.
	// Implement this method to update the vertices of the mesh
	// given the current state of the skeleton.
	// You will need both the bind pose world --> joint transforms.
	// and the current joint --> world transforms.
    // wT1B1-1p + (1 - w)T2B2-1p.
    
    vector <Vector3f> temp;
    
    for (int i=0 ; i < m_mesh.currentVertices.size() ; i++){
        
        Vector4f newPoint = Vector4f();
        for (int j=0 ; j<m_mesh.attachments[i].size() ; j++){
            
            newPoint = newPoint + m_mesh.attachments[i][j] *
            (   m_joints[j+1]->currentJointToWorldTransform
                * m_joints[j+1]->bindWorldToJointTransform
                * Vector4f(m_mesh.bindVertices[i][0], m_mesh.bindVertices[i][1],
                        m_mesh.bindVertices[i][2], 1.0f) );
            
        }
        
        temp.push_back(Vector3f(newPoint[0], newPoint[1], newPoint[2]));
        
    }
    m_mesh.currentVertices = temp;
    
    
}

