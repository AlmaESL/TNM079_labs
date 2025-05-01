/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 * Acknowledgements for original code base:
 * - Gunnar Johansson
 * - Ken Museth
 * - Michael Bang Nielsen
 * - Ola Nilsson
 * - Andreas Soderstrom
 *
 * Code updated in the period 2017-2018 by Jochen Jankowai
 *
 *************************************************************************************************/
#include "SimpleDecimationMesh.h"
#include "GUI/GLCamera.h"

// void SimpleDecimationMesh::computeCollapse(EdgeCollapse* collapse) {
//     // The new vertex position is implicitly stored as the
//     // position halfway along the edge. The cost is computed as
//     // the vertex-to-vertex distance between the new vertex
//     // and the old vertices at the edge's endpoints
//     const glm::vec3& v0 = mVerts[mEdges[collapse->halfEdge].vert].pos;
//     const glm::vec3& v1 = mVerts[mEdges[mEdges[collapse->halfEdge].pair].vert].pos;

//    collapse->position = 0.5f * (v0 + v1);
//    collapse->cost = glm::distance(collapse->position, v0);
//}

void SimpleDecimationMesh::computeCollapse(EdgeCollapse* collapse) {

    /*comment:
    We want to track camera position by maybe calling this collapse function in camera.h move().
    So that camera pos updates here and we can collapse wrt camera pos --> LoD
    */

    GLCamera mCamera;
    glm::vec3 cameraPos_original = glm::vec3(0.0f, 0.0f, -50.0f);

    // The z-axis is back and forth through the screen
    glm::vec3 cameraPos1 = mCamera.GetPosition() + glm::vec3(3.0f, 0.0f, 1.0f);
    glm::vec3 cameraPos2 = mCamera.GetPosition() + glm::vec3(3.0f, 0.0f, 50.0f);
    glm::vec3 cameraPos3 = mCamera.GetPosition() + glm::vec3(3.0f, 0.0f, 10.0f);
    glm::vec3 cameraPos4 = mCamera.GetPosition() + glm::vec3(3.0f, 0.0f, 25.0f);
    glm::vec3 currentCam = cameraPos3;

    const glm::vec3& v0 = mVerts[mEdges[collapse->halfEdge].vert].pos;
    const glm::vec3& v1 = mVerts[mEdges[mEdges[collapse->halfEdge].pair].vert].pos;

    // We take the position in the middle of the two vertices
    glm::vec3 tempPos = 0.5f * (v0 + v1);
    collapse->position = tempPos;

    // Create a linear level of resolution from camera position to object point
    currentCam = currentCam * glm::vec3(0.01f, 0.0f, 0.01f);

    // float distanceToCam = EuclideanDistance(tempPos, cameraPos);
    float distanceToCam = glm::distance(tempPos, currentCam);

    collapse->cost = glm::distance(collapse->position, v0) / distanceToCam;

    /*
    Camera is placed at -50, we use a linear scale 0-50, 0 is colse to camera ie far away ie low LOD

    */
}
