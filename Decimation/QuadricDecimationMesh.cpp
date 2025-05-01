#include "QuadricDecimationMesh.h"

const QuadricDecimationMesh::VisualizationMode QuadricDecimationMesh::QuadricIsoSurfaces =
    NewVisualizationMode("Quadric Iso Surfaces");

void QuadricDecimationMesh::Initialize() {
    // Allocate memory for the quadric array
    size_t numVerts = mVerts.size();
    mQuadrics.reserve(numVerts);
    std::streamsize width = std::cerr.precision();  // store stream precision
    for (size_t i = 0; i < numVerts; i++) {

        // Compute quadric for vertex i here
        mQuadrics.push_back(createQuadricForVert(i));

        // Calculate initial error, should be numerically close to 0

        glm::vec3 v0 = mVerts[i].pos;
        glm::vec4 v(v0[0], v0[1], v0[2], 1);
        glm::mat4 m = mQuadrics.back();

        // TODO CHECK
        float error = glm::dot(v, (m * v));
        // std::cerr << std::scientific << std::setprecision(2) << error << " ";
    }
    std::cerr << std::setprecision(width) << std::fixed;  // reset stream precision

    // Run the initialize for the parent class to initialize the edge collapses
    DecimationMesh::Initialize();
}

/*! \lab2 Implement the computeCollapse here */
/*!
 * \param[in,out] collapse The edge collapse object to (re-)compute,
 * DecimationMesh::EdgeCollapse
 */
void QuadricDecimationMesh::computeCollapse(
    EdgeCollapse* collapse) {  // Inspired by paper in ref of intructions (page 3, section 4)
    // Compute collapse->position and collapse->cost here
    // based on the quadrics at the edge endpoints

    // Extract the two vertices of the edge to be collapsed, use a 4 vector since position is in 4d
    // with plane parameters from quadrics matrices
    size_t v1 = e(collapse->halfEdge).vert;
    size_t v2 = e(e(collapse->halfEdge).next).vert;

    // Get the positions of the two vertices and the half-distance between them
    glm::vec4 v1Pos(v(v1).pos, 1.0f);
    glm::vec4 v2Pos(v(v2).pos, 1.0f);

    // The position of the collapse vertex is given Equation 1 in Garland-Heckbert where Q is the
    // sum ofquadrics of the two vertices of the edge to be collapsed.

    // 1. Get the quadrics of the two vertices of the edge to be collapsed
    glm::mat4 Q1 = mQuadrics[v1];
    glm::mat4 Q2 = mQuadrics[v2];

    // 2. Sum the quadrics
    glm::mat4 Q = Q1 + Q2;

    // 3. Compute the position of the collapse vertex --> Equation 1 in Garland-Heckbert: v = Q^-1 *
    // (0,0,0,1)
    glm::mat4 Qinv = Q;

    // --> The bottom row of Q should be 0,0,0,1
    Qinv[0][3] = 0.0f;
    Qinv[1][3] = 0.0f;
    Qinv[2][3] = 0.0f;
    Qinv[3][3] = 1.0f;

    // Check if Q is invertible
    float eps = 0.00001f;
    bool notInvertible = abs(glm::determinant(Qinv)) < eps;

    // If invertible Q, compute new vertex position v according to Equation 1
    if (!notInvertible) {

        glm::vec4 v_ = glm::inverse(Qinv) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

        // Set the position of the collapse vertex
        collapse->position = glm::vec3(v_);
        collapse->cost = glm::dot(v_, Q * v_);  // Compute the cost of the collapse

    } else {
        // If Q is not invertible:
        // "we attempt to find the optimal vertex along the segment v1v2.If this also fails,
        // we fall back on choosing Nv from amongst the end points and the midpoint If
        // not invertible, use the midpoint of the edge" --> Heckbert-Garland citation

        // Get the quadrics of the two vertices of the edge to be collapsed
        glm::vec3 v1Pos = v(v1).pos;
        glm::vec3 v2Pos = v(v2).pos;

        // Find the mid point between the two vertices
        glm::vec3 midPos = (v1Pos + v2Pos) / 2.0f;
        glm::vec4 midPos4(midPos, 1.0f);

        // Compute the cost of the collapses at the two vertices and the midpoint
        float cost1 = glm::dot(glm::vec4(v1Pos, 1.0f), Q * glm::vec4(v1Pos, 1.0f));
        float cost2 = glm::dot(glm::vec4(v2Pos, 1.0f), Q * glm::vec4(v2Pos, 1.0f));
        float costMid = glm::dot(midPos4, Q * midPos4);

        // Choose the vertex with the lowest cost to collapse to
        if (cost1 < cost2 && cost1 < costMid) {
            collapse->position = v1Pos;
            collapse->cost = cost1;
        } else if (cost2 < cost1 && cost2 < costMid) {
            collapse->position = v2Pos;
            collapse->cost = cost2;
        } else {
            collapse->position = midPos;
            collapse->cost = costMid;
        }
    }
}

/*! After each edge collapse the vertex properties need to be updated */
void QuadricDecimationMesh::updateVertexProperties(size_t ind) {
    DecimationMesh::updateVertexProperties(ind);
    mQuadrics[ind] = createQuadricForVert(ind);
}

/*!
 * \param[in] indx vertex index, points into HalfEdgeMesh::mVerts
 */
glm::mat4 QuadricDecimationMesh::createQuadricForVert(size_t indx) const {
    glm::mat4 Q({0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f},
                {0.0f, 0.0f, 0.0f, 0.0f});

    // Get all neighbor faces of vertex vert
    std::vector<size_t> neighborFaces = FindNeighborFaces(indx);

    // Loop through all faces and sum the quadrics
    for (size_t face : neighborFaces) {

        // Add the quadric for the face to the quadric for the vertex
        Q += createQuadricForFace(face);
    }

    // The quadric for a vertex is the sum of all the quadrics for the adjacent
    // faces Tip: Matrix4x4 has an operator +=

    return Q;  // Will be used to update mQuadrics on update and therefore give us Q1 and Q2 in computeCollapse (above function)
}

/*!
 * \param[in] indx face index, points into HalfEdgeMesh::mFaces
 */
glm::mat4 QuadricDecimationMesh::createQuadricForFace(size_t indx) const {

    // Calculate the quadric (outer product of plane parameters) for a face

    // here using the formula from Garland and Heckbert - the qudric matrix for a vertex needs the
    // quadric for each face in the current vertex's neighborhood

    // The plane of the face is given by ax + by + cz + d = 0, where [a,b,c] is the normal of the
    // face and d is the distance from the vertex to the plane --> Equation 2 and down in
    // Garland-Heckbert
    float a, b, c, d;

    // Get the face normal of the face
    glm::vec3 normal = f(indx).normal;

    // Compute the plane parameters
    a = normal[0];
    b = normal[1];
    c = normal[2];

    // Get the vertex identified by indx
    size_t face = f(indx).edge;
    size_t edge = e(face).vert;
    glm::vec3 vert = v(edge).pos;

    // Compute the distance from the vertex indx to the plane, we use minus sign because of plane
    // orientation
    d = -glm::dot(normal, vert);

    // Create the quadric matrix, K_p - see Garland-Heckbert for definition
    glm::mat4 Kp{{a * a, a * b, a * c, a * d},
                 {a * b, b * b, b * c, b * d},
                 {a * c, b * c, c * c, c * d},
                 {a * d, b * d, c * d, d * d}};

    return Kp;

    // return glm::mat4();
}

void QuadricDecimationMesh::Render() {
    DecimationMesh::Render();

    glEnable(GL_LIGHTING);
    glMatrixMode(GL_MODELVIEW);

    if (mVisualizationMode == QuadricIsoSurfaces) {
        // Apply transform
        glPushMatrix();  // Push modelview matrix onto stack

        // Implement the quadric visualization here
        std::cout << "Quadric visualization not implemented" << std::endl;

        // Restore modelview matrix
        glPopMatrix();
    }
}






// #include "QuadricDecimationMesh.h"
//
// const QuadricDecimationMesh::VisualizationMode QuadricDecimationMesh::QuadricIsoSurfaces =
//     NewVisualizationMode("Quadric Iso Surfaces");
//
// void QuadricDecimationMesh::Initialize() {
//     // Allocate memory for the quadric array
//     size_t numVerts = mVerts.size();
//     mQuadrics.reserve(numVerts);
//     std::streamsize width = std::cerr.precision();  // store stream precision
//     for (size_t i = 0; i < numVerts; i++) {
//
//         // Compute quadric for vertex i here
//         mQuadrics.push_back(createQuadricForVert(i));
//
//         // Calculate initial error, should be numerically close to 0
//
//         glm::vec3 v0 = mVerts[i].pos;
//         glm::vec4 v(v0[0], v0[1], v0[2], 1);
//         glm::mat4 m = mQuadrics.back();
//
//         // TODO CHECK
//         float error = glm::dot(v, (m * v));
//         // std::cerr << std::scientific << std::setprecision(2) << error << " ";
//     }
//     std::cerr << std::setprecision(width) << std::fixed;  // reset stream precision
//
//     // Run the initialize for the parent class to initialize the edge collapses
//     DecimationMesh::Initialize();
// }
//
///*! \lab2 Implement the computeCollapse here */
///*!
// * \param[in,out] collapse The edge collapse object to (re-)compute,
// * DecimationMesh::EdgeCollapse
// */
// void QuadricDecimationMesh::computeCollapse(
//    EdgeCollapse* collapse) {  // Inspired by paper in ref of intructions (page 3, section 4)
//    // Compute collapse->position and collapse->cost here
//    // based on the quadrics at the edge endpoints
//
//    // Define the vertices to constract (v2 into v1 to get v(roof), the resulting vertex after
//    // constraction)
//    int edgeIndex = collapse->halfEdge;
//    int vert1 = mEdges[edgeIndex].vert;
//    int vert2 = mEdges[mEdges[edgeIndex].pair].vert;
//
//    // Initialize Q(roof), approximate error at v(roof), rule: Q(roof) = Q1 + Q2, step 1.
//    glm::mat4 Q =
//        mQuadrics[vert1] + mQuadrics[vert2];  // mQuadrics are in part defined by and updated by
//                                              // createQuadricForVert afteer an edge collapse
//
//    // Error at vertex v = [vx, vy, vz, 1]
//    glm::mat4 Q_hat = Q;
//    Q_hat[0][3] = 0.0f;
//    Q_hat[1][3] = 0.0f;
//    Q_hat[2][3] = 0.0f;
//    Q_hat[3][3] = 1.0f;
//
//    // Calculate cost, step 3.
//    if (glm::determinant(Q_hat) != 0.0f) {  // Check if Q_hat is inversible via its determinant
//
//        // Compute v(roof), that minimizes delta v(roof)
//        Q_hat = glm::inverse(Q_hat);  // Inverse Q_hat
//        const glm::vec4 zero(0.0f, 0.0f, 0.0f, 1.0f);
//        const glm::vec4 v_new = Q_hat * zero;  // Eq.1 in paper, v_new is v(roof)
//
//        // Extract position from v(roof) to set as new vertexes positiion
//        const glm::vec3 pos_new(v_new.x, v_new.y, v_new.z);
//        collapse->position = pos_new;
//
//        // Compute cost using formula on top of page 3 (paper): delta(v) = v(transpose)Qv
//        const glm::vec4 mult = Q * v_new;
//        collapse->cost = glm::dot(v_new, mult);
//    } else {
//
//        // Get positions directly as vec3
//        const glm::vec3& v0 = mVerts[mEdges[collapse->halfEdge].vert].pos;
//        const glm::vec3& v1 = mVerts[mEdges[mEdges[collapse->halfEdge].pair].vert].pos;
//
//        // Compute midpoint position
//        collapse->position = (v0 + v1) * 0.5f;
//
//        // Compute cost as Euclidean distance from v0 to new position
//        collapse->cost = glm::length(collapse->position - v0);
//    }
//
//    // std::cerr << "computeCollapse in QuadricDecimationMesh not implemented.\n";
//}
//
///*! After each edge collapse the vertex properties need to be updated */
// void QuadricDecimationMesh::updateVertexProperties(size_t ind) {
//     DecimationMesh::updateVertexProperties(ind);
//     mQuadrics[ind] = createQuadricForVert(ind);
// }
//
///*!
// * \param[in] indx vertex index, points into HalfEdgeMesh::mVerts
// */
// glm::mat4 QuadricDecimationMesh::createQuadricForVert(size_t indx) const {
//    glm::mat4 Q({0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f},
//                {0.0f, 0.0f, 0.0f, 0.0f});
//
//    // The quadric for a vertex is the sum of all the quadrics for the adjacent
//    // faces Tip: Matrix4x4 has an operator +=
//
//    // Added for lab2
//    // Get all neighboring faces of index (from HalfEdgeMesh.cpp)
//    std::vector<size_t> foundFaces = HalfEdgeMesh::FindNeighborFaces(indx);
//
//    // Go through all neighboring faces and sum up all quadrics (Kp)
//    for (int i = 0; i < foundFaces.size(); i++) {
//        Q += createQuadricForFace(foundFaces[i]);
//    }
//    return Q;  // Will be used to update mQuadrics on update and therefore give us Q1 and Q2 in
//               // computeCollapse (above function)
//}
//
///*!
// * \param[in] indx face index, points into HalfEdgeMesh::mFaces
// */
// glm::mat4 QuadricDecimationMesh::createQuadricForFace(size_t indx) const {
//
//    // Calculate the quadric (outer product of plane parameters) for a face
//    // here using the formula from Garland and Heckbert
//
//    // Added for lab2 using page 4 section 5 of paper:
//
//    // We want to get d from ax + by + cz + d = 0 where a, b, c is normal
//    glm::vec3 v0 = v(e(mFaces[indx].edge).vert).pos;  // is x, y, z
//    glm::vec3 normal = mFaces[indx].normal;           // is a, b, c part of p
//
//    // Compute d by breaking out from ax + by + cz + d = 0
//    float d = -glm::dot(v0, normal);  // d = -(v0 * normal)
//
//    // Create homogeneous plane vector
//    glm::vec4 p(normal, d);  // p = normal1, normal2, normal3, d
//
//    // Compute Kp matrix, fundimental error quadric, page 4 left side
//    glm::mat4 Kp;
//    for (int i = 0; i < 4; i++) {
//        for (int j = 0; j < 4; j++) {
//            Kp[i][j] = (p[i] * p[j]);
//        }
//    }
//    return Kp;  // Gets summed up to represent an entire et of planes by matrix Q (function above)
//
//    // return glm::mat4();
//}
//
// void QuadricDecimationMesh::Render() {
//    DecimationMesh::Render();
//
//    glEnable(GL_LIGHTING);
//    glMatrixMode(GL_MODELVIEW);
//
//    if (mVisualizationMode == QuadricIsoSurfaces) {
//        // Apply transform
//        glPushMatrix();  // Push modelview matrix onto stack
//
//        // Implement the quadric visualization here
//        std::cout << "Quadric visualization not implemented" << std::endl;
//
//        // Restore modelview matrix
//        glPopMatrix();
//    }
//}
