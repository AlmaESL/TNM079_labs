#include <Geometry/HalfEdgeMesh.h>
#include <gtc/type_ptr.hpp>
#include <iterator>

HalfEdgeMesh::HalfEdgeMesh() {}

HalfEdgeMesh::~HalfEdgeMesh() {}

/*! \lab1 Implement the addFace */
/*!
 * \param[in] v1 vertex 1, glm::vec3
 * \param[in] v2 vertex 2, glm::vec3
 * \param[in] v3 vertex 3, glm::vec3
 */
bool HalfEdgeMesh::AddFace(const std::vector<glm::vec3>& verts) {
    // Add your code here
    // std::cerr << "ADD TRIANGLE NOT IMPLEMENTED. ";

    // Add the vertices of the face/triangle (1. in instructions, returns indices)
    std::size_t v1 = AddVertex(verts.at(0));
    std::size_t v2 = AddVertex(verts.at(1));
    std::size_t v3 = AddVertex(verts.at(2));

    // Add all half-edge pairs (2. in instructions, returns 2 indices (first is inner, second is
    // outer HE))
    std::pair<std::size_t, std::size_t> HE_pair1 =
        AddHalfEdgePair(v1, v2);  // we're going through this CCW
    std::pair<std::size_t, std::size_t> HE_pair2 = AddHalfEdgePair(v2, v3);
    std::pair<std::size_t, std::size_t> HE_pair3 = AddHalfEdgePair(v3, v1);

    // Connect inner ring - consisting of next and previous half-edge for each edge (3. in
    // instructions) Access the first half-edge in eacg pair using the first half-edge in each pair
    // (the second element is the outer loop's helf-edges)
    e(HE_pair1.first).next =
        HE_pair2.first;  // e() returns index of half-edge specified (all half-edges in a vector)
    e(HE_pair1.first).prev = HE_pair3.first;

    e(HE_pair2.first).next = HE_pair3.first;
    e(HE_pair2.first).prev = HE_pair1.first;

    e(HE_pair3.first).next = HE_pair1.first;
    e(HE_pair3.first).prev = HE_pair2.first;

    // Finally, create the face, don't forget to set the normal (which should be
    // normalized) (4. in instructions)
    Face face;                   // initialization of face instance
    face.edge = HE_pair1.first;  // connect the face to the first half-edge of the inner loop
    mFaces.push_back(face);

    // All half-edges share the same left face (previously added), connect inner edges to the newly
    // created face (5. in instructions)
    e(HE_pair1.first).face = mFaces.size() - 1;
    e(HE_pair2.first).face = mFaces.size() - 1;
    e(HE_pair3.first).face = mFaces.size() - 1;

    // compute and assign normal to added face, which will be the last face (mFaces.size()-1) in the
    // face vector
    mFaces.back().normal = FaceNormal(mFaces.size() - 1);

    // Optionally, track the (outer) boundary half-edges
    // to represent non-closed surfaces
    return true;
}

/*!
 * \param [in] v the vertex to add, glm::vec3
 * \return the index to the vertex
 */
size_t HalfEdgeMesh::AddVertex(const glm::vec3& v) {
    std::map<glm::vec3, size_t>::iterator it = mUniqueVerts.find(v);
    if (it != mUniqueVerts.end()) {
        return (*it).second;  // get the index of the already existing vertex
    }

    const auto indx = GetNumVerts();
    mUniqueVerts[v] = indx;  // op. [ ] constructs a new entry in map
    Vertex vert;
    vert.pos = v;
    mVerts.push_back(vert);  // add it to the vertex list

    return indx;
}

/*!
 * Inserts a half edge pair between HalfEdgeMesh::Vertex pointed to by v1 and
 * v2. The first HalfEdgeMesh::HalfEdge (v1->v2) is the inner one, and the
 * second (v2->v1) is the outer.
 * \param [in] v1 size_t index of vertex 1
 * \param [in] v2 size_t index of vertex 2
 * \return a pair the indices to the half-edges
 */
std::pair<size_t, size_t> HalfEdgeMesh::AddHalfEdgePair(size_t v1, size_t v2) {
    std::map<OrderedPair, size_t>::iterator it = mUniqueEdgePairs.find(OrderedPair(v1, v2));
    if (it != mUniqueEdgePairs.end()) {
        auto indx1 = it->second;
        auto indx2 = e(it->second).pair;
        if (v1 != e(indx1).vert) {
            std::swap(indx1, indx2);  // sort correctly
        }
        return {indx1, indx2};
    }

    // If not found, calculate both half-edges indices
    const auto indx1 = mEdges.size();
    const auto indx2 = indx1 + 1;

    // Create edges and set pair index
    HalfEdge edge1, edge2;
    edge1.pair = indx2;
    edge2.pair = indx1;

    // Connect the edges to the verts
    edge1.vert = v1;
    edge2.vert = v2;

    // Connect the verts to the edges
    v(v1).edge = indx1;
    v(v2).edge = indx2;

    // Store the edges in mEdges
    mEdges.push_back(edge1);
    mEdges.push_back(edge2);

    // Store the first edge in the map as an OrderedPair
    OrderedPair op(v1, v2);
    mUniqueEdgePairs[op] = indx1;  // op. [ ] constructs a new entry in map, ordering not important
    // sorting done when retrieving

    return {indx1, indx2};
}

/*! \lab1 HalfEdgeMesh Implement the MergeAdjacentBoundaryEdge */
/*!
 * Merges the outer UNINITIALIZED/BORDER to an already set inner half-edge.
 * \param [in] indx the index of the INNER half-edge, size_t
 */
void HalfEdgeMesh::MergeOuterBoundaryEdge(size_t innerEdge) {
    // Add your code here
    // 1. Merge first loop (around innerEdge->vert)
    // 2. Find leftmost edge, last edge counter clock-wise
    // 3. Test if there's anything to merge
    // 3a. If so merge the gap
    // 3b. And set border flags
    // 4. Merge second loop (around innerEdge->pair->vert)
}

/*! Proceeds to check if the mesh is valid. All indices are inspected and
 * checked to see that they are initialized. The method checks: mEdges, mFaces
 * and mVerts. Also checks to see if all verts have a neighborhood using the
 * findNeighbourFaces method.
 */
void HalfEdgeMesh::Validate() {
    std::vector<HalfEdge>::iterator iterEdge = mEdges.begin();
    std::vector<HalfEdge>::iterator iterEdgeEnd = mEdges.end();
    while (iterEdge != iterEdgeEnd) {
        if ((*iterEdge).face == EdgeState::Uninitialized ||
            (*iterEdge).next == EdgeState::Uninitialized ||
            (*iterEdge).pair == EdgeState::Uninitialized ||
            (*iterEdge).prev == EdgeState::Uninitialized ||
            (*iterEdge).vert == EdgeState::Uninitialized) {
            std::cerr << "HalfEdge " << iterEdge - mEdges.begin() << " not properly initialized"
                      << std::endl;
        }

        iterEdge++;
    }
    std::cerr << "Done with edge check (checked " << GetNumEdges() << " edges)" << std::endl;

    std::vector<Face>::iterator iterTri = mFaces.begin();
    std::vector<Face>::iterator iterTriEnd = mFaces.end();
    while (iterTri != iterTriEnd) {
        if ((*iterTri).edge == EdgeState::Uninitialized) {
            std::cerr << "Tri " << iterTri - mFaces.begin() << " not properly initialized"
                      << std::endl;
        }

        iterTri++;
    }
    std::cerr << "Done with face check (checked " << GetNumFaces() << " faces)" << std::endl;

    std::vector<Vertex>::iterator iterVertex = mVerts.begin();
    std::vector<Vertex>::iterator iterVertexEnd = mVerts.end();
    while (iterVertex != iterVertexEnd) {
        if ((*iterVertex).edge == EdgeState::Uninitialized) {
            std::cerr << "Vertex " << iterVertex - mVerts.begin() << " not properly initialized"
                      << std::endl;
        }

        iterVertex++;
    }
    std::cerr << "Done with vertex check (checked " << GetNumVerts() << " vertices)" << std::endl;

    std::cerr << "Looping through triangle neighborhood of each vertex... ";
    iterVertex = mVerts.begin();
    iterVertexEnd = mVerts.end();
    int emptyCount = 0;
    std::vector<size_t> problemVerts;
    while (iterVertex != iterVertexEnd) {
        std::vector<size_t> foundFaces = FindNeighborFaces(iterVertex - mVerts.begin());
        std::vector<size_t> foundVerts = FindNeighborVertices(iterVertex - mVerts.begin());
        if (foundFaces.empty() || foundVerts.empty()) emptyCount++;
        std::set<size_t> uniqueFaces(foundFaces.begin(), foundFaces.end());
        std::set<size_t> uniqueVerts(foundVerts.begin(), foundVerts.end());
        if (foundFaces.size() != uniqueFaces.size() || foundVerts.size() != uniqueVerts.size()) {
            problemVerts.push_back(iterVertex - mVerts.begin());
        }
        iterVertex++;
    }
    std::cerr << std::endl << "Done: " << emptyCount << " isolated vertices found" << std::endl;
    if (problemVerts.size()) {
        std::cerr << std::endl
                  << "Found " << problemVerts.size() << " duplicate faces in vertices: ";
        std::copy(problemVerts.begin(), problemVerts.end(),
                  std::ostream_iterator<size_t>(std::cerr, ", "));
        std::cerr << "\n";
    }
    std::cerr << std::endl
              << "The mesh has genus " << Genus() << ", and consists of " << Shells()
              << " shells.\n";

    std::cerr << "# Faces: " << std::to_string(mFaces.size()) << std::endl;
    std::cerr << "# Edges: " << std::to_string(mEdges.size() / 2) << std::endl;
    std::cerr << "# Vertices: " << std::to_string(mVerts.size()) << std::endl;
}

/*! \lab1 Implement the FindNeighborVertices */
/*! Loops over the neighborhood of a vertex and collects all the vertices sorted
 * counter clockwise. \param [in] vertexIndex  the index to vertex, size_t
 * \return a vector containing the indices to all the found vertices.
 */
std::vector<size_t> HalfEdgeMesh::FindNeighborVertices(size_t vertexIndex) const {
    // Collected vertices, sorted counter clockwise!
    std::vector<size_t> oneRing;

    // Choose a starting half-edge that originates from the vertex
    std::size_t start_edge = v(vertexIndex).edge;
    // Temp edge that iterates over all edges in the 1-ring of vertex vertexIndex
    std::size_t edge_iterator = start_edge;

    // Loop all half-edge pairs that are connected to the vertex
    while (true) {

        // Extract the vertex from the current half-edge and add it to the 1-ring vector
        std::size_t v = e(e(edge_iterator).next).vert;
        oneRing.push_back(v);

        // Step to the next half-edge in the 1-ring
        edge_iterator = e(e(edge_iterator).prev).pair;

        // Break once we've looped through all edges in the 1-ring
        if (edge_iterator == start_edge) {
            break;
        }
    }

    return oneRing;
}

/*! \lab1 Implement the FindNeighborFaces */
/*! Loops over the neighborhood of a vertex and collects all the faces sorted
 * counter clockwise. \param [in] vertexIndex  the index to vertex, size_t
 * \return a vector containing the indices to all the found faces.
 */
std::vector<size_t> HalfEdgeMesh::FindNeighborFaces(size_t vertexIndex) const {
    // Collected faces, sorted counter clockwise!
    std::vector<size_t> foundFaces;

    // Choose a starting half-edge that originates from the vertex
    std::size_t start_edge = v(vertexIndex).edge;
    // Temp edge that iterates over all edges in the 1-ring of vertex vertexIndex
    std::size_t edge_iterator = start_edge;

    // Loop through all edges in the 1-ring and get the face of each half-edge
    while (true) {

        // Extract face from current half-edge
        std::size_t face = e(edge_iterator).face;

        foundFaces.push_back(face);

        // Step to the next half-edge in the 1-ring
        edge_iterator = e(e(edge_iterator).prev).pair;

        // Break once we've looped through all edges in the 1-ring
        if (edge_iterator == start_edge) {
            break;
        }
    }

    return foundFaces;

 }

/*! \lab1 Implement the curvature
Gaussian curvature as per Lec 2 slide 9 (Eq.7)
Descrete mean as per Lec 2 slide 11 (Eq.9)*/
float HalfEdgeMesh::VertexCurvature(size_t vertexIndex) const {

    // ---------------------------- Gauss Curvature ------------------------------ //

    /*/ Copy code from SimpleMesh or compute more accurate estimate

    // Get the 1-ring of the vertex
     std::vector<size_t> oneRing = FindNeighborVertices(vertexIndex);
     assert(oneRing.size() != 0); // Check that the 1-ring exists

    // Get vertex position indecies
     size_t curr, next;
     const glm::vec3& vi = mVerts.at(vertexIndex).pos;

     float angleSum = 0.f;
     float area = 0.f;

     for (size_t i = 0; i < oneRing.size(); i++) {

        // Connections between vertices in the 1-ring
        curr = oneRing.at(i);

        // Check if we're at the last vertex in the 1-ring, then connect to the first vertex,
        // otherwise connect to the next vertex
        if (i < oneRing.size() - 1) {
            next = oneRing.at(i + 1);
        } else {
            next = oneRing.front();
        }

        // Find vertices in 1-ring according to figure 5 in lab text
        // next - beta
        const glm::vec3& nextPos = mVerts.at(next).pos;
        const glm::vec3& vj = mVerts.at(curr).pos;

        // Compute angle and area
        angleSum += acos(glm::dot(vj - vi, nextPos - vi) /
                         (glm::length(vj - vi) * glm::length(nextPos - vi)));
        area += glm::length(glm::cross(vi - vj, nextPos - vj)) * 0.5f;
    }

     return (2.f * static_cast<float>(M_PI) - angleSum) / area;

    // ---------------------------- End of Gauss Curvature ------------------------------ /*/

    // -------------------------------- Mean Curvature ----------------------------------- //
    // Inspired by code for Gauss Curvature (above) but using descrete Eq. 9 with function Cotangent
    // to get Mean

    //// Get the 1-ring of the vertex
    std::vector<size_t> oneRing = FindNeighborVertices(vertexIndex);
    assert(!oneRing.empty());  // Ensure the 1-ring exists

    // Get vertex position indices
    size_t curr, next, prevVertex;
    const glm::vec3& vi = mVerts.at(vertexIndex).pos;

    glm::vec3 meanCurvatureSum(0.f);
    float vorArea = 0.f;

    for (size_t i = 0; i < oneRing.size(); i++) {

        // New things begin here...

        // Ignore - only for docs

        //// Get current and next vertex in the 1-ring
        // curr = oneRing.at(i);
        // next = (i < oneRing.size() - 1) ? oneRing.at(i + 1) : oneRing.front();  // Same as above
        // but more concise (check if at last vertex in 1-ring)

        //// Defining prev so we can get vh
        //// Choose a starting half-edge that originates from the current vertex
        // std::size_t start_edge = v(curr).edge;
        //// Temp edge that iterates over all edges in the 1-ring of vertex vertexIndex
        // std::size_t edge_iterator = start_edge;
        // edge_iterator = e(e(e(edge_iterator).prev).pair).prev;
        // prevVertex = e(edge_iterator).vert;

        // Start with the current vertex in the 1-ring neighborhood
        curr = oneRing[i];
        // Get the next vertex in the 1-ring neighborhood
        next = (i < oneRing.size() - 1) ? oneRing[i + 1] : oneRing.front();
        // Get the previous vertex in the 1-ring neighborhood
        prevVertex = (i > 0) ? oneRing[i - 1] : oneRing.back();

        // current second (the pair) next

        // Find vertices in 1-ring according to figure 5 in lab text
        const glm::vec3& vj = mVerts.at(curr).pos;  // vj is current vertex in 1-ring neighborhood
        const glm::vec3& vk = mVerts.at(next).pos;  // vk is next vertex(since k comes after j...)
        const glm::vec3& vh =
            mVerts.at(prevVertex).pos;  // vk is next vertex(since h comes before i...)

        // Compute cotangent weights, Ask:What is a,b & c in Cotangent and is cotAlpha supoosed to
        // be "in another corner"?
        float cotAlpha = Cotangent(vi, vh, vj);  // Maybe need to change dephending on what a, b and
                                                 // c is...also we may need vh (prev)??
        float cotBeta = Cotangent(vj, vk, vi);

        // Compute weighted sum
        meanCurvatureSum += (cotAlpha + cotBeta) * (vi - vj);  // Eq. 9

        // Compute squared edge length for Voronoi area
        float edgeLengthSq_ij = glm::dot(vi - vj, vi - vj);  // Dot product = ^2

        // Compute local Voronoi area (simplified Eq. 10)
        vorArea += (edgeLengthSq_ij * cotBeta + edgeLengthSq_ij * cotAlpha);
    }

    // Compute and return mean curvature
    glm::vec3 meanCurvature = meanCurvatureSum / (0.5f * vorArea);
    return glm::length(meanCurvature);

    // comment:
    /*
     * Big sphere -> smaller curvature we got [0.9, 1.0] for sphere 1.0
     * Small sphere -> larger curvature because more aggressive changes on surafce we got
     * [9.9, 10.0]   literally x10!!!
     */

    // ---------------------------- End of Mean Curvature ------------------------------ /*/
}

float HalfEdgeMesh::FaceCurvature(size_t faceIndex) const {
    // NB Assumes vertex curvature already computed
    size_t indx = f(faceIndex).edge;
    const EdgeIterator it = GetEdgeIterator(indx);

    const auto& v1 = v(it.GetEdgeVertexIndex());
    const auto& v2 = v(it.Next().GetEdgeVertexIndex());
    const auto& v3 = v(it.Next().GetEdgeVertexIndex());

    return (v1.curvature + v2.curvature + v3.curvature) / 3.f;
}

glm::vec3 HalfEdgeMesh::FaceNormal(size_t faceIndex) const {
    size_t indx = f(faceIndex).edge;
    const EdgeIterator it = GetEdgeIterator(indx);

    const auto& p1 = v(it.GetEdgeVertexIndex()).pos;
    const auto& p2 = v(it.Next().GetEdgeVertexIndex()).pos;
    const auto& p3 = v(it.Next().GetEdgeVertexIndex()).pos;

    const auto e1 = p2 - p1;
    const auto e2 = p3 - p1;
    return glm::normalize(glm::cross(e1, e2));
}

// Calcuating per vertex normals using Eq.5 from instructions
glm::vec3 HalfEdgeMesh::VertexNormal(size_t vertexIndex) const {
    glm::vec3 n(0.f, 0.f, 0.f);

    // Average face normals for each face touching the vertex in the 1-ring, using MWE mean weighted
    // equally
    std::vector<size_t> neighborFaces = FindNeighborFaces(vertexIndex);  // Locate neighboring faces

    // Get every face normal in the 1-ring of vertex vertexIndex
    for (std::size_t i = 0; i < neighborFaces.size(); i++) {
        const Face& triangle = f(neighborFaces.at(i));
        n += triangle.normal;
    }

    // Normalize the sum
    n = glm::normalize(n);

    return n;
}

void HalfEdgeMesh::Initialize() {
    Validate();
    Update();
}

void HalfEdgeMesh::Update() {
    // Calculate and store all differentials and area

    // First update all face normals and triangle areas
    for (size_t i = 0; i < GetNumFaces(); i++) {
        f(i).normal = FaceNormal(i);
    }
    // Then update all vertex normals and curvature
    for (size_t i = 0; i < GetNumVerts(); i++) {
        // Vertex normals are just weighted averages
        mVerts.at(i).normal = VertexNormal(i);
    }

    // Then update vertex curvature
    for (size_t i = 0; i < GetNumVerts(); i++) {
        mVerts.at(i).curvature = VertexCurvature(i);
        //    std::cerr <<   mVerts.at(i).curvature << "\n";
    }

    // Finally update face curvature
    for (size_t i = 0; i < GetNumFaces(); i++) {
        f(i).curvature = FaceCurvature(i);
    }

    std::cerr << "Area: " << Area() << ".\n";
    std::cerr << "Volume: " << Volume() << ".\n";

    // Update vertex and face colors
    if (!mColorMap) {
        return;
    }

    float minCurvature = std::numeric_limits<float>::max();
    float maxCurvature = -std::numeric_limits<float>::max();

    if (!mAutoMinMax) {
        minCurvature = mMinCMap;
        maxCurvature = mMaxCMap;
    }

    if (mVisualizationMode == CurvatureVertex) {
        if (!mAutoMinMax) {
            std::cerr << "Mapping color based on vertex curvature with range [" << mMinCMap << ","
                      << mMaxCMap << "]" << std::endl;
        } else {
            // Compute range from vertices
            for (auto& vert : mVerts) {
                if (minCurvature > vert.curvature) minCurvature = vert.curvature;
                if (maxCurvature < vert.curvature) maxCurvature = vert.curvature;
            }
            std::cerr << "Automatic mapping of color based on vertex curvature with range ["
                      << minCurvature << "," << maxCurvature << "]" << std::endl;
            mMinCMap = minCurvature;
            mMaxCMap = maxCurvature;
        }
        for (auto& vert : mVerts) {
            vert.color = mColorMap->Map(vert.curvature, minCurvature, maxCurvature);
        }
    } else if (mVisualizationMode == CurvatureFace) {
        if (!mAutoMinMax) {
            std::cerr << "Mapping color based on face curvature with range [" << mMinCMap << ","
                      << mMaxCMap << "]" << std::endl;
        } else {
            // Compute range from faces
            for (auto& face : mFaces) {
                if (minCurvature > face.curvature) minCurvature = face.curvature;
                if (maxCurvature < face.curvature) maxCurvature = face.curvature;
            }
            std::cerr << "Automatic mapping of color based on face curvature with range ["
                      << minCurvature << "," << maxCurvature << "]" << std::endl;
            mMinCMap = minCurvature;
            mMaxCMap = maxCurvature;
        }
        for (auto& face : mFaces) {
            face.color = mColorMap->Map(face.curvature, minCurvature, maxCurvature);
        }
    }
}

/*! \lab1 Implement the area */
float HalfEdgeMesh::Area() const {
    float area = 0;

    // Compute the area of the mesh --> sum of all face areas as per Eq.11 in instructions

    // Iterate all faces in the mesh
    for (std::size_t i = 0; i < GetNumFaces(); i++) {

        // Get the vertices of face i
        const Face& face = f(i);

        // Use the iterator for the edges of current face i
        const EdgeIterator edge_iterator = GetEdgeIterator(face.edge);

        // Get the vertices of the face
        const glm::vec3& v1 =
            v(edge_iterator.GetEdgeVertexIndex()).pos;  // Get the vertex position of the first edge
        const glm::vec3& v2 = v(edge_iterator.Next().GetEdgeVertexIndex())
                                  .pos;  // Second edge located at the next edge
        const glm::vec3& v3 = v(edge_iterator.Next().GetEdgeVertexIndex())
                                  .pos;  // Third edge located at the previous edge

        // Compute the area of the face, which is the cross product of two edges divided by 2 when
        // assuming CCW
        area += 0.5f * glm::length(glm::cross(v2 - v1, v3 - v1));
    }

    return area;
}

/*! \lab1 Implement the volume */
float HalfEdgeMesh::Volume() const {
    float volume = 0;

    // The volume of a mesh is given by 3V = sum(f_i * n_i * a_i), where f = face vertices, n = unit
    // face normal, a = face area, as per Eq. 15 in instructions

    // Iterate all faces in the mesh
    for (std::size_t i = 0; i < GetNumFaces(); i++) {

        // Get current face
        const Face& face = f(i);

        // Use the iterator for the edges of current face
        const EdgeIterator edge_iterator = GetEdgeIterator(face.edge);

        // Get the vertices of the current face
        const glm::vec3& v1 = v(edge_iterator.GetEdgeVertexIndex()).pos;
        const glm::vec3& v2 = v(edge_iterator.Next().GetEdgeVertexIndex()).pos;
        const glm::vec3& v3 = v(edge_iterator.Next().GetEdgeVertexIndex()).pos;

        // Compute the area of the face
        const float area = 0.5f * glm::length(glm::cross(v2 - v1, v3 - v1));

        // Compute the normal of the face
        /*const glm::vec3 n = glm::normalize(glm::cross(v2 - v1, v3 - v1));*/

        // Compute the volume of the face
        /* volume += area * glm::dot(v1, n) / 3.f;*/
        volume += glm::dot(((v1 + v2 + v3) / 3.0f), face.normal) * area;
    }

    return volume / 3.f;
}

/*! \lab1 Calculate the number of shells  */
size_t HalfEdgeMesh::Shells() const { return 1; }

/*! \lab1 Implement the genus */
size_t HalfEdgeMesh::Genus() const {
    // Add code here

    // According to assumption: L=F --> Eq 2: V - E + F - 2(1 - G) = 0
    std::size_t V = mVerts.size();
    std::cout << "Number of vertices: " << V << std::endl;
    std::size_t E = mEdges.size() / 2;
    std::cout << "Number of edges: " << E << std::endl;
    std::size_t F = mFaces.size();
    std::cout << "Number of faces: " << F << std::endl;

    std::size_t S = Shells();

    std::size_t G;

    /*G = (V - E + F - 2) / 2;*/
    G = (E + 2 - V - F) / 2;

    return G;
}

void HalfEdgeMesh::Dilate(float amount) {
    for (Vertex& v : mVerts) {
        v.pos += amount * v.normal;
    }
    Initialize();
    Update();
}

void HalfEdgeMesh::Erode(float amount) {
    for (Vertex& v : mVerts) {
        v.pos -= amount * v.normal;
    }
    Initialize();
    Update();
}

void HalfEdgeMesh::Smooth(float amount) {
    for (Vertex& v : mVerts) {
        v.pos -= amount * v.normal * v.curvature;
    }
    Initialize();
    Update();
}

void HalfEdgeMesh::Render() {
    glEnable(GL_LIGHTING);
    glMatrixMode(GL_MODELVIEW);

    // Apply transform
    glPushMatrix();  // Push modelview matrix onto stack

    // Convert transform-matrix to format matching GL matrix format
    // Load transform into modelview matrix
    glMultMatrixf(glm::value_ptr(mTransform));

    if (mWireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }

    // Draw geometry
    glBegin(GL_TRIANGLES);
    const auto numTriangles = GetNumFaces();
    for (size_t i = 0; i < numTriangles; i++) {
        auto& face = f(i);

        auto* edge = &e(face.edge);

        auto& v1 = v(edge->vert);
        edge = &e(edge->next);

        auto& v2 = v(edge->vert);
        edge = &e(edge->next);

        auto& v3 = v(edge->vert);

        if (mVisualizationMode == CurvatureVertex) {
            glColor3fv(glm::value_ptr(v1.color));
            glNormal3fv(glm::value_ptr(v1.normal));
            glVertex3fv(glm::value_ptr(v1.pos));

            glColor3fv(glm::value_ptr(v2.color));
            glNormal3fv(glm::value_ptr(v2.normal));
            glVertex3fv(glm::value_ptr(v2.pos));

            glColor3fv(glm::value_ptr(v3.color));
            glNormal3fv(glm::value_ptr(v3.normal));
            glVertex3fv(glm::value_ptr(v3.pos));
        } else {
            glColor3fv(glm::value_ptr(face.color));
            glNormal3fv(glm::value_ptr(face.normal));

            glVertex3fv(glm::value_ptr(v1.pos));
            glVertex3fv(glm::value_ptr(v2.pos));
            glVertex3fv(glm::value_ptr(v3.pos));
        }
    }
    glEnd();

    // Mesh normals by courtesy of Richard Khoury
    if (mShowNormals) {
        glDisable(GL_LIGHTING);
        glBegin(GL_LINES);
        const auto numTriangles = GetNumFaces();
        for (size_t i = 0; i < numTriangles; i++) {
            auto& face = f(i);

            auto* edge = &e(face.edge);

            auto& v1 = v(edge->vert);
            edge = &e(edge->next);

            auto& v2 = v(edge->vert);
            edge = &e(edge->next);

            auto& v3 = v(edge->vert);

            auto faceStart = (v1.pos + v2.pos + v3.pos) / 3.f;
            auto faceEnd = faceStart + face.normal * 0.1f;

            glColor3f(1.f, 0.f, 0.f);  // Red for face normal
            glVertex3fv(glm::value_ptr(faceStart));
            glVertex3fv(glm::value_ptr(faceEnd));

            glColor3f(0.f, 1.f, 0.f);  // Vertex normals in Green
            glVertex3fv(glm::value_ptr(v1.pos));
            glVertex3fv(glm::value_ptr((v1.pos + v1.normal * 0.1f)));
            glVertex3fv(glm::value_ptr(v2.pos));
            glVertex3fv(glm::value_ptr((v2.pos + v2.normal * 0.1f)));
            glVertex3fv(glm::value_ptr(v3.pos));
            glVertex3fv(glm::value_ptr((v3.pos + v3.normal * 0.1f)));
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }

    if (mWireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Restore modelview matrix
    glPopMatrix();

    GLObject::Render();
}
