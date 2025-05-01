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
#include <Geometry/Quadric.h>

Quadric::Quadric(const glm::mat4& q) : mQuadric(q) {}

Quadric::~Quadric() {}

/*!
 * Evaluation of world coordinates are done through either transformation
 * of the world-coordinates by mWorld2Obj, or transformation of the quadric
 * coefficient matrix by GetTransform() ONCE (see Section 2.2 in lab text).
 */
float Quadric::GetValue(float x, float y, float z) const {

    // Transform th point from local to world space coordinate system
    TransformW2O(x, y, z);

    // Add a 1 for the homogeneous coordinate
    glm::vec4 p(x, y, z, 1.0f);

    // Compute the transformed point p according to eq. 21 --> mQuadric contains the coefficients of the function 
    // of an implicit quadric surface
    return glm::dot(p, mQuadric*p); 

    /*return 0.f;*/
}

/*!
 * Use the quadric matrix to evaluate the gradient.
 */
glm::vec3 Quadric::GetGradient(float x, float y, float z) const {

    // Transform point from local to world space coordinate system
    TransformW2O(x, y, z);

    // Add a 1 for the homogeneous coordinate
    glm::vec4 p(x, y, z, 1.0f);

    // Compute matrix Q_sub as a 3x4 matrix from mQuadric, eq 22 --> mQuadric contains the coefficients of the function 
    // of an implicit quadric surface
    glm::mat3x4 Q_sub = glm::mat3x4(mQuadric);

    // Following eq. 22, the gradient is given by 2(Q_sub * p)
    return glm::vec3(Q_sub * p) * 2.0f;

  /*  return glm::vec3(0.f, 0.f, 0.f);*/
}
