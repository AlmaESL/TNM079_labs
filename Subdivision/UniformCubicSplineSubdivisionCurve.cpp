#include <Subdivision/UniformCubicSplineSubdivisionCurve.h>
#include <glm.hpp>
#include <gtc/type_ptr.hpp>

UniformCubicSplineSubdivisionCurve::UniformCubicSplineSubdivisionCurve(
    const std::vector<glm::vec3>& joints, glm::vec3 lineColor, float lineWidth)
    : mCoefficients(joints), mControlPolygon(joints) {
    this->mLineColor = lineColor;
    this->mLineWidth = lineWidth;
}

void UniformCubicSplineSubdivisionCurve::Subdivide() {
    // Allocate space for new coefficients
    std::vector<glm::vec3> newc;

    assert(mCoefficients.size() > 4 && "Need at least 5 points to subdivide");

    // Implement the subdivision scheme for a natural cubic spline here

    // Loop over the coefficients and compute the new coefficients for the new points - Eq, 29-30
    for (size_t i = 0; i < mCoefficients.size(); i++) {
        
        // First endpoint -> c'0 = c0 and c'(1/2) = 1/8(4ci + 4c(i+1))
        if (i == 0) {
            newc.push_back(mCoefficients[i]); 
            newc.push_back((1.0f / 8.0f) * (4.0f * mCoefficients[i] + 4.0f * mCoefficients[i + 1]));
        } 
        // Second endpoint -> c'end = cend
        else if (i == mCoefficients.size()-1) {
            newc.push_back(mCoefficients[i]);

         // All other points -> c'i = (1/8)(c(i-1) + 6ci + c(i+1)), c'(1/2) = 1/8(4ci + 4c(i+1))
        } else {
            newc.push_back((1.0f / 8.0f) *
                           (mCoefficients[i - 1] + 6.0f * mCoefficients[i] + mCoefficients[i + 1]));
            newc.push_back((1.0f / 8.0f) * (4.0f * mCoefficients[i] + 4.0f * mCoefficients[i + 1]));
        }
    }

    // If 'mCoefficients' had size N, how large should 'newc' be? Perform a check
    // here!
    // The new coefficients should be 2*N - 1 
    assert(newc.size() == mCoefficients.size()*2-1 && "Incorrect number of new coefficients!");


    mCoefficients = newc;
}

void UniformCubicSplineSubdivisionCurve::Render() {
    // Apply transform
    glPushMatrix();  // Push modelview matrix onto stack

    // Convert transform-matrix to format matching GL matrix format
    // Load transform into modelview matrix
    glMultMatrixf(glm::value_ptr(mTransform));

    mControlPolygon.Render();

    // save line point and color states
    glPushAttrib(GL_POINT_BIT | GL_LINE_BIT | GL_CURRENT_BIT);

    // draw segments
    glLineWidth(mLineWidth);
    glColor3fv(glm::value_ptr(mLineColor));
    glBegin(GL_LINE_STRIP);
    // just draw the spline as a series of connected linear segments
    for (size_t i = 0; i < mCoefficients.size(); i++) {
        glVertex3fv(glm::value_ptr(mCoefficients.at(i)));
    }
    glEnd();

    // restore attribs
    glPopAttrib();

    glPopMatrix();

    GLObject::Render();
}
