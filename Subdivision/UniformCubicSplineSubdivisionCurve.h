/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 *   Gunnar Johansson (gunnar.johansson@itn.liu.se)
 *   Ken Museth (ken.museth@itn.liu.se)
 *   Michael Bang Nielsen (bang@daimi.au.dk)
 *   Ola Nilsson (ola.nilsson@itn.liu.se)
 *   Andreas Sˆderstrˆm (andreas.soderstrom@itn.liu.se)
 *
 *************************************************************************************************/
#ifndef __uc_spline_subdiv_h__
#define __uc_spline_subdiv_h__

#include "Geometry/Geometry.h"
#include "Geometry/LineStrip.h"
#include "Subdivision.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

class UniformCubicSplineSubdivisionCurve : public Subdivision, public Geometry {
public:
    UniformCubicSplineSubdivisionCurve(const std::vector<glm::vec3>& joints,
                                       glm::vec3 lineColor = glm::vec3(0.f, 1.f, 0.2f),
                                       float lineWidth = 2.f);

    virtual void Update() {}
    virtual void Initialize() {}

    virtual void Subdivide();

    virtual void Render();

    virtual const char* GetTypeName() { return typeid(UniformCubicSplineSubdivisionCurve).name(); }

protected:
    //! The coefficients dictating the shape
    std::vector<glm::vec3> mCoefficients;
    //! The control polygon is simply a LineStrip
    LineStrip mControlPolygon;

    // display information
    glm::vec3 mLineColor;
    float mLineWidth;
};
#endif
