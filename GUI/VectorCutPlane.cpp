/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 *   Gunnar Johansson (gunnar.johansson@itn.liu.se)
 *   Ken Museth (ken.museth@itn.liu.se)
 *   Michael Bang Nielsen (bang@daimi.au.dk)
 *   Ola Nilsson (ola.nilsson@itn.liu.se)
 *   Andreas Sderstrm (andreas.soderstrom@itn.liu.se)
 *
 *************************************************************************************************/

#include "VectorCutPlane.h"
#include <glm.hpp>
#include <gtc/type_ptr.hpp>

VectorCutPlane::VectorCutPlane(const std::string& name, float dx,
                               const Function3D<glm::vec3>* function)
    : mDx(dx), mFunction(function) {
    SetName(name);
    Update();
}

void VectorCutPlane::Render() {
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

    glLineWidth(1.f);
    glBegin(GL_LINES);
    size_t i = 0;
    float x = 0.f;
    for (float y = -1.f; y <= 1.f; y += mDx) {
        for (float z = -1.f; z <= 1.f; z += mDx) {
            glm::vec4 vec(x, y, z, 1.f);
            vec = mTransform * vec;

            glm::vec3 color = mColorMap->Map(mVectors[i], -1.f, 1.f);
            glColor3fv(glm::value_ptr(color));

            glm::vec3 end = glm::vec3(vec[0], vec[1], vec[2]) + mVectors[i] * mDx;
            glVertex3fv(glm::value_ptr(vec));
            glVertex3fv(glm::value_ptr(end));
            i++;
        }
    }
    glEnd();
    glLineWidth(1.f);

    glPointSize(3.f);
    glColor3f(1.f, 0.f, 0.f);
    glBegin(GL_POINTS);
    for (float y = -1.f; y <= 1.f; y += mDx) {
        for (float z = -1.f; z <= 1.f; z += mDx) {
            glm::vec4 vec(x, y, z, 1);
            vec = mTransform * vec;
            glVertex3fv(glm::value_ptr(vec));
        }
    }
    glEnd();
    glPointSize(1.f);

    glPopAttrib();

    GLObject::Render();
}

void VectorCutPlane::SetTransform(const glm::mat4& transform) {
    Geometry::SetTransform(transform);
    Update();
}

void VectorCutPlane::Update() {
    const float size = 2.f / mDx;
    std::cerr << "Building vector cut plane of size " << size << "x" << size << std::endl;
    mVectors.clear();
    float x = 0.f;
    for (float y = -1.f; y <= 1.f; y += mDx) {
        for (float z = -1.f; z <= 1.f; z += mDx) {
            glm::vec4 vec(x, y, z, 1.f);
            vec = mTransform * vec;
            mVectors.push_back(mFunction->GetValue(vec[0], vec[1], vec[2]));
        }
    }
}
