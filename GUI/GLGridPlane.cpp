
#include "GLGridPlane.h"

void GLGridPlane::Render() {
    glPushMatrix();

    glDisable(GL_LIGHTING);

    glBegin(GL_LINES);
    glColor4f(1.f, 1.f, 1.f, 0.2f);
    GLfloat spanX(mDimensions[0] / 2.f);
    GLfloat spanY(mDimensions[1] / 2.f);
    for (GLfloat x = -spanX; x < spanX + mDensity * 0.5f; x += mDensity) {
        glVertex2f(x, -spanY);
        glVertex2f(x, spanY);
    }
    for (GLfloat y = -spanY; y < spanY + mDensity * 0.5f; y += mDensity) {
        glVertex2f(-spanX, y);
        glVertex2f(spanX, y);
    }
    glEnd();

    glPopMatrix();

    GLObject::Render();
}
