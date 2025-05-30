#include <GUI/FluidVoxelCutPlane.h>
#include <gtc/type_ptr.hpp>

FluidVoxelCutPlane::FluidVoxelCutPlane(const std::string& name, const FluidSolver* solver)
    : mSolver(solver) {
    Bbox box = mSolver->GetBoundingBox();
    /*mDx = (box.pMax[0] - box.pMin[0]) / (float)mSolver->mVoxels.GetDimX();*/
    mDx = mSolver->mDx;

    SetName(name);
    Update();
}

void FluidVoxelCutPlane::SetTransform(const glm::mat4& transform) {
    Geometry::SetTransform(transform);
    Update();
}

void FluidVoxelCutPlane::Render() {
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

    glPointSize(5);
    glColor3f(1.f, 0.f, 0.f);
    glBegin(GL_QUADS);

    int i = 0;
    for (float x = -1; x <= 1; x += mDx) {
        for (float y = -1; y <= 1; y += mDx) {
            bool fluid = mFluidVoxels[i];

            if (fluid) {
                glColor4f(0.f, 0.f, 1.f, mOpacity);
            } else {
                glColor4f(1.f, 1.f, 1.f, mOpacity);
            }

            glm::vec3 vec = mFluidVoxelsPositions[i];

            glVertex3fv(glm::value_ptr(vec));  // bottom-left
            glVertex3f(vec[0] + mDx, vec[1], vec[2]);
            glVertex3f(vec[0] + mDx, vec[1] + mDx, vec[2]);
            glVertex3f(vec[0], vec[1] + mDx, vec[2]);

            ++i;
        }
    }

    glEnd();
    glPointSize(1.f);

    float lineOpacity = std::min(mOpacity * 1.5f, 1.f);
    glColor4f(1.f, 1.f, 0.f, lineOpacity);
    glLineWidth(3.f);

    i = 0;
    for (float x = -1; x <= 1; x += mDx) {
        for (float y = -1; y <= 1; y += mDx) {
            glBegin(GL_LINE_LOOP);

            glm::vec3 vec = mFluidVoxelsPositions[i];

            glVertex3fv(glm::value_ptr(vec));  // bottom-left
            glVertex3f(vec[0] + mDx, vec[1], vec[2]);
            glVertex3f(vec[0] + mDx, vec[1] + mDx, vec[2]);
            glVertex3f(vec[0], vec[1] + mDx, vec[2]);

            glEnd();

            ++i;
        }
    }

    glPopAttrib();

    GLObject::Render();
}

void FluidVoxelCutPlane::Update() {
    std::cerr << "Building vector cut plane of size " << 2 / mDx << "x" << 2 / mDx << std::endl;
    mFluidVoxels.clear();
    mFluidVoxelsPositions.clear();

    Bbox box = mSolver->GetBoundingBox();

    for (float x = -1; x <= 1; x += mDx) {
        for (float y = -1; y <= 1; y += mDx) {
            glm::vec4 vec(x, y, 0.f, 1.f);
            vec = mTransform * vec;

            int i = (int)((vec[0] - box.pMin[0]) / mDx);
            int j = (int)((vec[1] - box.pMin[1]) / mDx);
            int k = (int)((vec[2] - box.pMin[2]) / mDx);

            mFluidVoxels.push_back(mSolver->IsFluid(i, j, k));
            mFluidVoxelsPositions.push_back(glm::vec3(vec[0], vec[1], vec[2]));
        }
    }
}
