#pragma once

#include "Levelset/LevelSetOperator.h"
#include "Math/Function3D.h"

/*! \brief A level set operator that does external advection
 *
 * This class implements level set advectionr in an external vector field by the
 * PDE
 *
 *  \f$
 *  \dfrac{\partial \phi}{\partial t} + \mathbf{V}(\mathbf{x})\cdot \nabla \phi
 * = 0 \f$
 */
//! \lab4 Implement advection in external vector field
class OperatorAdvect : public LevelSetOperator {
protected:
    Function3D<glm::vec3>* mVectorField;

public:
    OperatorAdvect(LevelSet* LS, Function3D<glm::vec3>* vf)
        : LevelSetOperator(LS), mVectorField(vf) {}

    virtual float ComputeTimestep() {
        // Compute and return a stable timestep
        // (Hint: Function3D::GetMaxValue())

        // Get the max of the vector field - this gives the vector field direction
        glm::vec3 v = glm::abs(mVectorField->GetMaxValue());
        float maxV = glm::max(glm::max(v.x, v.y), v.z);

        // The timestep still limited by CFL in eq. 12 --> 0.9 to ensure strictly smaller than 
        return (mLS->GetDx() / maxV)*0.9f;

        /*  return 1.f;*/
    }

    virtual void Propagate(float time) {
        // Determine timestep for stability
        float dt = ComputeTimestep();

        // Propagate level set with stable timestep dt
        // until requested time is reached
        for (float elapsed = 0.f; elapsed < time;) {
            if (dt > time - elapsed) {
                dt = time - elapsed;
            }
            elapsed += dt;

            IntegrateEuler(dt);
            // IntegrateRungeKutta(dt);
        }
    }

    virtual float Evaluate(size_t i, size_t j, size_t k) {
        // Compute the rate of change (dphi/dt)

        // Remember that the point (i,j,k) is given in grid coordinates, while
        // the velocity field used for advection needs to be sampled in
        // world coordinates (x,y,z). You can use LevelSet::TransformGridToWorld()
        // for this task.

        // Convert grid coordinates to world coordinates
        float x = i;
        float y = j;
        float z = k;
        mLS->TransformGridToWorld(x, y, z);

        // Get the vector field at point (x,y,z)
        glm::vec3 v = mVectorField->GetValue(x, y, z);

        // Compute the gradient of the level set function at point (x,y,z)
        glm::vec3 gradient;

        // Use upwind scheme to know how to differentiate for the gradient of the levelset
        if (v.x < 0.f) {
            gradient.x = mLS->DiffXp(i, j, k);
        } else {
            gradient.x = mLS->DiffXm(i, j, k);
        }
        if (v.y < 0.f) {
            gradient.y = mLS->DiffYp(i, j, k);
        } else {
            gradient.y = mLS->DiffYm(i, j, k);
        }
        if (v.z < 0.f) {
            gradient.z = mLS->DiffZp(i, j, k);
        } else {
            gradient.z = mLS->DiffZm(i, j, k);
        }


        // Calculate the adversion equation --> Eq. 9a --> dphi/dt = -v * grad(phi)
        return -1.0f * glm::dot(v, gradient); 
        
        // return 0.f;
    }
   
};