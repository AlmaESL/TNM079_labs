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
#pragma once

#include "Levelset/LevelSetOperator.h"

/*! \brief A level set operator that does mean curvature flow.
 *
 * This class implements level set propagation in the normal direction
 * as defined by the mean curvature flow \f$\kappa\f$ in the following PDE
 *
 *  \f[
 *  \dfrac{\partial \phi}{\partial t} + \alpha \kappa|\nabla \phi| = 0
 *  \f]
 */
//! \lab4 Implement mean curvature flow
class OperatorMeanCurvatureFlow : public LevelSetOperator {
protected:
    //! Scaling parameter, affects time step constraint
    float mAlpha;

public:
    OperatorMeanCurvatureFlow(LevelSet* LS, float alpha = 0.9f)
        : LevelSetOperator(LS), mAlpha(alpha) {}

    virtual float ComputeTimestep() {
        // Compute and return a stable timestep

        // Get the grid spacing
        float mdx = mLS->GetDx();

        // The timestep is limited by the CFL condition in Eq. 18 --> mdx^2 / 6alpha
        return ((mdx * mdx) / (6.f * mAlpha))*0.9f;  // alpha predefined in the constructor

        /*return 1.f;*/
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

        // Compute the gradient magnitude of phi |gradient(phi)|
         float dx = mLS->DiffXpm(i, j, k);
         float dy = mLS->DiffYpm(i, j, k);
         float dz = mLS->DiffZpm(i, j, k);
         float gradient_magnitude = std::sqrt(dx * dx + dy * dy + dz * dz);

        // Term 1
        float phi_yy = mLS->Diff2Ypm(i, j, k);
        float phi_zz = mLS->Diff2Zpm(i, j, k);
        float phi_y = mLS->DiffYpm(i, j, k);
        float phi_z = mLS->DiffZpm(i, j, k);
        float phi_yz = mLS->Diff2YZpm(i, j, k);
        float phi_z_squared = phi_z * phi_z;


        // Term 2
        float phi_y_squared = phi_y * phi_y; 
        float phi_xx = mLS->Diff2Xpm(i, j, k);
        float phi_x = mLS->DiffXpm(i, j, k);
        float phi_xz = mLS->Diff2ZXpm(i, j, k);
        float phi_x_squared = phi_x * phi_x;

        // Term 3
        float phi_xy = mLS->Diff2XYpm(i,j,k); 


        // Numerators and denominator
        float numerator_term1 = (phi_x_squared * (phi_yy + phi_zz)) - (2.0f*phi_y*phi_z*phi_yz);  
        float denominator = 2.0f * std::pow((phi_x_squared + phi_y_squared + phi_z_squared), (3.f/2.f));
        float term1 = numerator_term1 / denominator; 

        float numerator_term2 = (phi_y_squared * (phi_xx + phi_zz)) - (2.0f*phi_x*phi_z*phi_xz); 
        float term2 = numerator_term2 / denominator; 

        float numerator_term3 = (phi_z_squared * (phi_xx + phi_yy) - (2.f*phi_x*phi_y*phi_xy)); 
        float term3 = numerator_term3/denominator; 

        // Get Kappa 
        float kappa = term1 + term2 + term3; 

        // compute the rate of change --> Eq. 13
        return mAlpha * kappa * gradient_magnitude;

        /* return 0.f;*/
    }
};
