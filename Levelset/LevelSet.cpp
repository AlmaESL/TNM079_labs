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

#include <Util/Util.h>
#include <gtx/string_cast.hpp>
#include <Levelset/LevelSet.h>

const LevelSet::VisualizationMode LevelSet::NarrowBand = NewVisualizationMode("Narrowband");

LevelSet::LevelSet(float dx) : mDx(dx) {}

LevelSet::LevelSet(float dx, const Implicit& impl) : mDx(dx) {
    // Get the bounding box (in world space) from the implicit
    // function to initialize the level set's bounding box.
    Bbox b = impl.GetBoundingBox();
    SetBoundingBox(b);

    // Loop over volume and sample the implicit function
    size_t i = 0, j = 0, k = 0;
    for (float x = mBox.pMin[0]; x < mBox.pMax[0] + 0.5f * mDx; x += mDx, i++) {
        for (float y = mBox.pMin[1]; y < mBox.pMax[1] + 0.5f * mDx; y += mDx, j++) {
            for (float z = mBox.pMin[2]; z < mBox.pMax[2] + 0.5f * mDx; z += mDx, k++) {
                mGrid.SetValue(i, j, k, impl.GetValue(x, y, z));
            }
            k = 0;
        }
        j = 0;
    }
}

/*! Assigns the level-set from a volume. Sets the dimensions of the bounding box
 * to match the dimensions of the grid scaled by dx
 */
LevelSet::LevelSet(float dx, const Volume<float>& vol) : mDx(dx) {
    // Get the bounding box (in world space) from the size of the grid
    Bbox b(glm::vec3(0, 0, 0), glm::vec3((vol.GetDimX() - 1) * mDx, (vol.GetDimY() - 1) * mDx,
                                         (vol.GetDimZ() - 1) * mDx));
    SetBoundingBox(b);

    // Loop over volume and copy the volume
    for (size_t i = 0; i < vol.GetDimX(); i++) {
        for (size_t j = 0; j < vol.GetDimY(); j++) {
            for (size_t k = 0; k < vol.GetDimZ(); k++) {
                mGrid.SetValue(i, j, k, vol.GetValue(i, j, k) * mDx);
            }
        }
    }
}

LevelSet::LevelSet(float dx, const Implicit& impl, const Bbox& box) : mDx(dx) {
    SetBoundingBox(box);

    // Loop over volume and sample the implicit function
    int i = 0, j = 0, k = 0;
    for (float x = mBox.pMin[0]; x < mBox.pMax[0] + 0.5f * mDx; x += mDx, i++) {
        for (float y = mBox.pMin[1]; y < mBox.pMax[1] + 0.5f * mDx; y += mDx, j++) {
            for (float z = mBox.pMin[2]; z < mBox.pMax[2] + 0.5f * mDx; z += mDx, k++) {
                mGrid.SetValue(i, j, k, impl.GetValue(x, y, z));
            }
            k = 0;
        }
        j = 0;
    }
}

float LevelSet::GetValue(float x, float y, float z) const {
    // We use the convention that world coordinates are represented by (x,y,z)
    // while grid coordinates are written as (i,j,k)
    TransformWorldToGrid(x, y, z);

    size_t i = static_cast<size_t>(x);
    size_t j = static_cast<size_t>(y);
    size_t k = static_cast<size_t>(z);

    float bx = x - static_cast<float>(i);
    float by = y - static_cast<float>(j);
    float bz = z - static_cast<float>(k);

    i = glm::clamp(i, size_t{0}, mGrid.GetDimX() - 1);
    j = glm::clamp(j, size_t{0}, mGrid.GetDimY() - 1);
    k = glm::clamp(k, size_t{0}, mGrid.GetDimZ() - 1);

    bx = glm::clamp(bx, 0.f, 1.f);
    by = glm::clamp(by, 0.f, 1.f);
    bz = glm::clamp(bz, 0.f, 1.f);

    if (i == mGrid.GetDimX() - 1) {
        i--;
        bx = 1;
    }
    if (j == mGrid.GetDimY() - 1) {
        j--;
        by = 1;
    }
    if (k == mGrid.GetDimZ() - 1) {
        k--;
        bz = 1;
    }

    float val = mGrid.GetValue(i, j, k) * (1 - bx) * (1 - by) * (1 - bz) +
                mGrid.GetValue(i + 1, j, k) * bx * (1 - by) * (1 - bz) +
                mGrid.GetValue(i + 1, j + 1, k) * bx * by * (1 - bz) +
                mGrid.GetValue(i, j + 1, k) * (1 - bx) * by * (1 - bz) +
                mGrid.GetValue(i, j, k + 1) * (1 - bx) * (1 - by) * bz +
                mGrid.GetValue(i + 1, j, k + 1) * bx * (1 - by) * bz +
                mGrid.GetValue(i + 1, j + 1, k + 1) * bx * by * bz +
                mGrid.GetValue(i, j + 1, k + 1) * (1 - bx) * by * bz;

    return val;
}

/*!
 * Evaluates gradient at (x,y,z) through discrete finite difference scheme.
 */
glm::vec3 LevelSet::GetGradient(float x, float y, float z) const {

    // First, transform coordinates to grid space
    //TransformWorldToGrid(x ,y ,z); 
    //
    // Gradient vector of level set function is defined as the deriative in each direction
    //return glm::vec3(DiffXpm(x, y, z), DiffYpm(x, y, z), DiffZpm(x, y, z));

   /* return Implicit::GetGradient(x, y, z);*/

    TransformWorldToGrid(x, y, z);

    float outI = glm::round(x);
    float outJ = glm::round(y);
    float outK = glm::round(z);

    size_t i = static_cast<size_t>(outI);
    size_t j = static_cast<size_t>(outJ);
    size_t k = static_cast<size_t>(outK);

    float fx = DiffXpm(i, j, k);
    float fy = DiffYpm(i, j, k);
    float fz = DiffZpm(i, j, k);

    return glm::vec3(fx, fy, fz);
}

/*!
 * Evaluates curvature at (x,y,z) through discrete finite difference scheme.
 */
float LevelSet::GetCurvature(float x, float y, float z) const {
    return Implicit::GetCurvature(x, y, z);
}

void LevelSet::SetBoundingBox(const Bbox& b) {
    // Loop over existing grid to find the maximum and minimum values
    // stored. These are used to initialize the new grid with decent values.
    LevelSetGrid::Iterator iter = mGrid.BeginNarrowBand();
    LevelSetGrid::Iterator iend = mGrid.EndNarrowBand();
    float maxVal = -std::numeric_limits<float>::max();
    float minVal = std::numeric_limits<float>::max();
    while (iter != iend) {
        size_t i = iter.GetI();
        size_t j = iter.GetJ();
        size_t k = iter.GetK();

        float val = mGrid.GetValue(i, j, k);
        if (maxVal < val) maxVal = val;
        if (minVal > val) minVal = val;
        iter++;
    }

    // Create a new grid with requested size
    glm::vec3 extent = b.pMax - b.pMin;
    size_t dimX = (size_t)Round(extent[0] / mDx) + 1;
    size_t dimY = (size_t)Round(extent[1] / mDx) + 1;
    size_t dimZ = (size_t)Round(extent[2] / mDx) + 1;
    LevelSetGrid grid(dimX, dimY, dimZ, minVal, maxVal);

    // Copy all old values to new grid
    iter = mGrid.BeginNarrowBand();
    while (iter != iend) {
        size_t i = iter.GetI();
        size_t j = iter.GetJ();
        size_t k = iter.GetK();

        // Get the (x,y,z) coordinates of grid point (i,j,k)
        float x = i * mDx + mBox.pMin[0];
        float y = j * mDx + mBox.pMin[1];
        float z = k * mDx + mBox.pMin[2];

        // Check that (x,y,z) is inside the new bounding box
        if (x < b.pMin[0] || x > b.pMax[0] || y < b.pMin[1] || y > b.pMax[1] || z < b.pMin[2] ||
            z > b.pMax[2]) {
            iter++;
            continue;
        }

        // Compute the new grid point (l,m,n)
        size_t l = (size_t)Round((x - b.pMin[0]) / mDx);
        size_t m = (size_t)Round((y - b.pMin[1]) / mDx);
        size_t n = (size_t)Round((z - b.pMin[2]) / mDx);

        grid.SetValue(l, m, n, mGrid.GetValue(i, j, k));
        iter++;
    }

    // Set inside and outside constants
    grid.SetInsideConstant(mGrid.GetInsideConstant());
    grid.SetOutsideConstant(mGrid.GetOutsideConstant());

    // Set the new bounding box
    Implicit::SetBoundingBox(b);

    // Reassign the new grid
    mGrid = grid;

    std::cerr << "Level set created with grid size: " << glm::to_string(mGrid.GetDimensions())
              << std::endl;
}

void LevelSet::SetNarrowBandWidth(int width) {
    mGrid.SetInsideConstant(-width * 0.5f * mDx);
    mGrid.SetOutsideConstant(width * 0.5f * mDx);
    mGrid.Rebuild();
}

int LevelSet::GetNarrowBandWidth() const {
    float width = mGrid.GetOutsideConstant() - mGrid.GetInsideConstant();
    return static_cast<int>(width / mDx);
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::DiffXm(size_t i, size_t j, size_t k) const {

    // Xm = phi_{i,j,k} - phi_{i-1,j,k})/mDx --> Eq. 10 --> how much the level set moves in discrete
    // steps
    return (mGrid.GetValue(i, j, k) - mGrid.GetValue(i - 1, j, k)) / mDx;

    /* return 0; */
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::DiffXp(size_t i, size_t j, size_t k) const {

    // Xp = phi_{i+1,j,k} - phi_{i,j,k})/mDx --> Eq. 10
    return (mGrid.GetValue(i + 1, j, k) - mGrid.GetValue(i, j, k)) / mDx;

    /*return 0;*/
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::DiffXpm(size_t i, size_t j, size_t k) const {

    // Xpm = phi_{i+1,j,k} - phi_{i-1,j,k})/2mDx --> Eq. 14
    return (mGrid.GetValue(i + 1, j, k) - mGrid.GetValue(i - 1, j, k)) / (2.0f * mDx);

    /*return 0;*/
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::Diff2Xpm(size_t i, size_t j, size_t k) const {

    // 2Xpm = phi_{i+1,j,k} - 2phi_{i,j,k} + phi_{i-1,j,k})/mDx^2 --> Eq. 17a
    return (mGrid.GetValue(i + 1, j, k) - 2.0f * mGrid.GetValue(i, j, k) +
            mGrid.GetValue(i - 1, j, k)) /
           (mDx * mDx);

    /*return 0;*/
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::DiffYm(size_t i, size_t j, size_t k) const {

    // Ym = phi_{i,j,k} - phi_{i,j-1,k})/mDx <-- same as x but in y-direction
    return (mGrid.GetValue(i, j, k) - mGrid.GetValue(i, j - 1, k)) / mDx;

    /* return 0;*/
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::DiffYp(size_t i, size_t j, size_t k) const {

    // Yp = phi_{i,j+1,k} - phi_{i,j,k})/mDx <-- same as x but in y-direction
    return (mGrid.GetValue(i, j + 1, k) - mGrid.GetValue(i, j, k)) / mDx;

    /*return 0;*/
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::DiffYpm(size_t i, size_t j, size_t k) const {

    // Ypm = phi_{i,j+1,k} - phi_{i,j-1,k})/2mDx
    return (mGrid.GetValue(i, j + 1, k) - mGrid.GetValue(i, j - 1, k)) / (2.0f * mDx);

    /* return 0;*/
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::Diff2Ypm(size_t i, size_t j, size_t k) const {

    // 2Ypm = phi_{i,j+1,k}-2phi_{i,j,k}+phi_{i,j-1,k}/mDx^2
    return (mGrid.GetValue(i, j + 1, k) - 2.0f * mGrid.GetValue(i, j, k) +
            mGrid.GetValue(i, j - 1, k)) /
           (mDx * mDx);

    /*return 0;*/
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::DiffZm(size_t i, size_t j, size_t k) const {

    // Zm = phi_{i,j,k}-phi_{i,j,k-1}/mDx <-- same as x and y but in z-direction
    return (mGrid.GetValue(i, j, k) - mGrid.GetValue(i, j, k - 1)) / mDx;

    /* return 0;*/
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::DiffZp(size_t i, size_t j, size_t k) const {

    // Zp = phi_{i,j,k+1}-phi_{i,j,k}/mDx <-- same as x and y but in z-direction
    return (mGrid.GetValue(i, j, k + 1) - mGrid.GetValue(i, j, k)) / mDx;

    /*return 0;*/
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::DiffZpm(size_t i, size_t j, size_t k) const {

    // Zpm = phi_{i,j,k+1}-phi_{i,j,k-1}/2mDx
    return (mGrid.GetValue(i, j, k + 1) - mGrid.GetValue(i, j, k - 1)) / (2.0f * mDx);

    /*return 0;*/
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::Diff2Zpm(size_t i, size_t j, size_t k) const {

    // 2Zpm = phi_{i,j,k+1}-2phi_{i,j,k}+phi_{i,j,k-1}/mDx^2
    return (mGrid.GetValue(i, j, k + 1) - 2.0f * mGrid.GetValue(i, j, k) +
            mGrid.GetValue(i, j, k - 1)) /
           (mDx * mDx);

    /*return 0;*/
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::Diff2XYpm(size_t i, size_t j, size_t k) const {

    // 2XYpm = phi_{i+1,j+1,k}-phi_{i+1,j-1,k}+phi_{i-1,j-1,k}-phi_{i-1,j+1,k} / 4mDx^2 --> Eq 17b
    return (mGrid.GetValue(i + 1, j + 1, k) - mGrid.GetValue(i + 1, j - 1, k) +
            mGrid.GetValue(i - 1, j - 1, k) - mGrid.GetValue(i - 1, j + 1, k)) /
           (4.0f * (mDx * mDx));

    /*return 0;*/
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::Diff2YZpm(size_t i, size_t j, size_t k) const {

    return (mGrid.GetValue(i, j + 1, k + 1) - mGrid.GetValue(i, j + 1, k - 1) +
            mGrid.GetValue(i, j - 1, k - 1) - mGrid.GetValue(i, j - 1, k + 1)) /
           (4.0f * (mDx * mDx));

    /*return 0;*/
}

//! \lab4
/*! Use the values in the grid (mGrid.GetValue) to compute the differentials */
// By convention, we use (i,j,k) to represent grid coordinates, while (x,y,z)
// represents world coordinates.
float LevelSet::Diff2ZXpm(size_t i, size_t j, size_t k) const {

    return (mGrid.GetValue(i + 1, j, k + 1) - mGrid.GetValue(i + 1, j, k - 1) +
            mGrid.GetValue(i - 1, j, k - 1) - mGrid.GetValue(i - 1, j, k + 1)) /
           (4.0f * (mDx * mDx));

    /* return 0;*/
}

float LevelSet::WENO(float v1, float v2, float v3, float v4, float v5) const { return 0; }

void LevelSet::Render() {
    Bbox box = GetBoundingBox();
    glm::vec3 extent = box.pMax - box.pMin;
    float dx = extent[0] / mGrid.GetDimX();
    float dy = extent[1] / mGrid.GetDimY();
    float dz = extent[2] / mGrid.GetDimZ();

    if (mVisualizationMode == NarrowBand) {
        glPointSize(3.0);
        glDisable(GL_LIGHTING);
        glColor3f(1.0, 0.0, 0.0);
        glBegin(GL_POINTS);
        LevelSetGrid::Iterator iter = mGrid.BeginNarrowBand();
        LevelSetGrid::Iterator iend = mGrid.EndNarrowBand();
        while (iter != iend) {
            size_t i = iter.GetI();
            size_t j = iter.GetJ();
            size_t k = iter.GetK();

            float x = i * dx + box.pMin[0];
            float y = j * dy + box.pMin[1];
            float z = k * dz + box.pMin[2];

            glVertex3f(x, y, z);
            iter++;
        }
        glEnd();
        glPointSize(1.f);
    }

    Implicit::Render();
}

void LevelSet::TransformWorldToGrid(float& i, float& j, float& k) const {
    TransformW2O(i, j, k);
    i = (i - mBox.pMin[0]) / mDx;
    j = (j - mBox.pMin[1]) / mDx;
    k = (k - mBox.pMin[2]) / mDx;
}

void LevelSet::TransformGridToWorld(float& x, float& y, float& z) const {
    x = x * mDx + mBox.pMin[0];
    y = y * mDx + mBox.pMin[1];
    z = z * mDx + mBox.pMin[2];

    glm::vec4 v(x, y, z, 1);
    v = mTransform * v;
    x = v[0];
    y = v[1];
    z = v[2];
}
