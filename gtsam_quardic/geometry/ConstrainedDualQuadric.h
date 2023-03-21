/* ----------------------------------------------------------------------------
This code defines a ConstrainedDualQuadric class in the gtsam_quadrics namespace.

A ConstrainedDualQuadric represents an ellipsoid using a rotation, translation, and the radii of its three axes.

The ellipsoid's pose is stored as a gtsam::Pose3, which contains the rotation and translation,
while the radii are stored as a gtsam::Vector3.

The code provides several constructors for initializing a ConstrainedDualQuadric object,
including a default constructor that initializes a unit sphere at the origin, and one that initializes from a 4x4 matrix.

It also provides methods for constructing a 4x4 quadric matrix from the pose and radii, and for normalizing the quadric.

Other methods include ones that calculate the ellipsoid's bounds and check if it contains a given point or if it is behind a given camera.

It also has methods for adding a quadric to a gtsam::Values object and for getting a quadric from a gtsam::Values object.

Finally, it has methods for printing the quadric and for comparing it with another ConstrainedDualQuadric object.

 * -------------------------------------------------------------------------- */
// 成员变量：
// gtsam::Pose3 pose_：代表椭球体的3D位姿
// gtsam::Vector3 radii_：代表椭球体三个轴的半径

// 构造函数：
// ConstrainedDualQuadric()：默认构造函数，生成原点处的单位球体
// ConstrainedDualQuadric(const gtsam::Matrix44& dQ)：从4x4矩阵构造，确保该二次曲面是椭球体
// ConstrainedDualQuadric(const gtsam::Pose3& pose, const gtsam::Vector3& radii)：从位姿和半径构造
// ConstrainedDualQuadric(const gtsam::Rot3& R, const gtsam::Point3& t, const gtsam::Vector3& r)：从旋转矩阵、平移向量和半径构造

// 类方法：
// gtsam::Matrix44 matrix(gtsam::OptionalJacobian<16, 9> H = boost::none) const：从位姿和半径构造4x4二次曲面矩阵
// gtsam::Matrix44 normalizedMatrix() const：返回归一化的二次曲面矩阵
// AlignedBox3 bounds() const：计算椭球体的轴对齐的边界
// bool isBehind(const gtsam::Pose3& cameraPose) const：如果椭球体的中心在相机的背面，返回true
// bool contains(const gtsam::Pose3& cameraPose) const：如果椭球体包含指定的点，则返回true
// static ConstrainedDualQuadric Retract(const gtsam::Vector9& v)：从切向空间的向量返回二次曲面
// static gtsam::Vector9 LocalCoordinates(const ConstrainedDualQuadric& q)：返回在原点处与二次曲面对应的切向空间的向量
// ConstrainedDualQuadric retract(const gtsam::Vector9& v) const：从当前二次曲面和切向空间的向量，返回在切向空间中的二次曲面
// gtsam::Vector9 localCoordinates(const ConstrainedDualQuadric& other) const：返回在切向空间中的两个二次曲面之间的向量

/**
 * @file ConstrainedDualQuadric.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a constrained dual quadric
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_quadrics/geometry/AlignedBox3.h>

#include <random>

namespace gtsam_quadrics
{

    /**
     * @class ConstrainedDualQuadric
     * A constrained dual quadric (r,t,s): see Nicholson et al. 2019 for details
     */
    class ConstrainedDualQuadric
    {
    protected:
        gtsam::Pose3 pose_;    ///< 3D pose of ellipsoid
        gtsam::Vector3 radii_; ///< radii of x,y,z axii

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// @name Constructors and named constructors
        /// @{

        /** default constructor, unit sphere at origin */
        ConstrainedDualQuadric();

        /**
         * Constructor from 4x4 Matrix,
         * Here we ensure the quadric is ellipsoidal
         * and constrain manually if it is not.
         * @param dQ
         */
        ConstrainedDualQuadric(const gtsam::Matrix44 &dQ);

        /**
         * Constructor pose and radii
         * @param pose quadric pose (Pose3)
         * @param radii quadric radii (Vector3)
         */
        ConstrainedDualQuadric(const gtsam::Pose3 &pose, const gtsam::Vector3 &radii)
            : pose_(pose), radii_(radii){};

        /**
         * Constructor from rotation, translation and shape
         * @param R quadric rotation (Rot3)
         * @param t quadric translation (Point3)
         * @param r quadric radii (Vector3)
         */
        ConstrainedDualQuadric(const gtsam::Rot3 &R, const gtsam::Point3 &t,
                               const gtsam::Vector3 &r)
            : pose_(gtsam::Pose3(R, t)), radii_(r){};

        /**
         * Constrains a generic dual quadric surface to be ellipsoidal
         * @param dual_quadric 4x4 symmetric matrix (Matrix4)
         */
        static ConstrainedDualQuadric constrain(const gtsam::Matrix4 &dual_quadric);

        /// @}
        /// @name Class accessors
        /// @{

        /** Get pose, avoid computation with it */
        gtsam::Pose3 pose(void) const { return pose_; }

        /** Get quadric radii, avoid computation with it */
        gtsam::Vector3 radii(void) const { return radii_; }

        /** Get quadric centroid */
        // 这个函数是用来获取Quadric的质心（centroid）的，它返回的是一个gtsam::Point3类型的对象，表示三维空间中的一个点。
        // 函数直接返回Quadric的位姿（pose_）的平移向量（translation），也就是Quadric的中心所在的位置。
        gtsam::Point3 centroid(void) const { return pose_.translation(); }

        /// @}
        /// @name Class methods
        /// @{

        /**
         * Constructs 4x4 quadric matrix from pose & radii
         * Q = Z * Qc * Z.T
         * Z = quadric pose in global frame
         * Qc = centered dualquadric diagonal matrix of shape (s1^2, s2^2, s3^2, -1)
         * where s1,s2,s3 are the radius of each axis on the ellipse
         * see Nicholson et. al 2019 QuadricSLAM for full details
         * @return 4x4 constrained quadric
         */
        gtsam::Matrix44 matrix(gtsam::OptionalJacobian<16, 9> H = boost::none) const;

        /** Returns the normalized dual quadric in matrix form */
        gtsam::Matrix44 normalizedMatrix(void) const;

        /**
         * Calculates the AlignedBox3 bounds of the ellipsoid
         * @return 3D axis aligned bounding box
         */
        AlignedBox3 bounds() const;

        /** Returns true if quadric centroid has negative depth */
        bool isBehind(const gtsam::Pose3 &cameraPose) const;

        /**
         * Returns true if quadric contains point
         * Points on the edge of the quadric are considered contained
         */
        bool contains(const gtsam::Pose3 &cameraPose) const;

        /// @}
        /// @name Manifold group traits
        /// @{
        enum
        {
            dimension = 9
        };

        /** The Retract at origin */
        static ConstrainedDualQuadric Retract(const gtsam::Vector9 &v);

        /** The Local at origin */
        static gtsam::Vector9 LocalCoordinates(const ConstrainedDualQuadric &q);

        /**
         * Moves from this by v in tangent space, then retracts back to a quadric
         * @param v displacement vector in tangent space
         * @return ConstrainedDualQuadric on the manifold
         */
        ConstrainedDualQuadric retract(const gtsam::Vector9 &v) const;

        /**
         * Calculates the distance in tanget space between two quadrics on the
         * manifold
         * @param other another ConstrainedDualQuadric
         * @return vector between ellipsoids in tangent space
         */
        gtsam::Vector9 localCoordinates(const ConstrainedDualQuadric &other) const;

        /** Add quadric to values */
        void addToValues(gtsam::Values &v, const gtsam::Key &k);

        /** Get Quadric from values */
        static ConstrainedDualQuadric getFromValues(const gtsam::Values &v,
                                                    const gtsam::Key &k);

        /// @}
        /// @name Testable group traits
        /// @{

        /** Prints the dual quadric with optional string */
        void print(const std::string &s = "") const;

        /** Compares two ConstrainedDualQuadrics */
        /// TODO: account for scaling by normalizing quadric
        bool equals(const ConstrainedDualQuadric &other, double tol = 1e-9) const;

        /// @}
    };

} // namespace gtsam_quadrics

/** \cond PRIVATE */
// Add ConstrainedDualQuadric to Manifold group
template <>
struct gtsam::traits<gtsam_quadrics::ConstrainedDualQuadric>
    : public gtsam::internal::Manifold<gtsam_quadrics::ConstrainedDualQuadric>
{
};

template <>
struct gtsam::traits<const gtsam_quadrics::ConstrainedDualQuadric>
    : public gtsam::internal::Manifold<gtsam_quadrics::ConstrainedDualQuadric>
{
};
/** \endcond */
