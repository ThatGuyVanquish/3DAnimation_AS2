#pragma once

#include <utility>
#include <Eigen/Core>
#include <Eigen/LU>
#include <igl/opengl/glfw/Viewer.h>
#include <iostream>
#include "../engine/Mesh.h"
#include "MeshSimplification.h"
#include <igl/AABB.h>

/*

	Helper functions for collision detection

*/

namespace CollisionDetection {
    static void fourColumnVertexMatrix(Eigen::MatrixXd &V) {
        V.conservativeResize(V.rows(), 4);
        for (int i = 0; i < V.rows(); i++)
            V(i, 3) = 1;
    }

    static Eigen::AlignedBox3d transformBox(
            Eigen::AlignedBox3d box,
            const Eigen::Matrix4d &transform
    ) {
        auto min = box.min(), max = box.max();
        Eigen::Vector4d newMin;
        newMin << min[0], min[1], min[2], 1;
        newMin = transform * newMin;
        Eigen::Vector4d newMax;
        newMax << max[0], max[1], max[2], 1;
        newMax = transform * newMax;
        return Eigen::AlignedBox<double, 3>(
                Eigen::Vector3d(newMin[0], newMin[1], newMin[2]),
                Eigen::Vector3d(newMax[0], newMax[1], newMax[2]));
    }

    static void calcBoxInSpace(const Eigen::AlignedBox3d &box,
                               const Eigen::Matrix4d &transform,
                               Eigen::Vector3d &C,
                               Eigen::Matrix3d &A,
                               Eigen::Vector3d &a) {
        // calculate C (center) in the global space
        Eigen::Vector4d Cvec4;
        Cvec4 << C[0], C[1], C[2], 1;
        Cvec4 = transform * Cvec4;
        C = Cvec4.head(3);
        // calculate A1, A2, A3 axes in the global space
        Eigen::Vector4d A1vec4 = transform * Eigen::Vector4d(1, 0, 0, 1);
        Eigen::Vector4d A2vec4 = transform * Eigen::Vector4d(0, 1, 0, 1);
        Eigen::Vector4d A3vec4 = transform * Eigen::Vector4d(0, 0, 1, 1);
        A.col(0) = A1vec4.head(3);
        A.col(1) = A2vec4.head(3);
        A.col(2) = A3vec4.head(3);

        // calculate a1, a2, a3 extents
        Eigen::Vector4d a_vec4;
        a_vec4 << box.sizes()[0], box.sizes()[1], box.sizes()[3], 1;
        a = (transform * a_vec4).head(3) / 2;
    }

    static bool intersects(
            const igl::AABB<Eigen::MatrixXd, 3> &obb1,
            const Eigen::Matrix4d &transform1,
            const igl::AABB<Eigen::MatrixXd, 3> &obb2,
            const Eigen::Matrix4d &transform2,
            Eigen::AlignedBox3d &collidedBox1,
            Eigen::AlignedBox3d &collidedBox2
    ) {
        std::cout << "blyat this is here\n";

        Eigen::Vector3d C1, a;
        Eigen::Matrix3d A;
        calcBoxInSpace(obb1.m_box, transform1, C1, A, a);
        Eigen::Vector3d C2, b;
        Eigen::Matrix3d B;
        calcBoxInSpace(obb1.m_box, transform1, C2, B, b);

        Eigen::Matrix3d C = A.transpose() * B;
        Eigen::Vector3d D = C2 - C1;

        int R0;
        int R1;
        int R;

        // L = A0
        R0 = a[0];
        R1 = b[0] * abs(C(0, 0)) + b[1] * abs(C(0, 1)) + b[2] * abs(C(0, 2));
        R = abs(A.col(0).transpose() * D);
        if (R > R0 + R1) return false;

        // L = A1
        R0 = a[1];
        R1 = b[0] * abs(C(1, 0)) + b[1] * abs(C(1, 1)) + b[2] * abs(C(1, 2));
        R = abs(A.col(1).transpose() * D);
        if (R > R0 + R1) return false;

        // L = A2
        R0 = a[2];
        R1 = b[0] * abs(C(2, 0)) + b[1] * abs(C(2, 1)) + b[2] * abs(C(2, 2));
        R = abs(A.col(2).transpose() * D);
        if (R > R0 + R1) return false;

        // L = B0
        R0 = a[0] * abs(C(0, 0)) + a[1] * abs(C(1, 0)) + a[2] * abs(C(2, 1));
        R1 = b[0];
        R = abs(B.col(0).transpose() * D);
        if (R > R0 + R1) return false;

        // L = B1
        R0 = a[0] * abs(C(0, 1)) + a[1] * abs(C(1, 1)) + a[2] * abs(C(2, 1));
        R1 = b[1];
        R = abs(B.col(1).transpose() * D);
        if (R > R0 + R1) return false;

        // L = B2
        R0 = a[0] * abs(C(0, 2)) + a[1] * abs(C(1, 2)) + a[2] * abs(C(2, 2));
        R1 = b[2];
        R = abs(B.col(2).transpose() * D);
        if (R > R0 + R1) return false;

        // L = A0 x B0
        R0 = a[1] * abs(C(2, 0)) + a[2] * abs(C(1, 0));
        R1 = b[1] * abs(C(0, 2)) + b[2] * abs(C(0, 1));
        int arg1 = C(1, 0) * A.col(2).transpose() * D;
        int arg2 = C(2, 0) * A.col(1).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A0 x B1
        R0 = a[1] * abs(C(2, 1)) + a[2] * abs(C(1, 1));
        R1 = b[1] * abs(C(0, 2)) + b[2] * abs(C(0, 0));
        arg1 = C(1, 1) * A.col(2).transpose() * D;
        arg2 = C(2, 1) * A.col(1).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A0 x B2
        R0 = a[1] * abs(C(2, 2)) + a[2] * abs(C(1, 2));
        R1 = b[1] * abs(C(0, 1)) + b[2] * abs(C(0, 0));
        arg1 = C(1, 2) * A.col(2).transpose() * D;
        arg2 = C(2, 2) * A.col(1).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A1 x B0
        R0 = a[0] * abs(C(2, 0)) + a[2] * abs(C(0, 0));
        R1 = b[1] * abs(C(1, 2)) + b[2] * abs(C(1, 1));
        arg1 = C(2, 0) * A.col(0).transpose() * D;
        arg2 = C(0, 0) * A.col(2).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A1 x B1
        R0 = a[0] * abs(C(2, 1)) + a[2] * abs(C(0, 1));
        R1 = b[0] * abs(C(1, 2)) + b[2] * abs(C(1, 0));
        arg1 = C(2, 1) * A.col(0).transpose() * D;
        arg2 = C(0, 1) * A.col(2).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A1 x B2
        R0 = a[0] * abs(C(2, 2)) + a[2] * abs(C(0, 2));
        R1 = b[0] * abs(C(1, 1)) + b[1] * abs(C(1, 0));
        arg1 = C(2, 2) * A.col(0).transpose() * D;
        arg2 = C(0, 2) * A.col(2).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A2 x B0
        R0 = a[0] * abs(C(1, 0)) + a[1] * abs(C(0, 0));
        R1 = b[1] * abs(C(2, 2)) + b[2] * abs(C(2, 1));
        arg1 = C(0, 0) * A.col(1).transpose() * D;
        arg2 = C(1, 0) * A.col(0).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A2 x B1
        R0 = a[0] * abs(C(1, 1)) + a[1] * abs(C(0, 1));
        R1 = b[0] * abs(C(2, 2)) + b[2] * abs(C(2, 0));
        arg1 = C(0, 1) * A.col(1).transpose() * D;
        arg2 = C(1, 1) * A.col(0).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        // L = A2 x B2
        R0 = a[0] * abs(C(1, 2)) + a[1] * abs(C(0, 2));
        R1 = b[0] * abs(C(2, 1)) + b[1] * abs(C(2, 0));
        arg1 = C(0, 2) * A.col(1).transpose() * D;
        arg2 = C(1, 2) * A.col(0).transpose() * D;
        R = abs(arg1 - arg2);
        if (R > R0 + R1) return false;

        if (obb1.is_leaf() && obb2.is_leaf()) {
            collidedBox1 = obb1.m_box;
            collidedBox2 = obb2.m_box;
            return true;
        }
        return
                intersects(*obb1.m_left, transform1, *obb2.m_left, transform2, collidedBox1, collidedBox2) ||
                intersects(*obb1.m_left, transform1, *obb2.m_right, transform2, collidedBox1, collidedBox2) ||
                intersects(*obb1.m_right, transform1, *obb2.m_left, transform2, collidedBox1, collidedBox2) ||
                intersects(*obb1.m_right, transform1, *obb2.m_right, transform2, collidedBox1, collidedBox2);
    }

}

