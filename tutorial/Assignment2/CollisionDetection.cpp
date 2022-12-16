#pragma once

#include <utility>
#include <Eigen/Core>
#include <Eigen/LU>
#include <igl/opengl/glfw/Viewer.h>
#include <iostream>
#include "../engine/Mesh.h"
#include "MeshSimplification.h"
#include <igl/AABB.h>
#include <GLFW/glfw3.h>
#include <igl/per_vertex_normals.h>
#include <Movable.h>
/*

	Helper functions for collision detection

*/

namespace CollisionDetection {

    static std::shared_ptr<cg3d::Mesh> meshifyBoundingBox(Eigen::AlignedBox3d &box) {

        Eigen::Vector3d
                blf = box.corner(Eigen::AlignedBox3d::BottomLeftFloor),
                brf = box.corner(Eigen::AlignedBox3d::BottomRightFloor),
                blc = box.corner(Eigen::AlignedBox3d::BottomLeftCeil),
                brc = box.corner(Eigen::AlignedBox3d::BottomRightCeil),
                tlf = box.corner(Eigen::AlignedBox3d::TopLeftFloor),
                trf = box.corner(Eigen::AlignedBox3d::TopRightFloor),
                tlc = box.corner(Eigen::AlignedBox3d::TopLeftCeil),
                trc = box.corner(Eigen::AlignedBox3d::TopRightCeil);
        Eigen::MatrixXd V(8, 3);
        V.row(0) = trc;
        V.row(1) = tlc;
        V.row(2) = blc;
        V.row(3) = brc;
        V.row(4) = trf;
        V.row(5) = tlf;
        V.row(6) = blf;
        V.row(7) = brf;


        Eigen::MatrixXi F(12, 3);

        F.row(0) = Eigen::Vector3i(0, 1, 2);
        F.row(1) = Eigen::Vector3i(0, 2, 3);
        F.row(2) = Eigen::Vector3i(7, 4, 0);
        F.row(3) = Eigen::Vector3i(7, 0, 3);
        F.row(4) = Eigen::Vector3i(4, 5, 1);
        F.row(5) = Eigen::Vector3i(4, 1, 0);
        F.row(6) = Eigen::Vector3i(5, 6, 2);
        F.row(7) = Eigen::Vector3i(5, 2, 1);
        F.row(8) = Eigen::Vector3i(3, 2, 6);
        F.row(9) = Eigen::Vector3i(3, 6, 7);
        F.row(10) = Eigen::Vector3i(6, 5, 4);
        F.row(11) = Eigen::Vector3i(6, 4, 7);

        Eigen::MatrixXd N;
        igl::per_vertex_normals(V, F, N);
        return std::make_shared<cg3d::Mesh>(cg3d::Mesh("Bounding box", V, F, N, {}));
    }

    static void calcBoxInSpace(const float &scale,
                               const Eigen::AlignedBox3d &box,
                               const Eigen::Matrix4f &transform,
                               Eigen::Vector3d &C,
                               Eigen::Matrix3d &A,
                               Eigen::Vector3d &a) {
        // calculate C (center) in the global space
        Eigen::Vector4d Cvec4;
        Cvec4 << box.center()[0], box.center()[1], box.center()[2], 1;
        C = (transform.cast<double>() * Cvec4).head(3);
        // calculate A1, A2, A3 axes in the global space
        Eigen::Vector4d A1vec4 = transform.cast<double>() * Eigen::Vector4d(1, 0, 0, 0);
        Eigen::Vector4d A2vec4 = transform.cast<double>() * Eigen::Vector4d(0, 1, 0, 0);
        Eigen::Vector4d A3vec4 = transform.cast<double>() * Eigen::Vector4d(0, 0, 1, 0);
        A.col(0) = A1vec4.head(3).normalized();
        A.col(1) = A2vec4.head(3).normalized();
        A.col(2) = A3vec4.head(3).normalized();
        // calculate a1, a2, a3 extents
        a = (box.sizes() / 2) * scale;
    }

    static bool intersects(
            const float &scale,
            const igl::AABB<Eigen::MatrixXd, 3> &obb0,
            const Eigen::Matrix4f &transform0,
            const igl::AABB<Eigen::MatrixXd, 3> &obb1,
            const Eigen::Matrix4f &transform1,
            Eigen::AlignedBox3d &collidedBox0,
            Eigen::AlignedBox3d &collidedBox1
    ) {
        Eigen::Vector3d C0, a;
        Eigen::Matrix3d A;
        calcBoxInSpace(scale, obb0.m_box, transform0, C0, A, a);
        Eigen::Vector3d C1, b;
        Eigen::Matrix3d B;
        calcBoxInSpace(scale, obb1.m_box, transform1, C1, B, b);

        Eigen::Matrix3d C = A.transpose() * B;
        Eigen::Vector3d D = C1 - C0;

        double R0;
        double R1;
        double R;

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
        R0 = a[0] * abs(C(0, 0)) + a[1] * abs(C(1, 0)) + a[2] * abs(C(2, 0));
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
        double arg1 = C(1, 0) * A.col(2).transpose() * D;
        double arg2 = C(2, 0) * A.col(1).transpose() * D;
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

        if (obb0.is_leaf() && obb1.is_leaf()) {
            collidedBox0 = obb0.m_box;
            collidedBox1 = obb1.m_box;
            return true;
        }
        if (obb0.m_left == nullptr || obb1.m_left == nullptr) return false;
        return
                intersects(scale, *obb0.m_left, transform0, *obb1.m_left, transform1, collidedBox0, collidedBox1) ||
                intersects(scale, *obb0.m_left, transform0, *obb1.m_right, transform1, collidedBox0, collidedBox1) ||
                intersects(scale, *obb0.m_right, transform0, *obb1.m_left, transform1, collidedBox0, collidedBox1) ||
                intersects(scale, *obb0.m_right, transform0, *obb1.m_right, transform1, collidedBox0, collidedBox1);
    }
}

