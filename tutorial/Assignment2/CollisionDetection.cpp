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
        return Eigen::AlignedBox3d(
                Eigen::Vector3d(newMin[0], newMin[1], newMin[2]),
                Eigen::Vector3d(newMax[0], newMax[1], newMax[2]));
    }

    static void assertAxis(const Eigen::AlignedBox3d &box,
                           const Eigen::Matrix4d &transform,
                           Eigen::Vector3d &C,
                           Eigen::Matrix3d &A,
                           Eigen::Vector3d &a) {

        Eigen::Vector3d
                blf = box.corner(Eigen::AlignedBox3d::BottomLeftFloor),
                brf = box.corner(Eigen::AlignedBox3d::BottomRightFloor),
                blc = box.corner(Eigen::AlignedBox3d::BottomLeftCeil),
                brc = box.corner(Eigen::AlignedBox3d::BottomRightCeil),
                tlf = box.corner(Eigen::AlignedBox3d::TopLeftFloor),
                trf = box.corner(Eigen::AlignedBox3d::TopRightFloor),
                tlc = box.corner(Eigen::AlignedBox3d::TopLeftCeil),
                trc = box.corner(Eigen::AlignedBox3d::TopRightCeil);
        Eigen::Vector4d
                blf4,
                brf4,
                blc4,
                brc4,
                tlf4,
                trf4,
                tlc4,
                trc4;
        blf4 << blf[0], blf[1], blf[2], 1;
        brf4 << brf[0], brf[1], brf[2], 1;
        blc4 << blc[0], blc[1], blc[2], 1;
        brc4 << brc[0], brc[1], brc[2], 1;
        tlf4 << tlf[0], tlf[1], tlf[2], 1;
        trf4 << trf[0], trf[1], trf[2], 1;
        tlc4 << tlc[0], tlc[1], tlc[2], 1;
        trc4 << trc[0], trc[1], trc[2], 1;
        blf4 = transform * blf4;
        brf4 = transform * brf4;
        blc4 = transform * blc4;
        brc4 = transform * brc4;
        tlf4 = transform * tlf4;
        trf4 = transform * trf4;
        tlc4 = transform * tlc4;
        trc4 = transform * trc4;
        blf = blf4.head(3);
        brf = brf4.head(3);
        blc = blc4.head(3);
        brc = brc4.head(3);
        tlf = tlf4.head(3);
        trf = trf4.head(3);
        tlc = tlc4.head(3);
        trc = trc4.head(3);
        if (blf != C + (-a[0]*A.col(0) -a[1]*A.col(1) -a[2]*A.col(2))) {
            std::cout << "+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*= assertAxis failed +*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=" << std::endl;
            std::cout << "blc is:\n " << blf << " but supposed to be:\n " << C + (-a[0]*A.col(0) -a[1]*A.col(1) -a[2]*A.col(2)) << std::endl;
        }
        if (brf != C + (a[0]*A.col(0) -a[1]*A.col(1) -a[2]*A.col(2))) {
            std::cout << "+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*= assertAxis failed +*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=" << std::endl;
            std::cout << "brf is:\n " << brf << " but supposed to be:\n " << C + (a[0]*A.col(0) -a[1]*A.col(1) -a[2]*A.col(2)) << std::endl;
        }
        if (blc != C + (-a[0]*A.col(0) -a[1]*A.col(1) +a[2]*A.col(2))) {
            std::cout << "+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*= assertAxis failed +*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=" << std::endl;
            std::cout << "blc is:\n " << blc << " but supposed to be:\n " << C + (-a[0]*A.col(0) -a[1]*A.col(1) +a[2]*A.col(2)) << std::endl;
        }
        if (brc != C + (a[0]*A.col(0) -a[1]*A.col(1) +a[2]*A.col(2))) {
            std::cout << "+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*= assertAxis failed +*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=" << std::endl;
            std::cout << "brc is:\n " << brc << " but supposed to be:\n " << C + (a[0]*A.col(0) -a[1]*A.col(1) +a[2]*A.col(2)) << std::endl;
        }
        if (tlf != C + (-a[0]*A.col(0) +a[1]*A.col(1) -a[2]*A.col(2))) {
            std::cout << "+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*= assertAxis failed +*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=" << std::endl;
            std::cout << "tlf is:\n " << tlf << " but supposed to be:\n " << C + (-a[0]*A.col(0) +a[1]*A.col(1) -a[2]*A.col(2)) << std::endl;
        }
        if (trf != C + (a[0]*A.col(0) +a[1]*A.col(1) -a[2]*A.col(2))) {
            std::cout << "+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*= assertAxis failed +*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=" << std::endl;
            std::cout << "trf is:\n " << trf << " but supposed to be:\n " << C + (a[0]*A.col(0) +a[1]*A.col(1) -a[2]*A.col(2)) << std::endl;
        }
        if (tlc != C + (-a[0]*A.col(0) +a[1]*A.col(1) +a[2]*A.col(2))) {
            std::cout << "+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*= assertAxis failed +*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=" << std::endl;
            std::cout << "tlc is:\n " << tlc << " but supposed to be:\n " << C + (-a[0]*A.col(0) +a[1]*A.col(1) +a[2]*A.col(2)) << std::endl;
        }
        if (trc != C + (a[0]*A.col(0) +a[1]*A.col(1) +a[2]*A.col(2))) {
            std::cout << "+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*= assertAxis failed +*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=" << std::endl;
            std::cout << "trc is:\n " << trc << " but supposed to be:\n " << C + (a[0]*A.col(0) +a[1]*A.col(1) +a[2]*A.col(2)) << std::endl;
        }
        std::cout << "+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*= assertAxis passed +*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=+*=" << std::endl;
    }

    static void calcBoxInSpace(const float &scale,
                               const Eigen::AlignedBox3d &box,
                               const Eigen::Affine3f &Tin,
                               Eigen::Vector3d &C,
                               Eigen::Matrix3d &A,
                               Eigen::Vector3d &a) {
        // calculate C (center) in the global space
        Eigen::Vector4d Cvec4;
        std::cout << "box.center()\n" << box.center() << std::endl;
//        Cvec4 << C[0], C[1], C[2], 1;
//        Cvec4 = transform * Cvec4;
//        C = Cvec4.head(3);
        Cvec4 << box.center()[0], box.center()[1], box.center()[2], 1;
        std::cout << "Tin\n" << Tin << std::endl;
        C = (Tin.cast<double>() * Cvec4).head(3);
        std::cout << "Center after Tin:\n" << C << std::endl;
        // calculate A1, A2, A3 axes in the global space
        Eigen::Vector4d A1vec4 = Tin.cast<double>() * Eigen::Vector4d(1, 0, 0, 0);
        std::cout << "A1vec4: \n" << A1vec4 << std::endl;
        Eigen::Vector4d A2vec4 = Tin.cast<double>() * Eigen::Vector4d(0, 1, 0, 0);
        std::cout << "A2vec4: \n" << A2vec4 << std::endl;
        Eigen::Vector4d A3vec4 = Tin.cast<double>() * Eigen::Vector4d(0, 0, 1, 0);
        std::cout << "A3vec4: \n" << A3vec4 << std::endl;
        std::cout << std::endl;
        A.col(0) = A1vec4.head(3);
        A.col(1) = A2vec4.head(3);
        A.col(2) = A3vec4.head(3);

        // calculate a1, a2, a3 extents
//        Eigen::Vector4d a_vec4;
//        a_vec4 << box.sizes()[0], box.sizes()[1], box.sizes()[2], 0;
//        std::cout << "a_vec4: " << a_vec4 << std::endl;
//        a = (cg3d::Movable::GetScaling(transform.cast<float>()).cast<double>() * a_vec4).head(3) / 2;
        a = (box.sizes() / 2) * scale;
        std::cout << "a: " << a << std::endl;
//        assertAxis(box, transform, C, A, a);
    }

    static bool intersects(
            const float &scale,
            const igl::AABB<Eigen::MatrixXd, 3> &obb1,
            const Eigen::Affine3f &Tin1,
            const igl::AABB<Eigen::MatrixXd, 3> &obb2,
            const Eigen::Affine3f &Tin2,
            Eigen::AlignedBox3d &collidedBox1,
            Eigen::AlignedBox3d &collidedBox2
    ) {
        std::cout << "///////////////////////////////////////////////////////////////" << std::endl;
        Eigen::Vector3d C1, a;
        Eigen::Matrix3d A;
        calcBoxInSpace(scale, obb1.m_box, Tin1, C1, A, a);
        Eigen::Vector3d C2, b;
        Eigen::Matrix3d B;
        calcBoxInSpace(scale, obb2.m_box, Tin2, C2, B, b);

        Eigen::Matrix3d C = A.transpose() * B;
        Eigen::Vector3d D = C2 - C1;
        std::cout << "C1 = " << C1.transpose() << " C2 = " << C2.transpose() << " D = " << D.transpose() << "\n\n";
//        Eigen::Vector4d C1vec4(obb1.m_box.center()[0], obb1.m_box.center()[1], obb1.m_box.center()[2], 1);
//        Eigen::Vector4d Dvec4 =
//                cg3d::Movable::GetTranslationRotation(transform2.cast<float>()).cast<double>() *
//                cg3d::Movable::GetTranslationRotation(transform1.cast<float>()).cast<double>().inverse() *
//                C1vec4;
//
//        Eigen::Vector3d D = Dvec4.head(3);

        double R0;
        double R1;
        double R;

        // L = A0
        R0 = a[0];
        R1 = b[0] * abs(C(0, 0)) + b[1] * abs(C(0, 1)) + b[2] * abs(C(0, 2));
        R = abs(A.col(0).transpose() * D);
        std::cout << "A0, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = A1
        R0 = a[1];
        R1 = b[0] * abs(C(1, 0)) + b[1] * abs(C(1, 1)) + b[2] * abs(C(1, 2));
        R = abs(A.col(1).transpose() * D);
        std::cout << "A1, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = A2
        R0 = a[2];
        R1 = b[0] * abs(C(2, 0)) + b[1] * abs(C(2, 1)) + b[2] * abs(C(2, 2));
        R = abs(A.col(2).transpose() * D);
        std::cout << "A2, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = B0
        R0 = a[0] * abs(C(0, 0)) + a[1] * abs(C(1, 0)) + a[2] * abs(C(2, 0));
        R1 = b[0];
        R = abs(B.col(0).transpose() * D);
        std::cout << "B0, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = B1
        R0 = a[0] * abs(C(0, 1)) + a[1] * abs(C(1, 1)) + a[2] * abs(C(2, 1));
        R1 = b[1];
        R = abs(B.col(1).transpose() * D);
        std::cout << "B1, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = B2
        R0 = a[0] * abs(C(0, 2)) + a[1] * abs(C(1, 2)) + a[2] * abs(C(2, 2));
        R1 = b[2];
        R = abs(B.col(2).transpose() * D);
        std::cout << "B2, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = A0 x B0
        R0 = a[1] * abs(C(2, 0)) + a[2] * abs(C(1, 0));
        R1 = b[1] * abs(C(0, 2)) + b[2] * abs(C(0, 1));
        double arg1 = C(1, 0) * A.col(2).transpose() * D;
        double arg2 = C(2, 0) * A.col(1).transpose() * D;
        R = abs(arg1 - arg2);
        std::cout << "A0 X B0, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = A0 x B1
        R0 = a[1] * abs(C(2, 1)) + a[2] * abs(C(1, 1));
        R1 = b[1] * abs(C(0, 2)) + b[2] * abs(C(0, 0));
        arg1 = C(1, 1) * A.col(2).transpose() * D;
        arg2 = C(2, 1) * A.col(1).transpose() * D;
        R = abs(arg1 - arg2);
        std::cout << "A0 X B1, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = A0 x B2
        R0 = a[1] * abs(C(2, 2)) + a[2] * abs(C(1, 2));
        R1 = b[1] * abs(C(0, 1)) + b[2] * abs(C(0, 0));
        arg1 = C(1, 2) * A.col(2).transpose() * D;
        arg2 = C(2, 2) * A.col(1).transpose() * D;
        R = abs(arg1 - arg2);
        std::cout << "A0 X B2, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = A1 x B0
        R0 = a[0] * abs(C(2, 0)) + a[2] * abs(C(0, 0));
        R1 = b[1] * abs(C(1, 2)) + b[2] * abs(C(1, 1));
        arg1 = C(2, 0) * A.col(0).transpose() * D;
        arg2 = C(0, 0) * A.col(2).transpose() * D;
        R = abs(arg1 - arg2);
        std::cout << "A1 X B0, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = A1 x B1
        R0 = a[0] * abs(C(2, 1)) + a[2] * abs(C(0, 1));
        R1 = b[0] * abs(C(1, 2)) + b[2] * abs(C(1, 0));
        arg1 = C(2, 1) * A.col(0).transpose() * D;
        arg2 = C(0, 1) * A.col(2).transpose() * D;
        R = abs(arg1 - arg2);
        std::cout << "A1 X B1, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = A1 x B2
        R0 = a[0] * abs(C(2, 2)) + a[2] * abs(C(0, 2));
        R1 = b[0] * abs(C(1, 1)) + b[1] * abs(C(1, 0));
        arg1 = C(2, 2) * A.col(0).transpose() * D;
        arg2 = C(0, 2) * A.col(2).transpose() * D;
        R = abs(arg1 - arg2);
        std::cout << "A1 X B2, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = A2 x B0
        R0 = a[0] * abs(C(1, 0)) + a[1] * abs(C(0, 0));
        R1 = b[1] * abs(C(2, 2)) + b[2] * abs(C(2, 1));
        arg1 = C(0, 0) * A.col(1).transpose() * D;
        arg2 = C(1, 0) * A.col(0).transpose() * D;
        R = abs(arg1 - arg2);
        std::cout << "A2 X B0, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = A2 x B1
        R0 = a[0] * abs(C(1, 1)) + a[1] * abs(C(0, 1));
        R1 = b[0] * abs(C(2, 2)) + b[2] * abs(C(2, 0));
        arg1 = C(0, 1) * A.col(1).transpose() * D;
        arg2 = C(1, 1) * A.col(0).transpose() * D;
        R = abs(arg1 - arg2);
        std::cout << "A2 X B1, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        // L = A2 x B2
        R0 = a[0] * abs(C(1, 2)) + a[1] * abs(C(0, 2));
        R1 = b[0] * abs(C(2, 1)) + b[1] * abs(C(2, 0));
        arg1 = C(0, 2) * A.col(1).transpose() * D;
        arg2 = C(1, 2) * A.col(0).transpose() * D;
        R = abs(arg1 - arg2);
        std::cout << "A2 X B2, R = " << R << std::endl << "R0 = " << R0 << " R1 = " << R1 << std::endl << std::endl;
        if (R > R0 + R1) return false;

        if (obb1.is_leaf() && obb2.is_leaf()) {
            std::cout << "Blyat it's true" << std::endl;
            collidedBox1 = obb1.m_box;
            collidedBox2 = obb2.m_box;
            return true;
        }
        return
                intersects(scale, *obb1.m_left, Tin1, *obb2.m_left, Tin2, collidedBox1, collidedBox2) ||
                intersects(scale, *obb1.m_left, Tin1, *obb2.m_right, Tin2, collidedBox1, collidedBox2) ||
                intersects(scale, *obb1.m_right, Tin1, *obb2.m_left, Tin2, collidedBox1, collidedBox2) ||
                intersects(scale, *obb1.m_right, Tin1, *obb2.m_right, Tin2, collidedBox1, collidedBox2);
    }

}

