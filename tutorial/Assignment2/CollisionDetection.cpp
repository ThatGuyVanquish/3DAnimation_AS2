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

        //std::cout << "bottom left floor " << blf.transpose() << std::endl;
        //std::cout << "bottom right floor " << brf.transpose() << std::endl;
        //std::cout << "blc " << blc.transpose() << std::endl;
        //std::cout << "brc " << brc.transpose() << std::endl;
        //std::cout << "tlf " << tlf.transpose() << std::endl;
        //std::cout << "trf " << trf.transpose() << std::endl;
        //std::cout << "tlc " << tlc.transpose() << std::endl;
        //std::cout << "trc " << trc.transpose() << std::endl;
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

    static bool intersects(
            const igl::AABB<Eigen::MatrixXd, 3> &obb1,
            const Eigen::Matrix4d &transform1,
            const igl::AABB<Eigen::MatrixXd, 3> &obb2,
            const Eigen::Matrix4d &transform2,
            Eigen::AlignedBox3d &collidedBox1,
            Eigen::AlignedBox3d &collidedBox2) {
        Eigen::AlignedBox3d transformedBox1 = transformBox(obb1.m_box, transform1),
                transformedBox2 = transformBox(obb2.m_box, transform2);
        if (transformedBox1.intersects(transformedBox2)) {
            if (obb1.is_leaf() && obb2.is_leaf()) {
                collidedBox1 = transformedBox1;
                collidedBox2 = transformedBox2;
                return true;
            }
            return
                    intersects(*obb1.m_left, transform1, *obb2.m_left, transform2, collidedBox1, collidedBox2) ||
                    intersects(*obb1.m_left, transform1, *obb2.m_right, transform2, collidedBox1, collidedBox2) ||
                    intersects(*obb1.m_right, transform1, *obb2.m_left, transform2, collidedBox1, collidedBox2) ||
                    intersects(*obb1.m_right, transform1, *obb2.m_right, transform2, collidedBox1, collidedBox2);
        } else return false;
    }
}

