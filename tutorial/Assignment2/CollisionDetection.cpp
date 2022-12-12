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

namespace CollisionDetection
{
	static void fourColumnVertexMatrix(Eigen::MatrixXd& V)
	{
		V.conservativeResize(V.rows(), 4);
		for (int i = 0; i < V.rows(); i++)
			V(i, 3) = 1;
	}
	
	static Eigen::AlignedBox3d transformBox(
		Eigen::AlignedBox3d box, 
		const Eigen::Matrix4f& transform
	)
	{
		auto min = box.min(), max = box.max();
		Eigen::Vector4f newMin;
		newMin << min[0], min[1], min[2], 1;
		newMin = transform * newMin;
		Eigen::Vector4f newMax;
		newMax << max[0], max[1], max[2], 1;
		newMax = transform * newMax;
		return Eigen::AlignedBox<double, 3>(
			Eigen::Vector3d(newMin[0], newMin[1], newMin[2]),
			Eigen::Vector3d(newMax[0], newMax[1], newMax[2]));
	}

	static bool intersects(
		const igl::AABB<Eigen::MatrixXd, 3>& obb1,
		const Eigen::Matrix4f& transform1,
		const igl::AABB<Eigen::MatrixXd, 3>& obb2,
		const Eigen::Matrix4f& transform2,
		Eigen::AlignedBox3d &collidedBox1,
		Eigen::AlignedBox3d &collidedBox2)
	{
		std::cout << "blyat this is here\n";
		Eigen::AlignedBox3d transformedBox1 = transformBox(obb1.m_box, transform1),
			transformedBox2 = transformBox(obb2.m_box, transform2);
		if (transformedBox1.intersects(transformedBox2))
		{
			if (obb1.is_leaf() && obb2.is_leaf())
			{
				collidedBox1 = transformedBox1;
				collidedBox2 = transformedBox2;
				return true;
			}
			return
				intersects(*obb1.m_left, transform1, *obb2.m_left, transform2, collidedBox1, collidedBox2) ||
				intersects(*obb1.m_left, transform1, *obb2.m_right, transform2, collidedBox1, collidedBox2) ||
				intersects(*obb1.m_right, transform1, *obb2.m_left, transform2, collidedBox1, collidedBox2) ||
				intersects(*obb1.m_right, transform1, *obb2.m_right, transform2, collidedBox1, collidedBox2);
		}
		else return false;
	}
}

