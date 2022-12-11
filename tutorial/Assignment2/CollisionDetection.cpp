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
	
	static Eigen::AlignedBox<double, 3> transformBox(
		Eigen::AlignedBox<double, 3> box, 
		const Eigen::Matrix4d& transform
	)
	{
		Eigen::Vector4d newMin = (box.min(), 1) * transform;
		Eigen::Vector4d newMax = (box.max(), 1) * transform;
		return Eigen::AlignedBox<double, 3>(
			(newMin.col(0), newMin.col(1), newMin.col(2)),
			(newMax.col(0), newMax.col(1), newMax.col(2)));
	}

	static bool intersects(
		const igl::AABB<Eigen::MatrixXd, 3>& obb1,
		const Eigen::Matrix4d& transform1,
		const igl::AABB<Eigen::MatrixXd, 3>& obb2,
		const Eigen::Matrix4d& transform2,
		Eigen::AlignedBox<double, 3> &collidedBox1,
		Eigen::AlignedBox<double, 3>& collidedBox2)
	{
		auto transformedBox1 = transformBox(obb1.m_box, transform1),
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

