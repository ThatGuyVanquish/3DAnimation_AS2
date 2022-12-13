#pragma once

#include "Scene.h"

#include "AutoMorphingModel.h"
#include <utility>
#include <Eigen/Core>
#include <Eigen/LU> 
#include <igl/opengl/glfw/Viewer.h>
#include <iostream>
#include "../engine/Mesh.h"
#include "MeshSimplification.h"
#include <igl/AABB.h>
#include "CollisionDetection.cpp"

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void CursorPosCallback(cg3d::Viewport* viewport, int x, int y, bool dragging, int* buttonState);

private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<MeshSimplification> myMeshObj;
    std::vector<std::shared_ptr<cg3d::AutoMorphingModel>> models;
    std::vector<igl::AABB<Eigen::MatrixXd, 3>> AABBs;
    std::vector<std::shared_ptr<cg3d::Model>> collisionBoxes;
    void reset(const int objIndex, std::vector<std::shared_ptr<cg3d::AutoMorphingModel>> &models);
    Eigen::Matrix4f prevTransform;
    void resetCB();
};
