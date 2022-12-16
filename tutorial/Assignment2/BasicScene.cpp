#include "BasicScene.h"
#include <chrono>
using namespace cg3d;
int kasd = 0;
//possible fields
bool startMoving = false;
bool firstTime = true;
float speed;
float cameraTranslate;
int objIndex, decimations, recalcQsRate;
Eigen::Vector3f dir = Movable::AxisVec(Movable::Axis::X);
std::shared_ptr<cg3d::Model> m0, m1;
std::shared_ptr<cg3d::Material> red;
void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create( "camera", fov, float(width) / height, near, far);
    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")}; 
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program)}; // empty material
    material->AddTexture(0, "textures/carbon.jpg", 2);

    auto green{ std::make_shared<Material>("green", program, true)};
    green->AddTexture(0, "textures/grass.bmp", 2);
    red = { std::make_shared<Material>("red", program) };
    red->AddTexture(0, "textures/box0.bmp", 2);

    std::vector<std::string> objFiles{ "data/bunny.off", /* 0 */
        "data/sphere.obj", /* 1 */
        //"data/cheburashka.off", /* 2 */
        //"data/fertility.off" /* 3 */,
        "data/cube.off"};
    objIndex = 0;
    decimations = 0;
    recalcQsRate = 10;
    std::chrono::time_point<std::chrono::steady_clock> m_StartTime = std::chrono::high_resolution_clock::now();
    myMeshObj = std::make_shared<MeshSimplification>(MeshSimplification(objFiles[objIndex], decimations, recalcQsRate));
    std::chrono::duration<float> duration = std::chrono::high_resolution_clock::now() - m_StartTime;
    std::cout << "Duration: " << duration.count() << std::endl;
    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        return model->meshIndex;
    };

    int numOfModels = 2;
    models = std::vector<std::shared_ptr<cg3d::AutoMorphingModel>>(numOfModels);
    for (int i = 0; i < numOfModels; i++)
    {
        models[i] = cg3d::AutoMorphingModel::Create(
            *cg3d::Model::Create("My Model " +std::to_string(i), myMeshObj->getMesh(), material),
            morphFunc
        );
        root->AddChild(models[i]);
    }
    modelScale = BasicScene::reset(objIndex, models);
    Eigen::MatrixXd V = myMeshObj->getMesh()->data[0].vertices;
    Eigen::MatrixXi F = myMeshObj->getMesh()->data[0].faces;
    for (int i = 0; i < 2; i++)
    {
        igl::AABB<Eigen::MatrixXd, 3> axisAligned;
        axisAligned.init(V, F);
        AABBs.push_back(axisAligned);
    }

    // bounding box for stationary object
    collisionBoxes.push_back(
        cg3d::Model::Create(
            "Bounding box 0", 
            CollisionDetection::meshifyBoundingBox(AABBs[0].m_box), 
            green
        )
    );
    collisionBoxes[0]->aggregatedTransform = models[0]->aggregatedTransform;
    collisionBoxes[0]->showFaces = false;
    collisionBoxes[0]->showWireframe = true;
    models[0]->AddChild(collisionBoxes[0]);
    models[0]->showFaces = false;
   
    // bounding box for moving object
    collisionBoxes.push_back(
        cg3d::Model::Create(
            "Bounding box 1", 
            CollisionDetection::meshifyBoundingBox(AABBs[1].m_box),
            green
        )
    );
    collisionBoxes[1]->aggregatedTransform = models[1]->aggregatedTransform;
    models[1]->AddChild(collisionBoxes[1]);
    collisionBoxes[1]->showFaces = false;
    collisionBoxes[1]->showWireframe = true;
    models[1]->showFaces = false;
    m0 = cg3d::Model::Create(
        "cb0",
        cg3d::Mesh::Cube(),
        red
    );
    m1 = cg3d::Model::Create(
        "cb1",
        cg3d::Mesh::Cube(),
        red
    );
    m0->isHidden = true;
    m1->isHidden = true;
    m0->aggregatedTransform = models[0]->aggregatedTransform;
    models[0]->AddChild(m0);
    m1->aggregatedTransform = models[1]->aggregatedTransform;
    models[1]->AddChild(m1);
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    //program.SetUniform4f("lightColor", 100.0f, 145.0f, 1.0f, 0.5f);
    //program.SetUniform4f("Kai", 0.0f, 1.0f, 1.0f, 1.0f);
    if (startMoving)
    {
        models[1]->Translate(speed * dir);
    }
    Eigen::AlignedBox3d box0, box1;
    if (models[1]->GetTransform() != prevTransform)
    {
        kasd++;
        std::cout << "-------------------------------------------------- test --------------------------------------------------" << kasd << std::endl;
        prevTransform = models[1]->GetTransform();
        if (CollisionDetection::intersects(
                modelScale,
                AABBs[0],
                models[0]->GetTransform(),
                AABBs[1],
                models[1]->GetTransform(),
                box0,
                box1))
            {
                startMoving = false;
                if (firstTime)
                {
                    std::vector<std::shared_ptr<cg3d::Mesh>> test0, test1;
                    test0.push_back(CollisionDetection::meshifyBoundingBox(box0));
                    test1.push_back(CollisionDetection::meshifyBoundingBox(box1));
                    m0->SetMeshList(test0);
                    m1->SetMeshList(test1);
                    m0->showFaces = false;
                    m1->showFaces = false;
                    m0->showWireframe = true;
                    m1->showWireframe = true;
                    m0->isHidden = false;
                    m1->isHidden = false;
                    //m1->Translate({ 0,0,10 });
                    //m2->Translate({ 0,0,10 });
                    firstTime = false;
                }
            }
    }

}

void BasicScene::KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_UP:
            models[1]->Rotate(0.1, Axis::Z);
            break;
        case GLFW_KEY_DOWN:
            models[1]->Rotate(-0.1, Axis::Z);
            break;
        case GLFW_KEY_LEFT:
            models[1]->Rotate(0.1, Axis::Y);
            /*camera->RotateInSystem(system, 0.1f, Axis::Y);*/
            break;
        case GLFW_KEY_RIGHT:
            models[1]->Rotate(-0.1, Axis::Y);
            /*camera->RotateInSystem(system, -0.1f, Axis::Y);*/
            break;
        case GLFW_KEY_W:
            camera->TranslateInSystem(system, { 0, 0.05f, 0 });
            break;
        case GLFW_KEY_S:
            camera->TranslateInSystem(system, { 0, -0.05f, 0 });
            break;
        case GLFW_KEY_A:
            camera->TranslateInSystem(system, { -0.05f, 0, 0 });
            break;
        case GLFW_KEY_D:
            camera->TranslateInSystem(system, { 0.05f, 0, 0 });
            break;
        case GLFW_KEY_E:
            camera->RotateInSystem(system, -0.05, cg3d::Movable::Axis::Y);
            break;
        case GLFW_KEY_Q:
            camera->RotateInSystem(system, 0.05, cg3d::Movable::Axis::Y);
            break;
        case GLFW_KEY_B:
            camera->TranslateInSystem(system, { 0, 0, 0.05f });
            break;
        case GLFW_KEY_F:
            camera->TranslateInSystem(system, { 0, 0, -0.05f });
            break;
        case GLFW_KEY_K:
            startMoving = true;
            break;
        case GLFW_KEY_R: // reset location
            BasicScene::reset(objIndex, models);
            BasicScene::resetCB();
            startMoving = false;
            break;
        case GLFW_KEY_Z:
            speed *= -1;
            break;
        case GLFW_KEY_X:
            startMoving = !startMoving;
            break;
        case GLFW_KEY_1:
            collisionBoxes[0]->isHidden = !collisionBoxes[0]->isHidden;
            collisionBoxes[1]->isHidden = !collisionBoxes[1]->isHidden;
            break;
        case GLFW_KEY_2:
            models[0]->isHidden = !models[0]->isHidden;
            models[1]->isHidden = !models[1]->isHidden;
            break;
        case GLFW_KEY_3:
            m0->isHidden = !m0->isHidden;
            m1->isHidden = !m1->isHidden;
            break;
        case GLFW_KEY_4:
            m0->isHidden = !m0->isHidden;
            break;
        case GLFW_KEY_5:
            models[0]->isHidden = !models[0]->isHidden;
            break;
        case GLFW_KEY_6:
            m1->isHidden = !m1->isHidden;
            break;
        case GLFW_KEY_7:
            models[1]->isHidden = !models[1]->isHidden;
            break;
        case GLFW_KEY_EQUAL:
            camera->Translate({ 0,0,-0.1 });
            break;
        case GLFW_KEY_MINUS:
            camera->Translate({ 0,0,0.1 });
            break;
        case GLFW_KEY_0:
            camera->SetTransform(Eigen::Matrix4f::Identity());
            camera->Translate({ 0, 0, cameraTranslate });
            break;
        case GLFW_KEY_RIGHT_BRACKET:
            if (models[0]->meshIndex > 0)
            {
                models[0]->meshIndex--;
                models[1]->meshIndex--;
            }
            break;
        case GLFW_KEY_LEFT_BRACKET:
            if (models[0]->meshIndex < decimations)
            {
                models[0]->meshIndex++;
                models[1]->meshIndex++;
            }
            break;
    }
        dir = models[1]->GetRotation() * Eigen::Vector3f::Identity();
    }
}

void BasicScene::CursorPosCallback(cg3d::Viewport* viewport, int x, int y, bool dragging, int* buttonState)
{
    if (dragging) {
        auto system = camera->GetRotation().transpose();
        auto moveCoeff = camera->CalcMoveCoeff(pickedModelDepth, viewport->width);
        auto angleCoeff = camera->CalcAngleCoeff(viewport->width);
        if (pickedModel) {
            pickedModel->SetTout(pickedToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
                pickedModel->TranslateInSystem(system, { float(x - xAtPress) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                pickedModel->RotateInSystem(system, float(x - xAtPress) / moveCoeff, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                pickedModel->RotateInSystem(system, float(x - xAtPress) / moveCoeff, Axis::Y);
                pickedModel->RotateInSystem(system, float(y - yAtPress) / moveCoeff, Axis::X);
            }
        }
        else {
            camera->SetTout(cameraToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE)
                camera->TranslateInSystem(system, { float(xAtPress - x) / moveCoeff, float(y - yAtPress) / moveCoeff, 0 });
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                camera->RotateInSystem(system, float(x - xAtPress) / 180, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE) {
                camera->RotateInSystem(system, float(x - xAtPress) / angleCoeff, Axis::Y);
                camera->RotateInSystem(system, float(y - yAtPress) / angleCoeff, Axis::X);
            }
        }
    }
}

float BasicScene::reset(const int objIndex, std::vector<std::shared_ptr<cg3d::AutoMorphingModel>>& models)
{
    camera->SetTransform(Eigen::Matrix4f::Identity());
    float scale, distX = 0, distY = 0;
    cameraTranslate = 0;
    switch (objIndex)
    {
        case 0: /* Bunny */
            speed = -0.005;
            scale = 3;
            cameraTranslate = 1;
            distX = 0.4;
            distY = -0.3;
            break;
        case 1: /* Sphere */
            speed = 0.005;
            scale = 1;
            cameraTranslate = 10;
            distX = 1;
            distY = -1;
            break;
        //case 2: /* Cheburashka */
        //    scale = 12.5;
        //    cameraTranslate = 40;
        //    distX = 3;
        //    distY = -3;
        //    break;
        //case 3: /* Fertility */
        //    scale = 0.078;
        //    cameraTranslate = 40;
        //    distX = 0.7;
        //    distY = -0.4;
        //    break;
        default:
            speed = 0.05;
            scale = 1.5;
            cameraTranslate = 10;
            distX = 2;
            distY = 0;
    }
    for (int i = 0; i < models.size(); i++)
    {
        models[i]->SetTransform(Eigen::Matrix4f::Identity());
        models[i]->Scale(scale);
        models[i]->showWireframe = true;
        if (i % 2 == 0)
            models[i]->Translate(distX, Movable::Axis::X);
        else
        {
            models[i]->Translate(-distX, Movable::Axis::X);
            if (objIndex == 0) models[i]->Rotate(3.1, Movable::Axis::Y); // rotate bunnies to face each other
        }
        models[i]->Translate(distY, Movable::Axis::Y);
    }
    camera->Translate({ 0, 0, cameraTranslate });
    return scale;
}

void BasicScene::resetCB()
{
    firstTime = true;
    m0->isHidden = true;
    m1->isHidden = true;
    for(int i = 0; i < collisionBoxes.size(); i++)
        collisionBoxes[i]->SetTransform(Eigen::Matrix4f::Identity());
}