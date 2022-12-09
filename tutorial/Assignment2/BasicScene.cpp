#include "BasicScene.h"
#include <chrono>
using namespace cg3d;

//possible fields
bool startMoving = false;
float speed = 0.01;
int dist = 0;
Eigen::Vector3f dir = Movable::AxisVec(Movable::Axis::X);
Eigen::MatrixXd V;
Eigen::MatrixXi F;


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
    material->AddTexture(0, "textures/box0.bmp", 2);
    std::vector<std::string> objFiles{ "data/bunny.off", /* 0 */
        "data/sphere.obj", /* 1 */
        "data/cheburashka.off", /* 2 */
        "data/fertility.off" /* 3 */,
        "data/cube.off"};
    int objIndex = 4;
    int decimations = 7;
    int recalcQsRate = 10;
    std::chrono::time_point<std::chrono::steady_clock> m_StartTime = std::chrono::high_resolution_clock::now();
    myMeshObj = std::make_shared<MeshSimplification>(MeshSimplification(objFiles[objIndex], decimations, recalcQsRate));
    std::chrono::duration<float> duration = std::chrono::high_resolution_clock::now() - m_StartTime;
    std::cout << "Duration: " << duration.count() << std::endl;
    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        return model->meshIndex;
    };

    float cameraTranslate = 0;
    
    int numOfModels = 2;
    models = std::vector<std::shared_ptr<cg3d::AutoMorphingModel>>(2);
    for (int i = 0; i < numOfModels; i++)
    {
        models[i] = cg3d::AutoMorphingModel::Create(
            *cg3d::Model::Create("My Model", myMeshObj->getMesh(), material),
            morphFunc
        );
        root->AddChild(models[i]);
        switch (objIndex)
        {
        case 0: /* Bunny */
            models[i]->Scale(20.0f);
            cameraTranslate = 10;
            dist = 2;
            break;
        case 1: /* Sphere */
            models[i]->Scale(2.5f);
            cameraTranslate = 10;
            dist = 3;
            break;
        case 2: /* Cheburashka */
            models[i]->Scale(12.5f);
            cameraTranslate = 40;
            dist = 3;
            break;
        case 3: /* Fertility */
            models[i]->Scale(0.078f);
            cameraTranslate = 40;
            break;
        default:
            models[i]->Scale(1.5f);
            cameraTranslate = 10;
            dist = 2;
        }
        models[i]->showWireframe = true;
        if (i % 2 == 0) models[i]->Translate(dist, Movable::Axis::X);
        else models[i]->Translate(-dist, Movable::Axis::X);
    }
    
    // Create AABBs
    AABBs = std::vector<igl::AABB<Eigen::MatrixXd, 3>>(2);
    //igl::read_triangle_mesh(objFiles[objIndex], V, F);
    //V.conservativeResize(V.rows(), 4);
    //for(int i = 0; i < V.rows(); i++)
    //    V(i, 3) = 1;
    // place models on the screen MISSING
    camera->Translate(cameraTranslate, Axis::Z);
    // somewhere, whenever we press the button "K" start moving models[0] towards models[1], 
    // allow arrows to move it in 2D towards models[1] in range of 180 degrees

}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
    if (startMoving)
    {
        models[1]->Translate(speed * dir);
    }
    
    //for (int i = 0; i < 2; i++)
    //{
        //auto temp = V * models[i]->GetTransform();
        //Eigen::MatrixXd tempV(V.rows(), 3);
        /*
        
        we want to take V which is N * 3 and change it to N * 4 to multiply it by transformation matrix 4x4
        Then we get the real position of every vertex according to the global environment
        Then we can initialize a new N * 3 matrix of doubles and initialize the AABBs with it.
        Afterwards we calculate the collision - we don't know yet blyat 

        find out how to calculate a_i and b_i and A_i and B_i and center of mass of mesh objects

        */
        //AABBs[i].init(temp, F);
    //}
    std::cout << "Model 1 is in \n" << models[1]->GetTransform() << std::endl;
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
            models[1]->Rotate(1.0, Axis::Y);
            dir = models[1]->GetRotation() * dir;
            //if (myAutoModel->meshIndex > 0)
            //    myAutoModel->meshIndex--;
            break;
        case GLFW_KEY_DOWN:
            models[1]->Rotate(-1.0, Axis::Y);
            dir = models[1]->GetRotation() * dir;
            //if (myAutoModel->meshIndex < myAutoModel->GetMesh(0)->data.size())
            //    myAutoModel->meshIndex++;
            break;
        case GLFW_KEY_LEFT:
            models[1]->Rotate(-1.0, Axis::X);
            dir = models[1]->GetRotation() * dir;
            /*camera->RotateInSystem(system, 0.1f, Axis::Y);*/
            break;
        case GLFW_KEY_RIGHT:
            models[1]->Rotate(1.0, Axis::X);
            dir = models[1]->GetRotation() * dir;
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
            startMoving = false;
            //models[0]->TranslateInSystem(system, { dist, 0, 0 });
            //models[1]->TranslateInSystem(system, { -dist, 0, 0 });
            break;
        case GLFW_KEY_Q:
            dir *= -1;
            break;
        }
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