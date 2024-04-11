#include <eigen_graphics/all.h>
#include <iostream>
// #include <cmath>

// clang-format off

int main() {
  // Test case for translate

  {
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> trans = Eigen::Graphics::translate(1.0f, 2.0f, 3.0f);
    Eigen::Matrix4f expected;
    expected << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                1, 2, 3, 1;
    if (trans.isApprox(expected))
      std::cout << "translate test passed!\n";
    else
      std::cout << "translate test failed!\n";

    std::cout << trans;
  }

  // Test case for scale
  {
    Eigen::Matrix4f scaling = Eigen::Graphics::scale(2.0f, 3.0f, 4.0f);
    Eigen::Matrix4f expected;
    expected << 2, 0, 0, 0,
                0, 3, 0, 0,
                0, 0, 4, 0,
                0, 0, 0, 1;
    if (scaling.isApprox(expected))
      std::cout << "scale test passed!\n";
    else
      std::cout << "scale test failed!\n";
  }

  // Test case for rotateRh
  {
    float angleX = EIGEN_PI / 4;
    float angleY = EIGEN_PI / 3;
    float angleZ = EIGEN_PI / 2;
    Eigen::Matrix4f rotation = Eigen::Graphics::rotateRh(angleX, angleY, angleZ);
    Eigen::Matrix4f expected;
    expected <<  0.5000000,  0.1889822,  0.8660254,  0.0000000,
                -0.8660254,  0.2500000,  0.4330127,  0.0000000,
                 0.0000000, -0.9486833,  0.3162278,  0.0000000,
                 0.0000000,  0.0000000,  0.0000000,  1.0000000;
    if (rotation.isApprox(expected, 1e-5))
      std::cout << "rotateRh test passed!\n";
    else
      std::cout << "rotateRh test failed!\n";
  }

  // Test case for lookAtRh
  {
    Eigen::Matrix<float, 3, 1> eye(0, 0, 0);
    Eigen::Matrix<float, 3, 1> target(0, 0, -1);
    Eigen::Matrix<float, 3, 1> up(0, 1, 0);
    Eigen::Matrix4f view = Eigen::Graphics::lookAtRh(eye, target, up);
    Eigen::Matrix4f expected;
    expected <<  1,  0,  0,  0,
                 0,  1,  0,  0,
                 0,  0, -1,  0,
                 0,  0,  0,  1;
    if (view.isApprox(expected))
      std::cout << "lookAtRh test passed!\n";
    else
      std::cout << "lookAtRh test failed!\n";
  }

  // Test case for perspectiveRhNo
  {
    float fovY = EIGEN_PI / 3;
    float aspect = 1.5f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f proj = Eigen::Graphics::perspectiveRhNo(fovY, aspect, zNear, zFar);
    Eigen::Matrix4f expected;
    expected <<  0.7071068,  0.0000000,  0.0000000,  0.0000000,
                 0.0000000,  1.0606602,  0.0000000,  0.0000000,
                 0.0000000,  0.0000000, -1.0020020, -1.0000000,
                 0.0000000,  0.0000000, -0.2002002,  0.0000000;
    if (proj.isApprox(expected, 1e-5))
      std::cout << "perspectiveRhNo test passed!\n";
    else 
      std::cout << "perspectiveRhNo test failed!\n";
  }

  // Test case for orthoRhNo
  {
    float width = 4.0f;
    float height = 3.0f;
    float zNear = 0.1f; 
    float zFar = 100.0f;
    Eigen::Matrix4f ortho = Eigen::Graphics::orthoRhNo(width, height, zNear, zFar);
    Eigen::Matrix4f expected;
    expected <<  0.5000000,  0.0000000,  0.0000000,  0.0000000,
                 0.0000000,  0.6666667,  0.0000000,  0.0000000,
                 0.0000000,  0.0000000, -0.0200200, -0.0000000,
                 0.0000000,  0.0000000, -1.0020020,  1.0000000;
    if (ortho.isApprox(expected, 1e-5))
      std::cout << "orthoRhNo test passed!\n";
    else
      std::cout << "orthoRhNo test failed!\n";
  }

  // Test case for transformPoint
  {
    Eigen::Matrix<float, 3, 1> point(1, 2, 3);
    Eigen::Matrix4f matrix;
    matrix << 1, 0, 0, 1,
              0, 1, 0, 2,
              0, 0, 1, 3,
              0, 0, 0, 1;
    Eigen::Matrix<float, 3, 1> result = Eigen::Graphics::transformPoint(point, matrix);
    Eigen::Matrix<float, 3, 1> expected(2, 4, 6);
    if (result.isApprox(expected))
      std::cout << "transformPoint test passed!\n";
    else  
      std::cout << "transformPoint test failed!\n";
  }

  return 0;
}

constexpr float kPi = 3.14159265358979323846f;

int main() {
  // Test case for translate
  {
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> trans = Eigen::Graphics::translate(1.0f, 2.0f, 3.0f);
    
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> trans1 = Eigen::Graphics::translate(Eigen::Vector3f(1.0f, 2.0f, 3.0f));
    Eigen::Matrix4f expected;
    expected << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                1, 2, 3, 1;
    if (trans.isApprox(expected))
      std::cout << "translate test passed!\n";
    else
      std::cout << "translate test failed!\n";
  }

  // Test case for addTranslate
  {
    Eigen::Matrix<float, 4, 4, Eigen::ColMajor> matrix = Eigen::Matrix4f::Identity();
    Eigen::Graphics::addTranslate(matrix, 1.0f, 2.0f, 3.0f);
    //Eigen::Graphics::addTranslate<Eigen::ColMajor>(matrix, 1.0f, 2.0f, 3.0f);
    Eigen::Graphics::addTranslate(matrix, Eigen::Vector3f(1.0f, 2.0f, 3.0f));
    //Eigen::Graphics::addTranslate<Eigen::ColMajor>(matrix, Eigen::Vector3f(1.0f, 2.0f, 3.0f));
    Eigen::Matrix4f expected;
    expected << 1, 0, 0, 1,
                0, 1, 0, 2,
                0, 0, 1, 3,
                0, 0, 0, 1;
    if (matrix.isApprox(expected))
      std::cout << "addTranslate test passed!\n";
    else
      std::cout << "addTranslate test failed!\n";
  }

  // Test case for setTranslate
  {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Zero();
    Eigen::Graphics::setTranslate(matrix, 1.0f, 2.0f, 3.0f);
    Eigen::Matrix4f expected;
    expected << 0, 0, 0, 1,
                0, 0, 0, 2,
                0, 0, 0, 3,
                0, 0, 0, 1;
    if (matrix.isApprox(expected))
      std::cout << "setTranslate test passed!\n";
    else
      std::cout << "setTranslate test failed!\n";
  }

  // Test case for scale
  {
    Eigen::Matrix4f scaleMatrix = Eigen::Graphics::scale(2.0f, 3.0f, 4.0f);
    Eigen::Matrix4f expected;
    expected << 2, 0, 0, 0,
                0, 3, 0, 0,
                0, 0, 4, 0,
                0, 0, 0, 1;
    if (scaleMatrix.isApprox(expected))
      std::cout << "scale test passed!\n";
    else
      std::cout << "scale test failed!\n";
  }

  // Test case for rotateRhX
  {
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateRhX(angle);
    Eigen::Matrix4f expected;
    expected << 1, 0,           0,          0,
                0, std::cos(angle), -std::sin(angle), 0,
                0, std::sin(angle),  std::cos(angle), 0,
                0, 0,           0,          1;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateRhX test passed!\n";
    else
      std::cout << "rotateRhX test failed!\n";
  }

  // Test case for rotateRhY
  {
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateRhY(angle);
    Eigen::Matrix4f expected;
    expected <<  std::cos(angle), 0, std::sin(angle), 0,
                 0,          1, 0,          0,
                -std::sin(angle), 0, std::cos(angle), 0,
                 0,          0, 0,          1;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateRhY test passed!\n";
    else
      std::cout << "rotateRhY test failed!\n";
  }

  // Test case for rotateRhZ
  {
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateRhZ(angle);
    Eigen::Matrix4f expected;
    expected << std::cos(angle), -std::sin(angle), 0, 0,
                std::sin(angle),  std::cos(angle), 0, 0,
                0,           0,          1, 0,
                0,           0,          0, 1;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateRhZ test passed!\n";
    else
      std::cout << "rotateRhZ test failed!\n";
  }

  // Test case for rotateRh (Euler angles)
  {
    float angleX = EIGEN_PI / 4.0f;
    float angleY = EIGEN_PI / 3.0f;
    float angleZ = EIGEN_PI / 2.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateRh(angleX, angleY, angleZ);
    Eigen::Matrix4f expected;
    expected <<  0.5f, -0.1464466f,  0.8535534f, 0.0f,
                 0.5f,  0.8535534f, -0.1464466f, 0.0f,
                -0.7071068f,  0.5f,  0.5f, 0.0f,
                 0.0f,  0.0f,  0.0f, 1.0f;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateRh (Euler angles) test passed!\n";
    else
      std::cout << "rotateRh (Euler angles) test failed!\n";
  }

  // Test case for rotateRh (axis-angle)
  {
    Eigen::Vector3f axis(1.0f, 1.0f, 1.0f);
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateRh(axis, angle);
    Eigen::Matrix4f expected;
    expected <<  0.804738f, -0.505879f,  0.310617f, 0.0f,
                 0.310617f,  0.804738f, -0.505879f, 0.0f,
                -0.505879f,  0.310617f,  0.804738f, 0.0f,
                 0.0f,       0.0f,       0.0f,      1.0f;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateRh (axis-angle) test passed!\n";
    else
      std::cout << "rotateRh (axis-angle) test failed!\n";
  }

  // Test case for rotateLhX
  {
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateLhX(angle);
    Eigen::Matrix4f expected;
    expected << 1, 0,           0,          0,
                0, std::cos(angle),  std::sin(angle), 0,
                0,-std::sin(angle),  std::cos(angle), 0,
                0, 0,           0,          1;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateLhX test passed!\n";
    else
      std::cout << "rotateLhX test failed!\n";
  }

  // Test case for rotateLhY
  {
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateLhY(angle);
    Eigen::Matrix4f expected;
    expected <<  std::cos(angle), 0,-std::sin(angle), 0,
                 0,          1, 0,          0,
                 std::sin(angle), 0, std::cos(angle), 0,
                 0,          0, 0,          1;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateLhY test passed!\n";
    else
      std::cout << "rotateLhY test failed!\n";
  }

  // Test case for rotateLhZ
  {
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateLhZ(angle);
    Eigen::Matrix4f expected;
    expected << std::cos(angle),  std::sin(angle), 0, 0,
               -std::sin(angle),  std::cos(angle), 0, 0,
                0,           0,          1, 0,
                0,           0,          0, 1;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateLhZ test passed!\n";
    else
      std::cout << "rotateLhZ test failed!\n";
  }

  // Test case for rotateLh (Euler angles)
  {
    float angleX = EIGEN_PI / 4.0f;
    float angleY = EIGEN_PI / 3.0f;
    float angleZ = EIGEN_PI / 2.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateLh(angleX, angleY, angleZ);
    Eigen::Matrix4f expected;
    expected <<  0.5f,  0.1464466f, -0.8535534f, 0.0f,
                 0.5f, -0.8535534f,  0.1464466f, 0.0f,
                 0.7071068f,  0.5f,  0.5f, 0.0f,
                 0.0f,  0.0f,  0.0f, 1.0f;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateLh (Euler angles) test passed!\n";
    else
      std::cout << "rotateLh (Euler angles) test failed!\n";
  }

  // Test case for rotateLh (axis-angle)
  {
    Eigen::Vector3f axis(1.0f, 1.0f, 1.0f);
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateLh(axis, angle);
    Eigen::Matrix4f expected;
    expected <<  0.804738f,  0.505879f, -0.310617f, 0.0f,
                -0.310617f,  0.804738f,  0.505879f, 0.0f,
                 0.505879f, -0.310617f,  0.804738f, 0.0f,
                 0.0f,       0.0f,       0.0f,      1.0f;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateLh (axis-angle) test passed!\n";
    else
      std::cout << "rotateLh (axis-angle) test failed!\n";
  }

  // Test case for lookAtRh
  {
    Eigen::Vector3f eye(0.0f, 0.0f, 5.0f);
    Eigen::Vector3f target(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
    Eigen::Matrix4f viewMatrix = Eigen::Graphics::lookAtRh(eye, target, up);
    Eigen::Matrix4f expected;
    expected << 1, 0,  0, 0,
                0, 1,  0, 0,
                0, 0, -1, 0,
                0, 0, -5, 1;
    if (viewMatrix.isApprox(expected))
      std::cout << "lookAtRh test passed!\n";
    else
      std::cout << "lookAtRh test failed!\n";
  }

  // Test case for lookAtLh
  {
    Eigen::Vector3f eye(0.0f, 0.0f, 5.0f);
    Eigen::Vector3f target(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
    Eigen::Matrix4f viewMatrix = Eigen::Graphics::lookAtLh(eye, target, up);
    Eigen::Matrix4f expected;
    expected << 1, 0,  0, 0,
                0, 1,  0, 0,
                0, 0,  1, 0,
                0, 0, -5, 1;
    if (viewMatrix.isApprox(expected))
      std::cout << "lookAtLh test passed!\n";
    else
      std::cout << "lookAtLh test failed!\n";
  }

  // Test case for lookToRh
  {
    Eigen::Vector3f eye(0.0f, 0.0f, 5.0f);
    Eigen::Vector3f direction(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
    Eigen::Matrix4f viewMatrix = Eigen::Graphics::lookToRh(eye, direction, up);
    Eigen::Matrix4f expected;
    expected << 1, 0,  0, 0,
                0, 1,  0, 0,
                0, 0, -1, 0,
                0, 0, -5, 1;
    if (viewMatrix.isApprox(expected))
      std::cout << "lookToRh test passed!\n";
    else
      std::cout << "lookToRh test failed!\n";
  }

  // Test case for lookToLh
  {
    Eigen::Vector3f eye(0.0f, 0.0f, 5.0f);
    Eigen::Vector3f direction(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
    Eigen::Matrix4f viewMatrix = Eigen::Graphics::lookToLh(eye, direction, up);
    Eigen::Matrix4f expected;
    expected << 1, 0,  0, 0,
                0, 1,  0, 0,
                0, 0,  1, 0,
                0, 0, -5, 1;
    if (viewMatrix.isApprox(expected))
      std::cout << "lookToLh test passed!\n";
    else
      std::cout << "lookToLh test failed!\n";
  }

  // Test case for perspectiveRhNo
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveRhNo(fovY, aspect, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,        0.0f,         0.0f,
                0.0f,       1.3416408f, 0.0f,         0.0f,
                0.0f,       0.0f,       -1.0020020f, -1.0f,
                0.0f,       0.0f,       -0.2002002f,  0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveRhNo test passed!\n";
    else
      std::cout << "perspectiveRhNo test failed!\n";
  }

  // Test case for perspectiveRhZo
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveRhZo(fovY, aspect, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f,        0.0f,
                0.0f,       1.3416408f, 0.0f,        0.0f,
                0.0f,       0.0f,       -1.0020020f, -1.0f,
                0.0f,       0.0f,       -0.1002002f,  0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveRhZo test passed!\n";
    else
      std::cout << "perspectiveRhZo test failed!\n";
  }

  // Test case for perspectiveLhNo
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveLhNo(fovY, aspect, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f,        0.0f,
                0.0f,       1.3416408f, 0.0f,        0.0f,
                0.0f,       0.0f,       1.0020020f,  1.0f,
                0.0f,       0.0f,       -0.2002002f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveLhNo test passed!\n";
    else
      std::cout << "perspectiveLhNo test failed!\n";
  }

  // Test case for perspectiveLhZo
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveLhZo(fovY, aspect, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f,        0.0f,
                0.0f,       1.3416408f, 0.0f,        0.0f,
                0.0f,       0.0f,       1.0020020f,  1.0f,
                0.0f,       0.0f,       -0.1002002f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveLhZo test passed!\n";
    else
      std::cout << "perspectiveLhZo test failed!\n";
  }

  // Test case for perspectiveRhNoInf
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveRhNoInf(fovY, aspect, zNear);
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f, 0.0f,
                0.0f,       1.3416408f, 0.0f, 0.0f,
                0.0f,       0.0f,      -1.0f,-1.0f,
                0.0f,       0.0f,      -0.2f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveRhNoInf test passed!\n";
    else
      std::cout << "perspectiveRhNoInf test failed!\n";
  }

  // Test case for perspectiveRhZoInf
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveRhZoInf(fovY, aspect, zNear);
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f, 0.0f,
                0.0f,       1.3416408f, 0.0f, 0.0f,
                0.0f,       0.0f,      -1.0f,-1.0f,
                0.0f,       0.0f,      -0.1f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveRhZoInf test passed!\n";
    else
      std::cout << "perspectiveRhZoInf test failed!\n";
  }

  // Test case for perspectiveLhNoInf
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveLhNoInf(fovY, aspect, zNear);
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f, 0.0f,
                0.0f,       1.3416408f, 0.0f, 0.0f,
                0.0f,       0.0f,       1.0f, 1.0f,
                0.0f,       0.0f,      -0.2f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveLhNoInf test passed!\n";
    else
      std::cout << "perspectiveLhNoInf test failed!\n";
  }

  // Test case for perspectiveLhZoInf
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveLhZoInf(fovY, aspect, zNear);
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f, 0.0f,
                0.0f,       1.3416408f, 0.0f, 0.0f,
                0.0f,       0.0f,       1.0f, 1.0f,
                0.0f,       0.0f,      -0.1f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveLhZoInf test passed!\n";
    else
      std::cout << "perspectiveLhZoInf test failed!\n";
  }

  // Test case for frustumRhZo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float nearVal = 0.1f;
    float farVal = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::frustumRhZo(left, right, bottom, top, nearVal, farVal);
    Eigen::Matrix4f expected;
    expected << 1.0f, 0.0f,  0.0f,       0.0f,
                0.0f, 1.0f,  0.0f,       0.0f,
                0.0f, 0.0f, -1.0020020f,-1.0f,
                0.0f, 0.0f, -0.1002002f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "frustumRhZo test passed!\n";
    else
      std::cout << "frustumRhZo test failed!\n";
  }

  // Test case for frustumRhNo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float nearVal = 0.1f;
    float farVal = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::frustumRhNo(left, right, bottom, top, nearVal, farVal);
    Eigen::Matrix4f expected;
    expected << 1.0f, 0.0f,  0.0f,       0.0f,
                0.0f, 1.0f,  0.0f,       0.0f,
                0.0f, 0.0f, -1.0020020f,-1.0f,
                0.0f, 0.0f, -0.2002002f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "frustumRhNo test passed!\n";
    else
      std::cout << "frustumRhNo test failed!\n";
  }

  // Test case for frustumLhZo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float nearVal = 0.1f;
    float farVal = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::frustumLhZo(left, right, bottom, top, nearVal, farVal);
    Eigen::Matrix4f expected;
    expected << 1.0f, 0.0f,  0.0f,       0.0f,
                0.0f, 1.0f,  0.0f,       0.0f,
                0.0f, 0.0f,  1.0020020f, 1.0f,
                0.0f, 0.0f, -0.1002002f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "frustumLhZo test passed!\n";
    else
      std::cout << "frustumLhZo test failed!\n";
  }

  // Test case for frustumLhNo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float nearVal = 0.1f;
    float farVal = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::frustumLhNo(left, right, bottom, top, nearVal, farVal);
    Eigen::Matrix4f expected;
    expected << 1.0f, 0.0f,  0.0f,       0.0f,
                0.0f, 1.0f,  0.0f,       0.0f,
                0.0f, 0.0f,  1.0020020f, 1.0f,
                0.0f, 0.0f, -0.2002002f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "frustumLhNo test passed!\n";
    else
      std::cout << "frustumLhNo test failed!\n";
  }

  // Test case for orthoLhZo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoLhZo(left, right, bottom, top, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,  0.0f,
                0.0f,  10.0f, 0.0f,  0.0f,
                0.0f,  0.0f,  0.01f, 0.0f,
                0.0f,  0.0f, -0.001f, 1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoLhZo test passed!\n";
    else
      std::cout << "orthoLhZo test failed!\n";
  }

  // Test case for orthoLhNo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoLhNo(left, right, bottom, top, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,     0.0f,
                0.0f,  10.0f, 0.0f,     0.0f,
                0.0f,  0.0f,  0.02002f, 0.0f,
                0.0f,  0.0f, -1.002f,   1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoLhNo test passed!\n";
    else
      std::cout << "orthoLhNo test failed!\n";
  }

  // Test case for orthoRhZo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoRhZo(left, right, bottom, top, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,   0.0f,
                0.0f,  10.0f, 0.0f,   0.0f,
                0.0f,  0.0f, -0.01f,  0.0f,
                0.0f,  0.0f, -0.001f, 1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoRhZo test passed!\n";
    else
      std::cout << "orthoRhZo test failed!\n";
  }

  // Test case for orthoRhNo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoRhNo(left, right, bottom, top, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,     0.0f,
                0.0f,  10.0f, 0.0f,     0.0f,
                0.0f,  0.0f, -0.02002f, 0.0f,
                0.0f,  0.0f, -1.002f,   1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoRhNo test passed!\n";
    else
      std::cout << "orthoRhNo test failed!\n";
  }

  // Test case for orthoLhZo (width, height)
  {
    float width = 0.2f;
    float height = 0.2f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoLhZo(width, height, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,  0.0f,
                0.0f,  10.0f, 0.0f,  0.0f,
                0.0f,  0.0f,  0.01f, 0.0f,
                0.0f,  0.0f, -0.001f, 1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoLhZo (width, height) test passed!\n";
    else
      std::cout << "orthoLhZo (width, height) test failed!\n";
  }

  // Test case for orthoLhNo (width, height)
  {
    float width = 0.2f;
    float height = 0.2f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoLhNo(width, height, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,     0.0f,
                0.0f,  10.0f, 0.0f,     0.0f,
                0.0f,  0.0f,  0.02002f, 0.0f,
                0.0f,  0.0f, -1.002f,   1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoLhNo (width, height) test passed!\n";
    else
      std::cout << "orthoLhNo (width, height) test failed!\n";
  }

  // Test case for orthoRhZo (width, height)
  {
    float width = 0.2f;
    float height = 0.2f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoRhZo(width, height, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,   0.0f,
                0.0f,  10.0f, 0.0f,   0.0f,
                0.0f,  0.0f, -0.01f,  0.0f,
                0.0f,  0.0f, -0.001f, 1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoRhZo (width, height) test passed!\n";
    else
      std::cout << "orthoRhZo (width, height) test failed!\n";
  }

  // Test case for orthoRhNo (width, height)
  {
    float width = 0.2f;
    float height = 0.2f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoRhNo(width, height, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,     0.0f,
                0.0f,  10.0f, 0.0f,     0.0f,
                0.0f,  0.0f, -0.02002f, 0.0f,
                0.0f,  0.0f, -1.002f,   1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoRhNo (width, height) test passed!\n";
    else
      std::cout << "orthoRhNo (width, height) test failed!\n";
  }

  // Test case for transformPoint
  {
    Eigen::Vector3f point(1.0f, 2.0f, 3.0f);
    Eigen::Matrix4f transformMatrix;
    transformMatrix << 1.0f, 0.0f, 0.0f, 4.0f,
                       0.0f, 1.0f, 0.0f, 5.0f,
                       0.0f, 0.0f, 1.0f, 6.0f,
                       0.0f, 0.0f, 0.0f, 1.0f;
    Eigen::Vector3f transformedPoint = Eigen::Graphics::transformPoint(point, transformMatrix);
    Eigen::Vector3f expected(5.0f, 7.0f, 9.0f);
    if (transformedPoint.isApprox(expected))
      std::cout << "transformPoint test passed!\n";
    else
      std::cout << "transformPoint test failed!\n";
  }

  // Test case for transformVector
  {
    Eigen::Vector3f vector(1.0f, 2.0f, 3.0f);
    Eigen::Matrix4f transformMatrix;
    transformMatrix << 1.0f, 0.0f, 0.0f, 4.0f,
                       0.0f, 1.0f, 0.0f, 5.0f,
                       0.0f, 0.0f, 1.0f, 6.0f,
                       0.0f, 0.0f, 0.0f, 1.0f;
    Eigen::Vector3f transformedVector = Eigen::Graphics::transformVector(vector, transformMatrix);
    Eigen::Vector3f expected(1.0f, 2.0f, 3.0f);
    if (transformedVector.isApprox(expected))
      std::cout << "transformVector test passed!\n";
    else
      std::cout << "transformVector test failed!\n";
  }

  return 0;
}

int main() {
  // Test case for translate
  {
    Eigen::Matrix<float, 4, 4> trans = Eigen::Graphics::translate(1.0f, 2.0f, 3.0f);
    Eigen::Matrix<float, 4, 4> trans1 = Eigen::Graphics::translate(Eigen::Vector3f(1.0f, 2.0f, 3.0f));
    Eigen::Matrix4f expected;
    expected << 1, 0, 0, 1,
                0, 1, 0, 2,
                0, 0, 1, 3,
                0, 0, 0, 1;
    //std::cout << trans << "\n\n";
    //std::cout << trans1 << "\n";
    if (trans.isApprox(expected) && trans1.isApprox(expected))
      std::cout << "translate test passed!\n";
    else
      std::cout << "translate test failed!\n";
  }

  // Test case for addTranslate
  {
    Eigen::Matrix<float, 4, 4, Eigen::ColMajor> matrix = Eigen::Matrix4f::Identity();
    Eigen::Graphics::addTranslate(matrix, 1.0f, 2.0f, 3.0f);
    Eigen::Graphics::addTranslate(matrix, Eigen::Vector3f(1.0f, 2.0f, 3.0f));
    Eigen::Matrix4f expected;
    expected << 1, 0, 0, 2,
                0, 1, 0, 4,
                0, 0, 1, 6,
                0, 0, 0, 1;
    //std::cout << matrix ;

    if (matrix.isApprox(expected))
      std::cout << "addTranslate test passed!\n";
    else
      std::cout << "addTranslate test failed!\n";
  }

  // Test case for setTranslate
  {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Zero();
    Eigen::Graphics::setTranslate(matrix, 1.0f, 2.0f, 3.0f);
    Eigen::Graphics::setTranslate(matrix, Eigen::Vector3f(4.0f, 5.0f, 6.0f));
    Eigen::Matrix4f expected;
    expected << 0, 0, 0, 4,
                0, 0, 0, 5,
                0, 0, 0, 6,
                0, 0, 0, 0;
    //std::cout << matrix ;
    if (matrix.isApprox(expected))
      std::cout << "setTranslate test passed!\n";
    else
      std::cout << "setTranslate test failed!\n";
  }

  // Test case for scale
  {
    Eigen::Matrix4f scaleMatrix = Eigen::Graphics::scale(2.0f, 3.0f, 4.0f);
    Eigen::Matrix4f scaleMatrix1 = Eigen::Graphics::scale(Eigen::Vector3f(2.0f, 3.0f, 4.0f));
    Eigen::Matrix4f expected;
    expected << 2, 0, 0, 0,
                0, 3, 0, 0,
                0, 0, 4, 0,
                0, 0, 0, 1;
    if (scaleMatrix.isApprox(expected) && scaleMatrix1.isApprox(expected))
      std::cout << "scale test passed!\n";
    else
      std::cout << "scale test failed!\n";
  }

  // Test case for rotateRhX
  {
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateRhX(angle);
    Eigen::Matrix4f expected;
    expected << 1, 0,           0,          0,
                0, std::cos(angle), -std::sin(angle), 0,
                0, std::sin(angle),  std::cos(angle), 0,
                0, 0,           0,          1;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateRhX test passed!\n";
    else
      std::cout << "rotateRhX test failed!\n";
  }

  // Test case for rotateRhY
  {
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateRhY(angle);
    Eigen::Matrix4f expected;
    expected <<  std::cos(angle), 0, std::sin(angle), 0,
                 0,          1, 0,          0,
                -std::sin(angle), 0, std::cos(angle), 0,
                 0,          0, 0,          1;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateRhY test passed!\n";
    else
      std::cout << "rotateRhY test failed!\n";
  }

  // Test case for rotateRhZ
  {
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateRhZ(angle);
    Eigen::Matrix4f expected;
    expected << std::cos(angle), -std::sin(angle), 0, 0,
                std::sin(angle),  std::cos(angle), 0, 0,
                0,           0,          1, 0,
                0,           0,          0, 1;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateRhZ test passed!\n";
    else
      std::cout << "rotateRhZ test failed!\n";
  }

  // Test case for g_rotateRh (Euler angles)
  {
    float angleX = EIGEN_PI / 6.0f;
    float angleY = EIGEN_PI / 4.0f;
    float angleZ = EIGEN_PI / 3.0f;
    
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateRh(angleX, angleY, angleZ);
    
    Eigen::Matrix4f expected;
    expected << 0.65974f, -0.435596f, 0.612372f, 0.0f,
                0.75f, 0.433013f, -0.5f, 0.0f,
                -0.0473671f, 0.789149f, 0.612372f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;
    
    //std::cout << rotateMatrix << "\n\n";
    //std::cout << expected << "\n\n";

    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateRh (Euler angles) test passed!\n";
    else
      std::cout << "rotateRh (Euler angles) test failed!\n";
  }

  // Test case for g_rotateRh (Euler angles - vector)
  {
    
    Eigen::Vector3f angles(EIGEN_PI / 6.0f, EIGEN_PI / 4.0f, EIGEN_PI / 3.0f);


    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateRh(angles);
    
    Eigen::Matrix4f expected;
    expected << 0.65974f, -0.435596f, 0.612372f, 0.0f,
                0.75f, 0.433013f, -0.5f, 0.0f,
                -0.0473671f, 0.789149f, 0.612372f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;
    
    //std::cout << rotateMatrix << "\n\n";
    //std::cout << expected << "\n\n";

    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateRh (Euler angles - vector) test passed!\n";
    else
      std::cout << "rotateRh (Euler angles - vector) test failed!\n";
  }
  
  // Test case for g_rotateRh (axis-angle)
  {
    Eigen::Vector3f axis(2.0f, 1.0f, 3.0f);
    float angle = EIGEN_PI / 3.0f;
    
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateRh(axis, angle);
    
    Eigen::Matrix4f expected;
    expected << 0.642857f, 0.765794f, -0.0171693f, 0.0f,
                -0.622936f, 0.535714f, 0.570053f, 0.0f,
                0.445741f, -0.355767f, 0.821429f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;
    
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateRh (axis-angle) test passed!\n";
    else
      std::cout << "rotateRh (axis-angle) test failed!\n";
  }

  // Test case for rotateLhX
  {
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateLhX(angle);
    Eigen::Matrix4f expected;
    expected << 1, 0,           0,          0,
                0, std::cos(angle),  std::sin(angle), 0,
                0,-std::sin(angle),  std::cos(angle), 0,
                0, 0,           0,          1;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateLhX test passed!\n";
    else
      std::cout << "rotateLhX test failed!\n";
  }

  // Test case for rotateLhY
  {
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateLhY(angle);
    Eigen::Matrix4f expected;
    expected <<  std::cos(angle), 0,-std::sin(angle), 0,
                 0,          1, 0,          0,
                 std::sin(angle), 0, std::cos(angle), 0,
                 0,          0, 0,          1;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateLhY test passed!\n";
    else
      std::cout << "rotateLhY test failed!\n";
  }

  // Test case for rotateLhZ
  {
    float angle = EIGEN_PI / 4.0f;
    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateLhZ(angle);
    Eigen::Matrix4f expected;
    expected << std::cos(angle),  std::sin(angle), 0, 0,
               -std::sin(angle),  std::cos(angle), 0, 0,
                0,           0,          1, 0,
                0,           0,          0, 1;
    if (rotateMatrix.isApprox(expected))
      std::cout << "rotateLhZ test passed!\n";
    else
      std::cout << "rotateLhZ test failed!\n";
  }

// Test case for rotateLh (Euler angles)
{
    float angleX = EIGEN_PI / 6.0f;
    float angleY = EIGEN_PI / 4.0f;
    float angleZ = EIGEN_PI / 3.0f;

    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateLh(angleX, angleY, angleZ);

    Eigen::Matrix4f expected;
    expected << 0.0473671f, 0.789149f, -0.612372f, 0.0f,
                -0.75f, 0.433013f, 0.5f, 0.0f,
                0.65974f, 0.435596f, 0.612372f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    //std::cout << rotateMatrix << std::endl;

    if (rotateMatrix.isApprox(expected))
        std::cout << "rotateLh (Euler angles) test passed!\n";
    else
        std::cout << "rotateLh (Euler angles) test failed!\n";
}

// Test case for rotateLh (Euler angles - vector)
{

    Eigen::Vector3f angles(EIGEN_PI / 6.0f, EIGEN_PI / 4.0f, EIGEN_PI / 3.0f);

    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateLh(angles);

    Eigen::Matrix4f expected;
    expected << 0.0473671f, 0.789149f, -0.612372f, 0.0f,
                -0.75f, 0.433013f, 0.5f, 0.0f,
                0.65974f, 0.435596f, 0.612372f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    //std::cout << rotateMatrix << std::endl;

    if (rotateMatrix.isApprox(expected))
        std::cout << "rotateLh (Euler angles - vector) test passed!\n";
    else
        std::cout << "rotateLh (Euler angles - vector) test failed!\n";
}

// Test case for rotateLh (axis-angle)
{
    Eigen::Vector3f axis(2.0f, 1.0f, 3.0f);
    float angle = EIGEN_PI / 3.0f;

    Eigen::Matrix4f rotateMatrix = Eigen::Graphics::rotateLh(axis, angle);

    //std::cout << rotateMatrix << std::endl;

    Eigen::Matrix4f expected;
    expected << 0.642857f, -0.622936f, 0.445741f, 0.0f,
                0.765794f, 0.535714f, -0.355767f, 0.0f,
                -0.0171693f, 0.570053f, 0.821429f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    if (rotateMatrix.isApprox(expected))
        std::cout << "rotateLh (axis-angle) test passed!\n";
    else
        std::cout << "rotateLh (axis-angle) test failed!\n";
}

  // Test case for lookAtRh
  {
    Eigen::Vector3f eye(0.0f, 0.0f, 5.0f);
    Eigen::Vector3f target(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
    Eigen::Matrix4f viewMatrix = Eigen::Graphics::lookAtRh(eye, target, up);
    Eigen::Matrix4f expected;
    expected << 1, 0,  0, 0,
                0, 1,  0, 0,
                0, 0, -1, 0,
                0, 0, -5, 1;
    if (viewMatrix.isApprox(expected))
      std::cout << "lookAtRh test passed!\n";
    else
      std::cout << "lookAtRh test failed!\n";
  }

  // Test case for lookAtLh
  {
    Eigen::Vector3f eye(0.0f, 0.0f, 5.0f);
    Eigen::Vector3f target(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
    Eigen::Matrix4f viewMatrix = Eigen::Graphics::lookAtLh(eye, target, up);
    Eigen::Matrix4f expected;
    expected << 1, 0,  0, 0,
                0, 1,  0, 0,
                0, 0,  1, 0,
                0, 0, -5, 1;
    if (viewMatrix.isApprox(expected))
      std::cout << "lookAtLh test passed!\n";
    else
      std::cout << "lookAtLh test failed!\n";
  }

  // Test case for lookToRh
  {
    Eigen::Vector3f eye(0.0f, 0.0f, 5.0f);
    Eigen::Vector3f direction(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
    Eigen::Matrix4f viewMatrix = Eigen::Graphics::lookToRh(eye, direction, up);
    Eigen::Matrix4f expected;
    expected << 1, 0,  0, 0,
                0, 1,  0, 0,
                0, 0, -1, 0,
                0, 0, -5, 1;
    if (viewMatrix.isApprox(expected))
      std::cout << "lookToRh test passed!\n";
    else
      std::cout << "lookToRh test failed!\n";
  }

  // Test case for lookToLh
  {
    Eigen::Vector3f eye(0.0f, 0.0f, 5.0f);
    Eigen::Vector3f direction(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
    Eigen::Matrix4f viewMatrix = Eigen::Graphics::lookToLh(eye, direction, up);
    Eigen::Matrix4f expected;
    expected << 1, 0,  0, 0,
                0, 1,  0, 0,
                0, 0,  1, 0,
                0, 0, -5, 1;
    if (viewMatrix.isApprox(expected))
      std::cout << "lookToLh test passed!\n";
    else
      std::cout << "lookToLh test failed!\n";
  }

  // Test case for perspectiveRhNo
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveRhNo(fovY, aspect, zNear, zFar);
    // TODO: add Eigen::Graphics::perspectiveRhNo (T fovY, T width, T height, T zNear, T zFar)
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,        0.0f,         0.0f,
                0.0f,       1.3416408f, 0.0f,         0.0f,
                0.0f,       0.0f,       -1.0020020f, -1.0f,
                0.0f,       0.0f,       -0.2002002f,  0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveRhNo test passed!\n";
    else
      std::cout << "perspectiveRhNo test failed!\n";
  }

  // Test case for perspectiveRhZo
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveRhZo(fovY, aspect, zNear, zFar);
    // TODO: add Eigen::Graphics::perspectiveRhZo (T fovY, T width, T height, T zNear, T zFar)
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f,        0.0f,
                0.0f,       1.3416408f, 0.0f,        0.0f,
                0.0f,       0.0f,       -1.0020020f, -1.0f,
                0.0f,       0.0f,       -0.1002002f,  0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveRhZo test passed!\n";
    else
      std::cout << "perspectiveRhZo test failed!\n";
  }

  // Test case for perspectiveLhNo
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveLhNo(fovY, aspect, zNear, zFar);
    // TODO: add Eigen::Graphics::perspectiveLhNo (T fovY, T width, T height, T zNear, T zFar)
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f,        0.0f,
                0.0f,       1.3416408f, 0.0f,        0.0f,
                0.0f,       0.0f,       1.0020020f,  1.0f,
                0.0f,       0.0f,       -0.2002002f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveLhNo test passed!\n";
    else
      std::cout << "perspectiveLhNo test failed!\n";
  }

  // Test case for perspectiveLhZo
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveLhZo(fovY, aspect, zNear, zFar);
    // TODO: add Eigen::Graphics::perspectiveLhZo (T fovY, T width, T height, T zNear, T zFar)
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f,        0.0f,
                0.0f,       1.3416408f, 0.0f,        0.0f,
                0.0f,       0.0f,       1.0020020f,  1.0f,
                0.0f,       0.0f,       -0.1002002f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveLhZo test passed!\n";
    else
      std::cout << "perspectiveLhZo test failed!\n";
  }

  // Test case for perspectiveRhNoInf
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveRhNoInf(fovY, aspect, zNear);
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f, 0.0f,
                0.0f,       1.3416408f, 0.0f, 0.0f,
                0.0f,       0.0f,      -1.0f,-1.0f,
                0.0f,       0.0f,      -0.2f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveRhNoInf test passed!\n";
    else
      std::cout << "perspectiveRhNoInf test failed!\n";
  }

  // Test case for perspectiveRhZoInf
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveRhZoInf(fovY, aspect, zNear);
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f, 0.0f,
                0.0f,       1.3416408f, 0.0f, 0.0f,
                0.0f,       0.0f,      -1.0f,-1.0f,
                0.0f,       0.0f,      -0.1f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveRhZoInf test passed!\n";
    else
      std::cout << "perspectiveRhZoInf test failed!\n";
  }

  // Test case for perspectiveLhNoInf
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveLhNoInf(fovY, aspect, zNear);
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f, 0.0f,
                0.0f,       1.3416408f, 0.0f, 0.0f,
                0.0f,       0.0f,       1.0f, 1.0f,
                0.0f,       0.0f,      -0.2f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveLhNoInf test passed!\n";
    else
      std::cout << "perspectiveLhNoInf test failed!\n";
  }

  // Test case for perspectiveLhZoInf
  {
    float fovY = EIGEN_PI / 4.0f;
    float aspect = 1.7777778f;
    float zNear = 0.1f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::perspectiveLhZoInf(fovY, aspect, zNear);
    Eigen::Matrix4f expected;
    expected << 0.7555555f, 0.0f,       0.0f, 0.0f,
                0.0f,       1.3416408f, 0.0f, 0.0f,
                0.0f,       0.0f,       1.0f, 1.0f,
                0.0f,       0.0f,      -0.1f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "perspectiveLhZoInf test passed!\n";
    else
      std::cout << "perspectiveLhZoInf test failed!\n";
  }

  // Test case for frustumRhZo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float nearVal = 0.1f;
    float farVal = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::frustumRhZo(left, right, bottom, top, nearVal, farVal);
    Eigen::Matrix4f expected;
    expected << 1.0f, 0.0f,  0.0f,       0.0f,
                0.0f, 1.0f,  0.0f,       0.0f,
                0.0f, 0.0f, -1.0020020f,-1.0f,
                0.0f, 0.0f, -0.1002002f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "frustumRhZo test passed!\n";
    else
      std::cout << "frustumRhZo test failed!\n";
  }

  // Test case for frustumRhNo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float nearVal = 0.1f;
    float farVal = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::frustumRhNo(left, right, bottom, top, nearVal, farVal);
    Eigen::Matrix4f expected;
    expected << 1.0f, 0.0f,  0.0f,       0.0f,
                0.0f, 1.0f,  0.0f,       0.0f,
                0.0f, 0.0f, -1.0020020f,-1.0f,
                0.0f, 0.0f, -0.2002002f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "frustumRhNo test passed!\n";
    else
      std::cout << "frustumRhNo test failed!\n";
  }

  // Test case for frustumLhZo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float nearVal = 0.1f;
    float farVal = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::frustumLhZo(left, right, bottom, top, nearVal, farVal);
    Eigen::Matrix4f expected;
    expected << 1.0f, 0.0f,  0.0f,       0.0f,
                0.0f, 1.0f,  0.0f,       0.0f,
                0.0f, 0.0f,  1.0020020f, 1.0f,
                0.0f, 0.0f, -0.1002002f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "frustumLhZo test passed!\n";
    else
      std::cout << "frustumLhZo test failed!\n";
  }

  // Test case for frustumLhNo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float nearVal = 0.1f;
    float farVal = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::frustumLhNo(left, right, bottom, top, nearVal, farVal);
    Eigen::Matrix4f expected;
    expected << 1.0f, 0.0f,  0.0f,       0.0f,
                0.0f, 1.0f,  0.0f,       0.0f,
                0.0f, 0.0f,  1.0020020f, 1.0f,
                0.0f, 0.0f, -0.2002002f, 0.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "frustumLhNo test passed!\n";
    else
      std::cout << "frustumLhNo test failed!\n";
  }

  // Test case for orthoLhZo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoLhZo(left, right, bottom, top, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,  0.0f,
                0.0f,  10.0f, 0.0f,  0.0f,
                0.0f,  0.0f,  0.01f, 0.0f,
                0.0f,  0.0f, -0.001f, 1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoLhZo test passed!\n";
    else
      std::cout << "orthoLhZo test failed!\n";
  }

  // Test case for orthoLhNo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoLhNo(left, right, bottom, top, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,     0.0f,
                0.0f,  10.0f, 0.0f,     0.0f,
                0.0f,  0.0f,  0.02002f, 0.0f,
                0.0f,  0.0f, -1.002f, 1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoLhNo test passed!\n";
    else
      std::cout << "orthoLhNo test failed!\n";
  }

  // Test case for orthoRhZo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoRhZo(left, right, bottom, top, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,   0.0f,
                0.0f,  10.0f, 0.0f,   0.0f,
                0.0f,  0.0f, -0.01f,  0.0f,
                0.0f,  0.0f, -0.001f, 1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoRhZo test passed!\n";
    else
      std::cout << "orthoRhZo test failed!\n";
  }

  // Test case for orthoRhNo
  {
    float left = -0.1f;
    float right = 0.1f;
    float bottom = -0.1f;
    float top = 0.1f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoRhNo(left, right, bottom, top, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,     0.0f,
                0.0f,  10.0f, 0.0f,     0.0f,
                0.0f,  0.0f, -0.02002f, 0.0f,
                0.0f,  0.0f, -1.002f,   1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoRhNo test passed!\n";
    else
      std::cout << "orthoRhNo test failed!\n";
  }

  // Test case for orthoLhZo (width, height)
  {
    float width = 0.2f;
    float height = 0.2f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoLhZo(width, height, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,  0.0f,
                0.0f,  10.0f, 0.0f,  0.0f,
                0.0f,  0.0f,  0.01f, 0.0f,
                0.0f,  0.0f, -0.001f, 1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoLhZo (width, height) test passed!\n";
    else
      std::cout << "orthoLhZo (width, height) test failed!\n";
  }

  // Test case for orthoLhNo (width, height)
  {
    float width = 0.2f;
    float height = 0.2f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoLhNo(width, height, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,     0.0f,
                0.0f,  10.0f, 0.0f,     0.0f,
                0.0f,  0.0f,  0.02002f, 0.0f,
                0.0f,  0.0f, -1.002f,   1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoLhNo (width, height) test passed!\n";
    else
      std::cout << "orthoLhNo (width, height) test failed!\n";
  }

  // Test case for orthoRhZo (width, height)
  {
    float width = 0.2f;
    float height = 0.2f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoRhZo(width, height, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,   0.0f,
                0.0f,  10.0f, 0.0f,   0.0f,
                0.0f,  0.0f, -0.01f,  0.0f,
                0.0f,  0.0f, -0.001f, 1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoRhZo (width, height) test passed!\n";
    else
      std::cout << "orthoRhZo (width, height) test failed!\n";
  }

  // Test case for orthoRhNo (width, height)
  {
    float width = 0.2f;
    float height = 0.2f;
    float zNear = 0.1f;
    float zFar = 100.0f;
    Eigen::Matrix4f projectionMatrix = Eigen::Graphics::orthoRhNo(width, height, zNear, zFar);
    Eigen::Matrix4f expected;
    expected << 10.0f, 0.0f,  0.0f,     0.0f,
                0.0f,  10.0f, 0.0f,     0.0f,
                0.0f,  0.0f, -0.02002f, 0.0f,
                0.0f,  0.0f, -1.002f,   1.0f;
    if (projectionMatrix.isApprox(expected))
      std::cout << "orthoRhNo (width, height) test passed!\n";
    else
      std::cout << "orthoRhNo (width, height) test failed!\n";
  }

  // Test case for transformPoint
  {
    Eigen::Vector3f point(1.0f, 2.0f, 3.0f);
    Eigen::Matrix4f transformMatrix;
    transformMatrix << 1.0f, 0.0f, 0.0f, 4.0f,
                       0.0f, 1.0f, 0.0f, 5.0f,
                       0.0f, 0.0f, 1.0f, 6.0f,
                       0.0f, 0.0f, 0.0f, 1.0f;
    Eigen::Vector3f transformedPoint = Eigen::Graphics::transformPoint(point, transformMatrix);
    Eigen::Vector3f expected(5.0f, 7.0f, 9.0f);
    if (transformedPoint.isApprox(expected))
      std::cout << "transformPoint test passed!\n";
    else
      std::cout << "transformPoint test failed!\n";
  }

  // Test case for transformVector
  {
    Eigen::Vector3f vector(1.0f, 2.0f, 3.0f);
    Eigen::Matrix4f transformMatrix;
    transformMatrix << 1.0f, 0.0f, 0.0f, 4.0f,
                       0.0f, 1.0f, 0.0f, 5.0f,
                       0.0f, 0.0f, 1.0f, 6.0f,
                       0.0f, 0.0f, 0.0f, 1.0f;
    Eigen::Vector3f transformedVector = Eigen::Graphics::transformVector(vector, transformMatrix);
    Eigen::Vector3f expected(1.0f, 2.0f, 3.0f);
    if (transformedVector.isApprox(expected))
      std::cout << "transformVector test passed!\n";
    else
      std::cout << "transformVector test failed!\n";
  }

  return 0;
}

// clang-format on