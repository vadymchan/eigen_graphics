// all.h

#ifndef EIGEN_GRAPHICS_MODULE_H
#define EIGEN_GRAPHICS_MODULE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

 namespace Eigen {
 namespace Graphics {

// TODO:
// - change the order of the parameters to be consistent with the rest of the library (Options first, then T)
// - add a default value for Options (StorageOptions::ColMajor)
// - change the order of if check (first should be Options == StorageOptions::ColMajor)

// clang-format off

 template <StorageOptions Options = Eigen::RowMajor, typename T>
 Eigen::Matrix<T, 4, 4, Options> translate(T dx, T dy, T dz) {
 Eigen::Matrix<T, 4, 4, Options> translateMat = Eigen::Matrix<T, 4, 4, Options>::Identity();
 if (Options == StorageOptions::RowMajor) {
   translateMat <<
     1,   0,   0,   0,
     0,   1,   0,   0,
     0,   0,   1,   0,
     dx,  dy,  dz,  1;
 } else if (Options == StorageOptions::ColMajor) {
   translateMat <<
     1,   0,   0,   dx,
     0,   1,   0,   dy,
     0,   0,   1,   dz,
     0,   0,   0,   1;
 }
 return translateMat;
}

 template <StorageOptions Options = Eigen::RowMajor, typename T>
 Eigen::Matrix<T, 4, 4, Options> translate(const Eigen::Matrix<T, 3, 1, Options>& translation) {
 return translate<T, Options>(translation(0), translation(1), translation(2));
}

 template <typename T, int Options = StorageOptions::RowMajor>
 void addTranslate(Eigen::Matrix<T, 4, 4, Options>& matrix, T dx, T dy, T dz) {
 if (Options == StorageOptions::RowMajor) {
   matrix(3, 0) += dx;
   matrix(3, 1) += dy;
   matrix(3, 2) += dz;
 } else if (Options == StorageOptions::ColMajor) {
   matrix(0, 3) += dx;
   matrix(1, 3) += dy;
   matrix(2, 3) += dz;
 }
}

 template <typename T, int Options = StorageOptions::RowMajor>
 void addTranslate(Eigen::Matrix<T, 4, 4, Options>&    matrix,
                     const Eigen::Matrix<T, 3, 1, Options>& translation) {
 addTranslate(matrix, translation(0), translation(1), translation(2));
}

 template <typename T, int Options = StorageOptions::RowMajor>
 void setTranslate(Eigen::Matrix<T, 4, 4, Options>& matrix, T dx, T dy, T dz) {
 Eigen::Matrix<T, 4, 1, Options> translation(dx, dy, dz, matrix(3, 3));
 if (Options == StorageOptions::RowMajor) {
   matrix.row(3) = translation.transpose();
 } else if (Options == StorageOptions::ColMajor) {
   matrix.col(3) = translation;
 }
}

 template <typename T, int Options = StorageOptions::RowMajor>
 void setTranslate(Eigen::Matrix<T, 4, 4, Options>&    matrix,
                     const Eigen::Matrix<T, 3, 1, Options>& translation) {
 setTranslate(matrix, translation(0), translation(1), translation(2));
}

 template <typename T>
 Eigen::Matrix<T, 4, 4> scale(T sx, T sy, T sz) {
    return (Eigen::Matrix<T, 4, 4>() <<
        sx, 0,  0,  0,
        0,  sy, 0,  0,
        0,  0,  sz, 0,
        0,  0,  0,  1).finished();
}

 template <typename T>
 Eigen::Matrix<T, 4, 4> scale(const Eigen::Matrix<T, 3, 1>& scale) {
 return scale<T>(scale(0), scale(1), scale(2));
}

// BEGIN: rotation matrix creation functions
// ----------------------------------------------------------------------------
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> rotateRhX(T angle) {
  const T kCosAngle = std::cos(angle);
  const T kSinAngle = std::sin(angle);

  Eigen::Matrix<T, 4, 4, Options> rotateMat = Eigen::Matrix<T, 4, 4, Options>::Identity();

  if (Options == StorageOptions::RowMajor) {
    rotateMat << 1,   0,           0,          0,
                 0,   kCosAngle,   kSinAngle,  0,
                 0,  -kSinAngle,   kCosAngle,  0,
                 0,   0,           0,          1;
  } else if (Options == StorageOptions::ColMajor) {
    rotateMat << 1,   0,           0,          0,
                 0,   kCosAngle,  -kSinAngle,  0,
                 0,   kSinAngle,   kCosAngle,  0,
                 0,   0,           0,          1;
  }
  return rotateMat;
 }

 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> rotateRhY(T angle) {
  const T kCosAngle = std::cos(angle);
  const T kSinAngle = std::sin(angle);

  Eigen::Matrix<T, 4, 4, Options> rotateMat = Eigen::Matrix<T, 4, 4, Options>::Identity();

  if (Options == StorageOptions::RowMajor) {
    rotateMat <<  kCosAngle,   0,  -kSinAngle,  0,
                  0,           1,   0,          0,
                  kSinAngle,   0,   kCosAngle,  0,
                  0,           0,   0,          1;
  } else if (Options == StorageOptions::ColMajor) {
    rotateMat <<  kCosAngle,   0,   kSinAngle,  0,
                  0,           1,   0,          0,
                 -kSinAngle,   0,   kCosAngle,  0,
                  0,           0,   0,          1;
  }
  return rotateMat;
 }

 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> rotateRhZ(T angle) {
  const T kCosAngle = std::cos(angle);
  const T kSinAngle = std::sin(angle);

  Eigen::Matrix<T, 4, 4, Options> rotateMat = Eigen::Matrix<T, 4, 4, Options>::Identity();

  if (Options == StorageOptions::RowMajor) {
    rotateMat <<  kCosAngle,   kSinAngle,  0,  0,
                 -kSinAngle,   kCosAngle,  0,  0,
                  0,           0,          1,  0,
                  0,           0,          0,  1;
  } else if (Options == StorageOptions::ColMajor) {
    rotateMat <<  kCosAngle,  -kSinAngle,  0,  0,
                  kSinAngle,   kCosAngle,  0,  0,
                  0,           0,          1,  0,
                  0,           0,          0,  1;
  }
  return rotateMat;
 }

/**
* @brief Creates a rotation matrix in the right-handed coordinate system.
*
* This function generates a 4x4 rotation matrix that represents the combined
* rotation around the X, Y, and Z axes. The rotation order applied is
* Z (roll) -> X (pitch) -> Y (yaw)
*
* @param angleX The rotation angle around the X-axis (roll) in radians.
* @param angleY The rotation angle around the Y-axis (pitch) in radians.
* @param angleZ The rotation angle around the Z-axis (yaw) in radians.
*
* @return A 4x4 rotation matrix in the right-handed coordinate system.
*/
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> rotateRh(T angleX, T angleY, T angleZ) {
 const T kSX = std::sin(angleX);
 const T kCX = std::cos(angleX);
 const T kSY = std::sin(angleY);
 const T kCY = std::cos(angleY);
 const T kSZ = std::sin(angleZ);
 const T kCZ = std::cos(angleZ);

 Eigen::Matrix<T, 4, 4, Options> rotateMat = Eigen::Matrix<T, 4, 4, Options>::Identity();

 if (Options == StorageOptions::RowMajor) {
   rotateMat <<
     kCY * kCZ + kSY * kSX * kSZ,    kCX * kSZ,    kCY * kSX * kSZ - kSY * kCZ,   0,
     kCZ * kSY * kSX - kCY * kSZ,    kCX * kCZ,    kSY * kSZ + kCY * kSX * kCZ,   0,
     kCX * kSY,                     -kSX,          kCY * kCX,                     0,
     0,                              0,            0,                             1;
 } else if (Options == StorageOptions::ColMajor) {
   rotateMat <<
     kCY * kCZ + kSY * kSX * kSZ,    kCZ * kSY * kSX - kCY * kSZ,    kCX * kSY,   0,
     kCX * kSZ,                      kCX * kCZ,                     -kSX,         0,
     kCY * kSX * kSZ - kSY * kCZ,    kSY * kSZ + kCY * kSX * kCZ,    kCY * kCX,   0,
     0,                              0,                              0,           1;
 }
 return rotateMat;
}

 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> rotateRh(const Eigen::Matrix<T, 3, 1, Options>& angles) {
   return rotateRh<T, Options>(angles(0), angles(1), angles(2));
}

/**
* @brief Creates a rotation matrix for rotation around an arbitrary axis in a
* right-handed coordinate system.
*
* Utilizes Rodrigues' rotation formula to generate a 4x4 rotation matrix given
* an arbitrary axis and rotation angle. The axis does not need to be normalized
* as the function will normalize it.
*
* @param axis The 3D vector representing the axis of rotation.
* @param angle The rotation angle around the axis, in radians.
*
* @note This function is designed for right-handed coordinate systems. It
* automatically normalizes the axis of rotation.
*/
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> rotateRh(const Eigen::Matrix<T, 3, 1, Options>& axis, T angle) {
 const T kCosAngle    = std::cos(angle);
 const T kSinAngle    = std::sin(angle);
 const T kOneMinusCos = 1 - kCosAngle;

 Eigen::Matrix<T, 3, 1, Options> normalizedAxis = axis.normalized();
 const T& x = normalizedAxis(0);
 const T& y = normalizedAxis(1);
 const T& z = normalizedAxis(2);

 Eigen::Matrix<T, 4, 4, Options> rotateMat = Eigen::Matrix<T, 4, 4, Options>::Identity();

 if (Options == StorageOptions::RowMajor) {
   rotateMat <<
     kCosAngle  +  x*x*kOneMinusCos,   x*y*kOneMinusCos - z*kSinAngle,   x*z*kOneMinusCos + y*kSinAngle,   0,
     y*x*kOneMinusCos + z*kSinAngle,   kCosAngle  +  y*y*kOneMinusCos,   y*z*kOneMinusCos - x*kSinAngle,   0,
     z*x*kOneMinusCos - y*kSinAngle,   z*y*kOneMinusCos + x*kSinAngle,   kCosAngle  +  z*z*kOneMinusCos,   0,
     0,                                0,                                0,                                1;
 } else if (Options == StorageOptions::ColMajor) {
   rotateMat <<
     kCosAngle  +  x*x*kOneMinusCos,   y*x*kOneMinusCos + z*kSinAngle,   z*x*kOneMinusCos - y*kSinAngle,   0,
     x*y*kOneMinusCos - z*kSinAngle,   kCosAngle  +  y*y*kOneMinusCos,   z*y*kOneMinusCos + x*kSinAngle,   0,
     x*z*kOneMinusCos + y*kSinAngle,   y*z*kOneMinusCos - x*kSinAngle,   kCosAngle  +  z*z*kOneMinusCos,   0,
     0,                                0,                                0,                                1;
 }
 return rotateMat;
}

 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> rotateLhX(T angle) {
 return rotateRhX<T, Options>(-angle);
}

 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> rotateLhY(T angle) {
 return rotateRhY<T, Options>(-angle);
}

 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> rotateLhZ(T angle) {
 return rotateRhZ<T, Options>(-angle);
}


/**
 * @brief Creates a combined rotation matrix around X, Y, and Z axes in the
 * left-handed coordinate system.
 *
 * This function generates a 4x4 rotation matrix that represents the combined
 * rotation around the X, Y, and Z axes (pitch, yaw, and roll) in the order of
 * Z (roll) -> X (pitch) -> Y (yaw) using the right-handed function but inverts
 * the angles for left-handed coordinate system adaptation.
 *
 * @param angleX The rotation angle around the X-axis (pitch) in radians.
 * @param angleY The rotation angle around the Y-axis (yaw) in radians.
 * @param angleZ The rotation angle around the Z-axis (roll) in radians.
 *
 * @return A 4x4 rotation matrix that operates in the left-handed coordinate
 * system.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> rotateLh(T angleX, T angleY, T angleZ) {
  return rotateRh<T, Options>(-angleX, -angleY, -angleZ);
}

 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> rotateLh(const Eigen::Matrix<T, 3, 1, Options>& angles) {
  return rotateLh<T, Options>(angles(0), angles(1), angles(2));
}

/**

@brief Creates a rotation matrix for rotation around an arbitrary axis in a
 left-handed coordinate system.
 Utilizes Rodrigues' rotation formula to generate a 4x4 rotation matrix given
 an arbitrary axis and rotation angle. The function inverts the angle for
 adaptation to left-handed coordinate systems but maintains the axis
 direction.
@param axis The 3D vector representing the axis of rotation.
@param angle The rotation angle around the axis, in radians.
@note This function normalizes the axis of rotation automatically. */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> rotateLh(const Eigen::Matrix<T, 3, 1, Options>& axis, T angle) {
  return rotateRh<T, Options>(axis, -angle);
}
// END: rotation matrix creation functions
// ----------------------------------------------------------------------------

// clang-format on

// BEGIN: view matrix creation functions
// ----------------------------------------------------------------------------

 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> lookAtRh(const Eigen::Matrix<T, 3, 1, Options>& eye,
                                         const Eigen::Matrix<T, 3, 1, Options>& target,
                                         const Eigen::Matrix<T, 3, 1, Options>& worldUp) {
  Eigen::Matrix<T, 3, 1, Options> forward = (target - eye).normalized();
  Eigen::Matrix<T, 3, 1, Options> right = worldUp.cross(forward).normalized();
  Eigen::Matrix<T, 3, 1, Options> up = forward.cross(right);

  Eigen::Matrix<T, 4, 4, Options> viewMatrix = Eigen::Matrix<T, 4, 4, Options>::Identity();

  if (Options == StorageOptions::RowMajor) {
    viewMatrix(0, 0) = right(0);
    viewMatrix(0, 1) = right(1);
    viewMatrix(0, 2) = right(2);
    viewMatrix(3, 0) = -right.dot(eye);

    viewMatrix(1, 0) = up(0);
    viewMatrix(1, 1) = up(1);
    viewMatrix(1, 2) = up(2);
    viewMatrix(3, 1) = -up.dot(eye);

    // forward - depends on handness
    viewMatrix(2, 0) = -forward(0);
    viewMatrix(2, 1) = -forward(1);
    viewMatrix(2, 2) = -forward(2);
    viewMatrix(3, 2) = -forward.dot(eye);
  } else if (Options == StorageOptions::ColMajor) {
    viewMatrix(0, 0) = right(0);
    viewMatrix(1, 0) = right(1);
    viewMatrix(2, 0) = right(2);
    viewMatrix(0, 3) = -right.dot(eye);

    viewMatrix(0, 1) = up(0);
    viewMatrix(1, 1) = up(1);
    viewMatrix(2, 1) = up(2);
    viewMatrix(1, 3) = -up.dot(eye);

    // forward - depends on handness
    viewMatrix(0, 2) = -forward(0);
    viewMatrix(1, 2) = -forward(1);
    viewMatrix(2, 2) = -forward(2);
    viewMatrix(2, 3) = -forward.dot(eye);
  }

  return viewMatrix;
}

 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> lookAtLh(const Eigen::Matrix<T, 3, 1, Options>& eye,
                                         const Eigen::Matrix<T, 3, 1, Options>& target,
                                         const Eigen::Matrix<T, 3, 1, Options>& worldUp) {
  Eigen::Matrix<T, 3, 1, Options> forward = (target - eye).normalized();
  Eigen::Matrix<T, 3, 1, Options> right = worldUp.cross(forward).normalized();
  Eigen::Matrix<T, 3, 1, Options> up = forward.cross(right);

  Eigen::Matrix<T, 4, 4, Options> viewMatrix = Eigen::Matrix<T, 4, 4, Options>::Identity();

  if (Options == StorageOptions::RowMajor) {
    viewMatrix(0, 0) = right(0);
    viewMatrix(0, 1) = right(1);
    viewMatrix(0, 2) = right(2);
    viewMatrix(3, 0) = -right.dot(eye);

    viewMatrix(1, 0) = up(0);
    viewMatrix(1, 1) = up(1);
    viewMatrix(1, 2) = up(2);
    viewMatrix(3, 1) = -up.dot(eye);

    // forward - depends on handness
    viewMatrix(2, 0) = forward(0);
    viewMatrix(2, 1) = forward(1);
    viewMatrix(2, 2) = forward(2);
    viewMatrix(3, 2) = -forward.dot(eye);
  } else if (Options == StorageOptions::ColMajor) {
    viewMatrix(0, 0) = right(0);
    viewMatrix(1, 0) = right(1);
    viewMatrix(2, 0) = right(2);
    viewMatrix(0, 3) = -right.dot(eye);

    viewMatrix(0, 1) = up(0);
    viewMatrix(1, 1) = up(1);
    viewMatrix(2, 1) = up(2);
    viewMatrix(1, 3) = -up.dot(eye);

    // forward - depends on handness
    viewMatrix(0, 2) = forward(0);
    viewMatrix(1, 2) = forward(1);
    viewMatrix(2, 2) = forward(2);
    viewMatrix(2, 3) = -forward.dot(eye);
  }

  return viewMatrix;
}

 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> lookToRh(const Eigen::Matrix<T, 3, 1, Options>& eye,
                                         const Eigen::Matrix<T, 3, 1, Options>& direction,
                                         const Eigen::Matrix<T, 3, 1, Options>& worldUp) {
  Eigen::Matrix<T, 3, 1, Options> forward = direction.normalized();
  Eigen::Matrix<T, 3, 1, Options> right = worldUp.cross(forward).normalized();
  Eigen::Matrix<T, 3, 1, Options> up = forward.cross(right);

  Eigen::Matrix<T, 4, 4, Options> viewMatrix = Eigen::Matrix<T, 4, 4, Options>::Identity();

  if (Options == StorageOptions::RowMajor) {
    viewMatrix(0, 0) = right(0);
    viewMatrix(0, 1) = right(1);
    viewMatrix(0, 2) = right(2);
    viewMatrix(3, 0) = -right.dot(eye);

    viewMatrix(1, 0) = up(0);
    viewMatrix(1, 1) = up(1);
    viewMatrix(1, 2) = up(2);
    viewMatrix(3, 1) = -up.dot(eye);

    // forward - depends on handness
    viewMatrix(2, 0) = -forward(0);
    viewMatrix(2, 1) = -forward(1);
    viewMatrix(2, 2) = -forward(2);
    viewMatrix(3, 2) = -forward.dot(eye);
  } else if (Options == StorageOptions::ColMajor) {
    viewMatrix(0, 0) = right(0);
    viewMatrix(1, 0) = right(1);
    viewMatrix(2, 0) = right(2);
    viewMatrix(0, 3) = -right.dot(eye);

    viewMatrix(0, 1) = up(0);
    viewMatrix(1, 1) = up(1);
    viewMatrix(2, 1) = up(2);
    viewMatrix(1, 3) = -up.dot(eye);

    // forward - depends on handness
    viewMatrix(0, 2) = -forward(0);
    viewMatrix(1, 2) = -forward(1);
    viewMatrix(2, 2) = -forward(2);
    viewMatrix(2, 3) = -forward.dot(eye);
  }

  return viewMatrix;
}

 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> lookToLh(const Eigen::Matrix<T, 3, 1, Options>& eye,
                                         const Eigen::Matrix<T, 3, 1, Options>& direction,
                                         const Eigen::Matrix<T, 3, 1, Options>& worldUp) {
  Eigen::Matrix<T, 3, 1, Options> forward = direction.normalized();
  Eigen::Matrix<T, 3, 1, Options> right = worldUp.cross(forward).normalized();
  Eigen::Matrix<T, 3, 1, Options> up = forward.cross(right);

  Eigen::Matrix<T, 4, 4, Options> viewMatrix = Eigen::Matrix<T, 4, 4, Options>::Identity();

  if (Options == StorageOptions::RowMajor) {
    viewMatrix(0, 0) = right(0);
    viewMatrix(0, 1) = right(1);
    viewMatrix(0, 2) = right(2);
    viewMatrix(3, 0) = -right.dot(eye);

    viewMatrix(1, 0) = up(0);
    viewMatrix(1, 1) = up(1);
    viewMatrix(1, 2) = up(2);
    viewMatrix(3, 1) = -up.dot(eye);

    // forward - depends on handness
    viewMatrix(2, 0) = forward(0);
    viewMatrix(2, 1) = forward(1);
    viewMatrix(2, 2) = forward(2);
    viewMatrix(3, 2) = -forward.dot(eye);
  } else if (Options == StorageOptions::ColMajor) {
    viewMatrix(0, 0) = right(0);
    viewMatrix(1, 0) = right(1);
    viewMatrix(2, 0) = right(2);
    viewMatrix(0, 3) = -right.dot(eye);

    viewMatrix(0, 1) = up(0);
    viewMatrix(1, 1) = up(1);
    viewMatrix(2, 1) = up(2);
    viewMatrix(1, 3) = -up.dot(eye);

    // forward - depends on handness
    viewMatrix(0, 2) = forward(0);
    viewMatrix(1, 2) = forward(1);
    viewMatrix(2, 2) = forward(2);
    viewMatrix(2, 3) = -forward.dot(eye);
  }

  return viewMatrix;
}

// END: view matrix creation functions
// ----------------------------------------------------------------------------

// BEGIN: perspective projection creation matrix
// ----------------------------------------------------------------------------

/**

 Generates a right-handed perspective projection matrix with a depth range of negative one to one.
@note RH-NO - Right-Handed, Negative One to One depth range. */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> perspectiveRhNo(T fovY, T aspect, T zNear, T zFar) {
  // validate aspect ratio to prevent division by zero
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));
  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  Eigen::Matrix<T, 4, 4, Options> perspeciveMatrix = Eigen::Matrix<T, 4, 4, Options>::Zero();
  perspeciveMatrix(0, 0) = static_cast<T>(1) / (tanHalfFovY * aspect);
  perspeciveMatrix(1, 1) = static_cast<T>(1) / (tanHalfFovY);
  perspeciveMatrix(2, 2) = -(zFar + zNear) / (zFar - zNear);  // not the same (depends on handness + NO / LO)
  if (Options == StorageOptions::RowMajor) {
    perspeciveMatrix(3, 2) = -(static_cast<T>(2) * zFar * zNear) / (zFar - zNear);  // depends on NO / LO
    perspeciveMatrix(2, 3) = -static_cast<T>(1);                                    // depends on handness (-z)
  } else if (Options == StorageOptions::ColMajor) {
    perspeciveMatrix(2, 3) = -(static_cast<T>(2) * zFar * zNear) / (zFar - zNear);  // depends on handness (-z)
    perspeciveMatrix(3, 2) = -static_cast<T>(1);                                    // depends on NO / LO
  }
  return perspeciveMatrix;
}

/**

 Generates a right-handed perspective projection matrix with a depth range of zero to one.
@note RH-ZO - Right-Handed, Zero to One depth range. */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> perspectiveRhZo(T fovY, T aspect, T zNear, T zFar) {
  // validate aspect ratio to prevent division by zero
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));
  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  Eigen::Matrix<T, 4, 4, Options> perspeciveMatrix = Eigen::Matrix<T, 4, 4, Options>::Zero();
  perspeciveMatrix(0, 0) = static_cast<T>(1) / (tanHalfFovY * aspect);
  perspeciveMatrix(1, 1) = static_cast<T>(1) / (tanHalfFovY);
  perspeciveMatrix(2, 2) = -zFar / (zFar - zNear);  // not the same (depends on handness + NO / LO)
  if (Options == StorageOptions::RowMajor) {
    perspeciveMatrix(3, 2) = -(zFar * zNear) / (zFar - zNear);  // depends on NO / LO
    perspeciveMatrix(2, 3) = -static_cast<T>(1);                // depends on handness (-z)
  } else if (Options == StorageOptions::ColMajor) {
    perspeciveMatrix(2, 3) = -(zFar * zNear) / (zFar - zNear);  // depends on handness (-z)
    perspeciveMatrix(3, 2) = -static_cast<T>(1);                // depends on NO / LO
  }
  return perspeciveMatrix;
}

/**
 * Generates a left-handed perspective projection matrix with a depth range of negative one to one.
 * @note LH-NO - Left-Handed, Negative One to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> perspectiveLhNo(T fovY, T aspect, T zNear, T zFar) {
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  Eigen::Matrix<T, 4, 4, Options> perspeciveMatrix = Eigen::Matrix<T, 4, 4, Options>::Zero();
  perspeciveMatrix(0, 0) = static_cast<T>(1) / (tanHalfFovY * aspect);
  perspeciveMatrix(1, 1) = static_cast<T>(1) / (tanHalfFovY);
  perspeciveMatrix(2, 2) = (zFar + zNear) / (zFar - zNear);  // not the same (depends on handness + NO / LO)
  if (Options == StorageOptions::RowMajor) {
    perspeciveMatrix(3, 2) = -(static_cast<T>(2) * zFar * zNear) / (zFar - zNear);  // depends on NO / LO
    perspeciveMatrix(2, 3) = static_cast<T>(1);                                     // depends on handness (z, not -z)
  } else if (Options == StorageOptions::ColMajor) {
    perspeciveMatrix(2, 3) = -(static_cast<T>(2) * zFar * zNear) / (zFar - zNear);  // depends on NO / LO
    perspeciveMatrix(3, 2) = static_cast<T>(1);                                     // depends on handness (z, not -z)
  }
  return perspeciveMatrix;
}

/**
 * Generates a left-handed perspective projection matrix with a depth range of zero to one.
 * @note LH-ZO - Left-Handed, Zero to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> perspectiveLhZo(T fovY, T aspect, T zNear, T zFar) {
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  Eigen::Matrix<T, 4, 4, Options> perspeciveMatrix = Eigen::Matrix<T, 4, 4, Options>::Zero();
  perspeciveMatrix(0, 0) = static_cast<T>(1) / (tanHalfFovY * aspect);
  perspeciveMatrix(1, 1) = static_cast<T>(1) / (tanHalfFovY);
  perspeciveMatrix(2, 2) = zFar / (zFar - zNear);  // not the same (depends on handness + NO / LO)
  if (Options == StorageOptions::RowMajor) {
    perspeciveMatrix(3, 2) = -(zFar * zNear) / (zFar - zNear);  // depends on NO / LO
    perspeciveMatrix(2, 3) = static_cast<T>(1);                 // depends on handness (z, not -z)
  } else if (Options == StorageOptions::ColMajor) {
    perspeciveMatrix(2, 3) = -(zFar * zNear) / (zFar - zNear);  // depends on NO / LO
    perspeciveMatrix(3, 2) = static_cast<T>(1);                 // depends on handness (z, not -z)
  }
  return perspeciveMatrix;
}

/**
 * Generates a right-handed perspective projection matrix based on field of
 * view, width, and height with a depth range of negative one to one.
 * @note RH-NO - Right-Handed, Negative One to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> perspectiveRhNo(T fovY, T width, T height, T zNear, T zFar) {
  auto aspectRatio = width / height;
  auto perspectiveMatrix = perspectiveRhNo(fovY, aspectRatio, zNear, zFar);
  return perspectiveMatrix;
}

/**
 * Generates a right-handed perspective projection matrix based on field of
 * view, width, and height with a depth range of zero to one.
 * @note RH-ZO - Right-Handed, Zero to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> perspectiveRhZo(T fovY, T width, T height, T zNear, T zFar) {
  auto aspectRatio = width / height;
  auto perspectiveMatrix = perspectiveRhZo(fovY, aspectRatio, zNear, zFar);
  return perspectiveMatrix;
}

/**
 * Generates a left-handed perspective projection matrix based on field of view,
 * width, and height with a depth range of negative one to one.
 * @note LH-NO - Left-Handed, Negative One to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> perspectiveLhNo(T fovY, T width, T height, T zNear, T zFar) {
  auto aspectRatio = width / height;
  auto perspectiveMatrix = perspectiveLhNo(fovY, aspectRatio, zNear, zFar);
  return perspectiveMatrix;
}

/**
 * Generates a left-handed perspective projection matrix based on field of view,
 * width, and height with a depth range of zero to one.
 * @note LH-ZO - Left-Handed, Zero to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> perspectiveLhZo(T fovY, T width, T height, T zNear, T zFar) {
  auto aspectRatio = width / height;
  auto perspectiveMatrix = perspectiveLhZo(fovY, aspectRatio, zNear, zFar);
  return perspectiveMatrix;
}

/**
 * Generates a right-handed perspective projection matrix optimized for
 * rendering scenes with an infinite far plane.
 * @note RH-NO-Inf - Right-Handed, Negative One to One depth range, Infinite far
 * plane.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> perspectiveRhNoInf(T fovY, T aspect, T zNear) {
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

  // Explanation of matrix structure.
  // We use default perspective projection creation matrices, but for infinite
  // far plane we need to change matrix a little bit. As far approaches
  // infinity, (far / (far - near)) approaches 1, and (near / (far - near))
  // approaches 0. Thus:
  // 1) -(far + near) / (far - near) => -1
  // 2) -(2 * far * near) / (far - near) => -2 * near

  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  Eigen::Matrix<T, 4, 4, Options> perspectiveMatrix = Eigen::Matrix<T, 4, 4, Options>::Zero();
  perspectiveMatrix(0, 0) = static_cast<T>(1) / (tanHalfFovY * aspect);
  perspectiveMatrix(1, 1) = static_cast<T>(1) / tanHalfFovY;
  perspectiveMatrix(2, 2) = -static_cast<T>(1);  // depends on handness (-z)

  if (Options == StorageOptions::RowMajor) {
    perspectiveMatrix(2, 3) = -static_cast<T>(1);          // depends on handness (-z)
    perspectiveMatrix(3, 2) = -static_cast<T>(2) * zNear;  // depends on NO / LO
  } else if (Options == StorageOptions::ColMajor) {
    perspectiveMatrix(3, 2) = -static_cast<T>(1);          // depends on handness (-z)
    perspectiveMatrix(2, 3) = -static_cast<T>(2) * zNear;  // depends on NO / LO
  }
  return perspectiveMatrix;
}

/**
 * Generates a right-handed perspective projection matrix for scenes with an
 * infinite far plane, optimized for a zero to one depth range.
 * @note RH-ZO-Inf - Right-Handed, Zero to One depth range, Infinite far plane.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> perspectiveRhZoInf(T fovY, T aspect, T zNear) {
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

  // Explanation of matrix structure.
  // We use default perspective projection creation matrices, but for infinite
  // far plane we need to change matrix a little bit. As far approaches
  // infinity, (far / (far - near)) approaches 1, and (near / (far - near))
  // approaches 0. Thus:
  // 1) -far / (far - near) => -1
  // 2) -(far * near) / (far - near) => -near

  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  Eigen::Matrix<T, 4, 4, Options> perspectiveMatrix = Eigen::Matrix<T, 4, 4, Options>::Zero();
  perspectiveMatrix(0, 0) = static_cast<T>(1) / (tanHalfFovY * aspect);
  perspectiveMatrix(1, 1) = static_cast<T>(1) / tanHalfFovY;
  perspectiveMatrix(2, 2) = -static_cast<T>(1);  // depends on handness (-z)

  if (Options == StorageOptions::RowMajor) {
    perspectiveMatrix(2, 3) = -static_cast<T>(1);  // depends on handness (-z)
    perspectiveMatrix(3, 2) = -zNear;              // depends on NO / LO
  } else if (Options == StorageOptions::ColMajor) {
    perspectiveMatrix(3, 2) = -static_cast<T>(1);  // depends on handness (-z)
    perspectiveMatrix(2, 3) = -zNear;              // depends on NO / LO
  }
  return perspectiveMatrix;
}

/**
 * Generates a left-handed perspective projection matrix for rendering with an
 * infinite far plane, using a depth range of negative one to one.
 * @note LH-NO-Inf - Left-Handed, Negative One to One depth range, Infinite far
 * plane.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> perspectiveLhNoInf(T fovY, T aspect, T zNear) {
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

  // Explanation of matrix structure.
  // We use default perspective projection creation matrices, but for infinite
  // far plane we need to change matrix a little bit. As far approaches
  // infinity, (far / (far - near)) approaches 1, and (near / (far - near))
  // approaches 0. Thus:
  // 1) (far + near) / (far - near) => 1
  // 2) -(2 * far * near) / (far - near) => -2 * near

  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  Eigen::Matrix<T, 4, 4, Options> perspectiveMatrix = Eigen::Matrix<T, 4, 4, Options>::Zero();
  perspectiveMatrix(0, 0) = static_cast<T>(1) / (tanHalfFovY * aspect);
  perspectiveMatrix(1, 1) = static_cast<T>(1) / tanHalfFovY;
  perspectiveMatrix(2, 2) = static_cast<T>(1);  // depends on handness (z, not -z)

  if (Options == StorageOptions::RowMajor) {
    perspectiveMatrix(2, 3) = static_cast<T>(1);           // depends on handness (z, not -z)
    perspectiveMatrix(3, 2) = -static_cast<T>(2) * zNear;  // depends on NO / LO
  } else if (Options == StorageOptions::ColMajor) {
    perspectiveMatrix(3, 2) = static_cast<T>(1);           // depends on handness (z, not -z)
    perspectiveMatrix(2, 3) = -static_cast<T>(2) * zNear;  // depends on NO / LO
  }
  return perspectiveMatrix;
}

/**
 * Produces a left-handed perspective projection matrix optimized for scenes
 * with an infinite far plane, and a depth range of zero to one.
 * @note LH-ZO-Inf - Left-Handed, Zero to One depth range, Infinite far plane.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> perspectiveLhZoInf(T fovY, T aspect, T zNear) {
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

  // Explanation of matrix structure.
  // We use default perspective projection creation matrices, but for infinite
  // far plane we need to change matrix a little bit. As far approaches
  // infinity, (far / (far - near)) approaches 1, and (near / (far - near))
  // approaches 0. Thus:
  // 1) far / (far - near) => 1
  // 2) -(far * near) / (far - near) => -near

  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  Eigen::Matrix<T, 4, 4, Options> perspectiveMatrix = Eigen::Matrix<T, 4, 4, Options>::Zero();
  perspectiveMatrix(0, 0) = static_cast<T>(1) / (tanHalfFovY * aspect);
  perspectiveMatrix(1, 1) = static_cast<T>(1) / tanHalfFovY;
  perspectiveMatrix(2, 2) = static_cast<T>(1);  // depends on handness (z, not -z)

  if (Options == StorageOptions::RowMajor) {
    perspectiveMatrix(2, 3) = static_cast<T>(1);  // depends on handness (z, not -z)
    perspectiveMatrix(3, 2) = -zNear;             // depends on NO / LO
  } else if (Options == StorageOptions::ColMajor) {
    perspectiveMatrix(3, 2) = static_cast<T>(1);  // depends on handness (z, not -z)
    perspectiveMatrix(2, 3) = -zNear;             // depends on NO / LO
  }

  return perspectiveMatrix;
}

// END: perspective projection creation matrix
// ----------------------------------------------------------------------------

// BEGIN: frustrum (perspective projection matrix that off center) creation functions
// ----------------------------------------------------------------------------

/**
 * Generates a right-handed frustum projection matrix with a depth range of zero
 * to one.
 * @note RH-ZO - Right-Handed, Zero to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> frustumRhZo(T left, T right, T bottom, T top, T nearVal, T farVal) {
  Eigen::Matrix<T, 4, 4, Options> frustrum = Eigen::Matrix<T, 4, 4, Options>::Zero();
  frustrum(0, 0) = (static_cast<T>(2) * nearVal) / (right - left);
  frustrum(1, 1) = (static_cast<T>(2) * nearVal) / (top - bottom);
  frustrum(2, 2) = farVal / (nearVal - farVal);  // depends on NO / ZO

  if (Options == StorageOptions::RowMajor) {
    frustrum(2, 0) = (right + left) / (right - left);           // depends on handness
    frustrum(2, 1) = (top + bottom) / (top - bottom);           // depends on handness
    frustrum(3, 2) = -(farVal * nearVal) / (farVal - nearVal);  // depends on NO / ZO
    frustrum(2, 3) = -static_cast<T>(1);                        // depends on handness
  } else if (Options == StorageOptions::ColMajor) {
    frustrum(0, 2) = (right + left) / (right - left);           // depends on handness
    frustrum(1, 2) = (top + bottom) / (top - bottom);           // depends on handness
    frustrum(2, 3) = -(farVal * nearVal) / (farVal - nearVal);  // depends on NO / ZO
    frustrum(3, 2) = -static_cast<T>(1);                        // depends on handness
  }
  return frustrum;
}

/**
 * Generates a right-handed frustum projection matrix with a depth range of
 * negative one to one.
 * @note RH-NO - Right-Handed, Negative One to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> frustumRhNo(T left, T right, T bottom, T top, T nearVal, T farVal) {
  Eigen::Matrix<T, 4, 4, Options> frustrum = Eigen::Matrix<T, 4, 4, Options>::Zero();
  frustrum(0, 0) = (static_cast<T>(2) * nearVal) / (right - left);
  frustrum(1, 1) = (static_cast<T>(2) * nearVal) / (top - bottom);
  frustrum(2, 2) = -(farVal + nearVal) / (farVal - nearVal);  // depends on NO / ZO

  if (Options == StorageOptions::RowMajor) {
    frustrum(2, 0) = (right + left) / (right - left);                               // depends on handness
    frustrum(2, 1) = (top + bottom) / (top - bottom);                               // depends on handness
    frustrum(3, 2) = -(static_cast<T>(2) * farVal * nearVal) / (farVal - nearVal);  // depends on NO / ZO
    frustrum(2, 3) = -static_cast<T>(1);                                            // depends on handness
  } else if (Options == StorageOptions::ColMajor) {
    frustrum(0, 2) = (right + left) / (right - left);                               // depends on handness
    frustrum(1, 2) = (top + bottom) / (top - bottom);                               // depends on handness
    frustrum(2, 3) = -(static_cast<T>(2) * farVal * nearVal) / (farVal - nearVal);  // depends on NO / ZO
    frustrum(3, 2) = -static_cast<T>(1);                                            // depends on handness
  }
  return frustrum;
}

/**
 * Generates a left-handed frustum projection matrix with a depth range of zero
 * to one.
 * @note LH-ZO - Left-Handed, Zero to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> frustumLhZo(T left, T right, T bottom, T top, T nearVal, T farVal) {
  Eigen::Matrix<T, 4, 4, Options> frustrum = Eigen::Matrix<T, 4, 4, Options>::Zero();
  frustrum(0, 0) = (static_cast<T>(2) * nearVal) / (right - left);
  frustrum(1, 1) = (static_cast<T>(2) * nearVal) / (top - bottom);
  frustrum(2, 2) = farVal / (farVal - nearVal);  // depends on NO / ZO

  if (Options == StorageOptions::RowMajor) {
    frustrum(2, 0) = -(right + left) / (right - left);          // depends on handness
    frustrum(2, 1) = -(top + bottom) / (top - bottom);          // depends on handness
    frustrum(3, 2) = -(farVal * nearVal) / (farVal - nearVal);  // depends on NO / ZO
    frustrum(2, 3) = static_cast<T>(1);                         // depends on handness
  } else if (Options == StorageOptions::ColMajor) {
    frustrum(0, 2) = -(right + left) / (right - left);          // depends on handness
    frustrum(1, 2) = -(top + bottom) / (top - bottom);          // depends on handness
    frustrum(2, 3) = -(farVal * nearVal) / (farVal - nearVal);  // depends on NO / ZO
    frustrum(3, 2) = static_cast<T>(1);                         // depends on handness
  }
  return frustrum;
}

/**
 * Generates a left-handed frustum projection matrix with a depth range of
 * negative one to one.
 * @note LH-NO - Left-Handed, Negative One to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> frustumLhNo(T left, T right, T bottom, T top, T nearVal, T farVal) {
  Eigen::Matrix<T, 4, 4, Options> frustrum = Eigen::Matrix<T, 4, 4, Options>::Zero();
  frustrum(0, 0) = (static_cast<T>(2) * nearVal) / (right - left);
  frustrum(1, 1) = (static_cast<T>(2) * nearVal) / (top - bottom);
  frustrum(2, 2) = (farVal + nearVal) / (farVal - nearVal);  // depends on NO / ZO

  if (Options == StorageOptions::RowMajor) {
    frustrum(2, 0) = -(right + left) / (right - left);                              // depends on handness
    frustrum(2, 1) = -(top + bottom) / (top - bottom);                              // depends on handness
    frustrum(3, 2) = -(static_cast<T>(2) * farVal * nearVal) / (farVal - nearVal);  // depends on NO / ZO
    frustrum(2, 3) = static_cast<T>(1);                                             // depends on handness
  } else if (Options == StorageOptions::ColMajor) {
    frustrum(0, 2) = -(right + left) / (right - left);                              // depends on handness
    frustrum(1, 2) = -(top + bottom) / (top - bottom);                              // depends on handness
    frustrum(2, 3) = -(static_cast<T>(2) * farVal * nearVal) / (farVal - nearVal);  // depends on NO / ZO
    frustrum(3, 2) = static_cast<T>(1);                                             // depends on handness
  }

  return frustrum;
}

// END: frustrum (perspective projection matrix that off center) creation functions
// ----------------------------------------------------------------------------

// BEGIN: orthographic projection creation matrix
// ----------------------------------------------------------------------------

// TODO: add ortho functions (LH/RH) that takes (left, right, bottom, top) w/o near / far

/**
 * Generates a left-handed orthographic projection matrix with a depth range of
 * zero to one.
 * @note LH-ZO - Left-Handed, Zero to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> orthoLhZo(T left, T right, T bottom, T top, T zNear, T zFar) {
  Eigen::Matrix<T, 4, 4, Options> orthographicMat = Eigen::Matrix<T, 4, 4, Options>::Identity();
  orthographicMat(0, 0) = static_cast<T>(2) / (right - left);
  orthographicMat(1, 1) = static_cast<T>(2) / (top - bottom);
  orthographicMat(2, 2) = static_cast<T>(1) / (zFar - zNear);  // depends on handness + ZO / NO
  if (Options == StorageOptions::RowMajor) {
    orthographicMat(3, 0) = -(right + left) / (right - left);
    orthographicMat(3, 1) = -(top + bottom) / (top - bottom);
    orthographicMat(3, 2) = -zNear / (zFar - zNear);  // depends on ZO / NO
  } else if (Options == StorageOptions::ColMajor) {
    orthographicMat(0, 3) = -(right + left) / (right - left);
    orthographicMat(1, 3) = -(top + bottom) / (top - bottom);
    orthographicMat(2, 3) = -zNear / (zFar - zNear);  // depends on ZO / NO
  }
  return orthographicMat;
}

/**
 * Generates a left-handed orthographic projection matrix with a depth range of
 * negative one to one.
 * @note LH-NO - Left-Handed, Negative One to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> orthoLhNo(T left, T right, T bottom, T top, T zNear, T zFar) {
  Eigen::Matrix<T, 4, 4, Options> orthographicMat = Eigen::Matrix<T, 4, 4, Options>::Identity();
  orthographicMat(0, 0) = static_cast<T>(2) / (right - left);
  orthographicMat(1, 1) = static_cast<T>(2) / (top - bottom);
  orthographicMat(2, 2) = static_cast<T>(2) / (zFar - zNear);  // depends on handness + ZO / NO
  if (Options == StorageOptions::RowMajor) {
    orthographicMat(3, 0) = -(right + left) / (right - left);
    orthographicMat(3, 1) = -(top + bottom) / (top - bottom);
    orthographicMat(3, 2) = -(zFar + zNear) / (zFar - zNear);  // depends on ZO / NO
  } else if (Options == StorageOptions::ColMajor) {
    orthographicMat(0, 3) = -(right + left) / (right - left);
    orthographicMat(1, 3) = -(top + bottom) / (top - bottom);
    orthographicMat(2, 3) = -(zFar + zNear) / (zFar - zNear);  // depends on ZO / NO
  }
  return orthographicMat;
}

/**
 * Generates a right-handed orthographic projection matrix with a depth range of
 * zero to one.
 * @note RH-ZO - Right-Handed, Zero to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> orthoRhZo(T left, T right, T bottom, T top, T zNear, T zFar) {
  Eigen::Matrix<T, 4, 4, Options> orthographicMat = Eigen::Matrix<T, 4, 4, Options>::Identity();
  orthographicMat(0, 0) = static_cast<T>(2) / (right - left);
  orthographicMat(1, 1) = static_cast<T>(2) / (top - bottom);
  orthographicMat(2, 2) = -static_cast<T>(1) / (zFar - zNear);  // depends on handness + ZO / NO
  if (Options == StorageOptions::RowMajor) {
    orthographicMat(3, 0) = -(right + left) / (right - left);
    orthographicMat(3, 1) = -(top + bottom) / (top - bottom);
    orthographicMat(3, 2) = -zNear / (zFar - zNear);  // depends on ZO / NO
  } else if (Options == StorageOptions::ColMajor) {
    orthographicMat(0, 3) = -(right + left) / (right - left);
    orthographicMat(1, 3) = -(top + bottom) / (top - bottom);
    orthographicMat(2, 3) = -zNear / (zFar - zNear);  // depends on ZO / NO
  }
  return orthographicMat;
}

/**
 * Generates a right-handed orthographic projection matrix with a depth range of
 * negative one to one.
 * @note RH-NO - Right-Handed, Negative One to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> orthoRhNo(T left, T right, T bottom, T top, T zNear, T zFar) {
  Eigen::Matrix<T, 4, 4, Options> orthographicMat = Eigen::Matrix<T, 4, 4, Options>::Identity();
  orthographicMat(0, 0) = static_cast<T>(2) / (right - left);
  orthographicMat(1, 1) = static_cast<T>(2) / (top - bottom);
  orthographicMat(2, 2) = -static_cast<T>(2) / (zFar - zNear);  // depends on handness + ZO / NO
  if (Options == StorageOptions::RowMajor) {
    orthographicMat(3, 0) = -(right + left) / (right - left);
    orthographicMat(3, 1) = -(top + bottom) / (top - bottom);
    orthographicMat(3, 2) = -(zFar + zNear) / (zFar - zNear);  // depends on ZO / NO
  } else if (Options == StorageOptions::ColMajor) {
    orthographicMat(0, 3) = -(right + left) / (right - left);
    orthographicMat(1, 3) = -(top + bottom) / (top - bottom);
    orthographicMat(2, 3) = -(zFar + zNear) / (zFar - zNear);  // depends on ZO / NO
  }
  return orthographicMat;
}

// TODO: remove code duplication from the functions below

/**
 * Generates a left-handed orthographic projection matrix based on the given width and height,
 * with a depth range of zero to one. This simplifies setting up the projection by directly
 * specifying the viewport dimensions and the near and far clipping planes.
 * @note LH-ZO - Left-Handed, Zero to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> orthoLhZo(T width, T height, T zNear, T zFar) {
  T halfWidth = width / static_cast<T>(2);
  T halfHeight = height / static_cast<T>(2);

  T left = -halfWidth;
  T right = halfWidth;
  T bottom = -halfHeight;
  T top = halfHeight;

  return orthoLhZo<T, Options>(left, right, bottom, top, zNear, zFar);
}

/**
 * Generates a left-handed orthographic projection matrix based on the given width and height,
 * with a depth range of negative one to one. This simplifies setting up the projection by
 * directly specifying the viewport dimensions and the near and far clipping planes.
 * @note LH-NO - Left-Handed, Negative One to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> orthoLhNo(T width, T height, T zNear, T zFar) {
  T halfWidth = width / static_cast<T>(2);
  T halfHeight = height / static_cast<T>(2);

  T left = -halfWidth;
  T right = halfWidth;
  T bottom = -halfHeight;
  T top = halfHeight;

  return orthoLhNo<T, Options>(left, right, bottom, top, zNear, zFar);
}

/**
 * Generates a right-handed orthographic projection matrix based on the given width and height,
 * with a depth range of zero to one. This simplifies setting up the projection by directly
 * specifying the viewport dimensions and the near and far clipping planes.
 * @note RH-ZO - Right-Handed, Zero to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> orthoRhZo(T width, T height, T zNear, T zFar) {
  T halfWidth = width / static_cast<T>(2);
  T halfHeight = height / static_cast<T>(2);

  T left = -halfWidth;
  T right = halfWidth;
  T bottom = -halfHeight;
  T top = halfHeight;

  return orthoRhZo<T, Options>(left, right, bottom, top, zNear, zFar);
}

/**
 * Generates a right-handed orthographic projection matrix based on the given width and height,
 * with a depth range of negative one to one. This simplifies setting up the projection by
 * directly specifying the viewport dimensions and the near and far clipping planes.
 * @note RH-NO - Right-Handed, Negative One to One depth range.
 */
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 4, 4, Options> orthoRhNo(T width, T height, T zNear, T zFar) {
  T halfWidth = width / static_cast<T>(2);
  T halfHeight = height / static_cast<T>(2);

  T left = -halfWidth;
  T right = halfWidth;
  T bottom = -halfHeight;
  T top = halfHeight;

  return orthoRhNo<T, Options>(left, right, bottom, top, zNear, zFar);
}

// END: orthographic projection creation matrix
// ----------------------------------------------------------------------------

/**
 * @brief Transforms a 3D point using a specified transformation matrix and
 * applies perspective division.
 *
 * @param point The point to be transformed. When a Vector object is passed, it
 * is treated as a point with the homogeneous coordinate set to 1.
 *
 * @note This function automatically applies perspective division for points
 * transformed with a perspective projection matrix. The default tolerance is
 * used for perspectiveDivide. Consider adding a parameter to adjust this if
 * needed.
 */
// template <typename T, int Options = StorageOptions::RowMajor>
//// requires std::floatinpoint<T>
// Eigen::Matrix<T, 3, 1, Options> transformPoint(const Eigen::Matrix<T, 3, 1, Options>& point,
//                                                const Eigen::Matrix<T, 4, 4, Options>& matrix) {
//   // TODO: currently in this implementation default tolerance used for
//   // perspectiveDivide. Consider add pararmeter if there will be a need
//   Eigen::Matrix<T, 4, 1, Options> result = Eigen::Matrix<T, 4, 1, Options>::Ones() * point;
//   result = matrix * result;
//   // applied when perspective projection matrix is used
//   if (std::abs(result(3)) > std::numeric_limits<T>::epsilon()) {
//     result /= result(3);
//   }
//   return result.head(3);
// }
 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 3, 1, Options> transformPoint(const Eigen::Matrix<T, 3, 1, Options>& point,
                                                const Eigen::Matrix<T, 4, 4, Options>& matrix) {
   // Convert the 3D point to homogeneous coordinates (4D vector with a 1 appended)
   Eigen::Matrix<T, 4, 1, Options> homogeneousPoint = point.homogeneous();

   // Transform the homogeneous point using the matrix
   Eigen::Matrix<T, 4, 1, Options> transformedHomogeneousPoint = matrix * homogeneousPoint;

   // Convert the transformed homogeneous point back to 3D by performing perspective division
   Eigen::Matrix<T, 3, 1, Options> transformedPoint = transformedHomogeneousPoint.hnormalized();

   return transformedPoint;
 }

 template <typename T, int Options = StorageOptions::RowMajor>
 Eigen::Matrix<T, 3, 1, Options> transformVector(const Eigen::Matrix<T, 3, 1, Options>& vector,
                                                 const Eigen::Matrix<T, 4, 4, Options>& matrix) {
   Eigen::Matrix<T, 4, 1, Options> result = Eigen::Matrix<T, 4, 1, Options>::Zero();
   result.head(3) = vector;
   result = matrix * result;
   return result.head(3);
 }

 }  // namespace Graphics
 }  // namespace Eigen
     result(3, 2) = static_cast<T>(1);
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> frustumLhZo(T left, T right, T bottom, T top, T nearVal, T farVal) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = (static_cast<T>(2) * nearVal) / (right - left);
   result(1, 1) = (static_cast<T>(2) * nearVal) / (top - bottom);
   result(2, 2) = farVal / (farVal - nearVal);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(2, 0) = -(right + left) / (right - left);
     result(2, 1) = -(top + bottom) / (top - bottom);
     result(3, 2) = -(farVal * nearVal) / (farVal - nearVal);
     result(2, 3) = static_cast<T>(1);
   } else {
     result(0, 2) = -(right + left) / (right - left);
     result(1, 2) = -(top + bottom) / (top - bottom);
     result(2, 3) = -(farVal * nearVal) / (farVal - nearVal);
     result(3, 2) = static_cast<T>(1);
   }
   return result;
 }

// Translation
 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> translate(T dx, T dy, T dz) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   if constexpr (Option == StorageOptions::RowMajor) {
     result(3, 0) = dx;
     result(3, 1) = dy;
     result(3, 2) = dz;
   } else {
     result(0, 3) = dx;
     result(1, 3) = dy;
     result(2, 3) = dz;
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> translate(const Matrix<T, 3, 1, Option>& translation) {
   return translate(translation.x(), translation.y(), translation.z());
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 void addTranslate(Matrix<T, 4, 4, Option>& matrix, T dx, T dy, T dz) {
   if constexpr (Option == StorageOptions::RowMajor) {
     matrix(3, 0) += dx;
     matrix(3, 1) += dy;
     matrix(3, 2) += dz;
   } else {
     matrix(0, 3) += dx;
     matrix(1, 3) += dy;
     matrix(2, 3) += dz;
   }
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 void addTranslate(Matrix<T, 4, 4, Option>& matrix, const Matrix<T, 3, 1, Option>& translation) {
   addTranslate(matrix, translation.x(), translation.y(), translation.z());
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 void setTranslate(Matrix<T, 4, 4, Option>& matrix, T dx, T dy, T dz) {
   Matrix<T, 4, 1, Option> translation(dx, dy, dz, matrix(3, 3));
   if constexpr (Option == StorageOptions::RowMajor) {
     matrix.row(3) = translation;
   } else {
     matrix.col(3) = translation;
   }
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 void setTranslate(Matrix<T, 4, 4, Option>& matrix, const Matrix<T, 3, 1, Option>& translation) {
   setTranslate(matrix, translation.x(), translation.y(), translation.z());
 }

// Scale
 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> scale(T sx, T sy, T sz) {
   return Matrix<T, 4, 4, Option>{
     sx, 0, 0, 0,
     0, sy, 0, 0,
     0, 0, sz, 0,
     0, 0, 0, 1
   };
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> scale(const Matrix<T, 3, 1, Option>& scale) {
   return scale(scale.x(), scale.y(), scale.z());
 }

// Rotation
 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> rotateRhX(T angle) {
   T cosAngle = std::cos(angle);
   T sinAngle = std::sin(angle);

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   if constexpr (Option == StorageOptions::RowMajor) {
     result(1, 1) = cosAngle;
     result(1, 2) = sinAngle;
     result(2, 1) = -sinAngle;
     result(2, 2) = cosAngle;
   } else {
     result(1, 1) = cosAngle;
     result(2, 1) = -sinAngle;
     result(1, 2) = sinAngle;
     result(2, 2) = cosAngle;
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> rotateRhY(T angle) {
   T cosAngle = std::cos(angle);
   T sinAngle = std::sin(angle);

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   if constexpr (Option == StorageOptions::RowMajor) {
     result(0, 0) = cosAngle;
     result(0, 2) = -sinAngle;
     result(2, 0) = sinAngle;
     result(2, 2) = cosAngle;
   } else {
     result(0, 0) = cosAngle;
     result(2, 0) = sinAngle;
     result(0, 2) = -sinAngle;
     result(2, 2) = cosAngle;
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> rotateRhZ(T angle) {
   T cosAngle = std::cos(angle);
   T sinAngle = std::sin(angle);

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   if constexpr (Option == StorageOptions::RowMajor) {
     result(0, 0) = cosAngle;
     result(0, 1) = sinAngle;
     result(1, 0) = -sinAngle;
     result(1, 1) = cosAngle;
   } else {
     result(0, 0) = cosAngle;
     result(1, 0) = -sinAngle;
     result(0, 1) = sinAngle;
     result(1, 1) = cosAngle;
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> rotateRh(T angleX, T angleY, T angleZ) {
   T sX = std::sin(angleX);
   T cX = std::cos(angleX);
   T sY = std::sin(angleY);
   T cY = std::cos(angleY);
   T sZ = std::sin(angleZ);
   T cZ = std::cos(angleZ);

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   if constexpr (Option == StorageOptions::RowMajor) {
     result(0, 0) = cY * cZ + sY * sX * sZ;
     result(0, 1) = cX * sZ;
     result(0, 2) = cY * sX * sZ - sY * cZ;
     result(1, 0) = cZ * sY * sX - cY * sZ;
     result(1, 1) = cX * cZ;
     result(1, 2) = sY * sZ + cY * sX * cZ;
     result(2, 0) = cX * sY;
     result(2, 1) = -sX;
     result(2, 2) = cY * cX;
   } else {
     result(0, 0) = cY * cZ + sY * sX * sZ;
     result(1, 0) = cZ * sY * sX - cY * sZ;
     result(2, 0) = cX * sY;
     result(0, 1) = cX * sZ;
     result(1, 1) = cX * cZ;
     result(2, 1) = -sX;
     result(0, 2) = cY * sX * sZ - sY * cZ;
     result(1, 2) = sY * sZ + cY * sX * cZ;
     result(2, 2) = cY * cX;
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> rotateRh(const Matrix<T, 3, 1, Option>& angles) {
   return rotateRh(angles.x(), angles.y(), angles.z());
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> rotateRh(const Matrix<T, 3, 1, Option>& axis, T angle) {
   T cosAngle = std::cos(angle);
   T sinAngle = std::sin(angle);
   T oneMinusCos = 1 - cosAngle;

   Matrix<T, 3, 1, Option> normalizedAxis = axis.normalized();
   T x = normalizedAxis.x();
   T y = normalizedAxis.y();
   T z = normalizedAxis.z();

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   if constexpr (Option == StorageOptions::RowMajor) {
     result(0, 0) = cosAngle + x * x * oneMinusCos;
     result(0, 1) = x * y * oneMinusCos - z * sinAngle;
     result(0, 2) = x * z * oneMinusCos + y * sinAngle;
     result(1, 0) = y * x * oneMinusCos + z * sinAngle;
     result(1, 1) = cosAngle + y * y * oneMinusCos;
     result(1, 2) = y * z * oneMinusCos - x * sinAngle;
     result(2, 0) = z * x * oneMinusCos - y * sinAngle;
     result(2, 1) = z * y * oneMinusCos + x * sinAngle;
     result(2, 2) = cosAngle + z * z * oneMinusCos;
   } else {
     result(0, 0) = cosAngle + x * x * oneMinusCos;
     result(1, 0) = y * x * oneMinusCos + z * sinAngle;
     result(2, 0) = z * x * oneMinusCos - y * sinAngle;
     result(0, 1) = x * y * oneMinusCos - z * sinAngle;
     result(1, 1) = cosAngle + y * y * oneMinusCos;
     result(2, 1) = z * y * oneMinusCos + x * sinAngle;
     result(0, 2) = x * z * oneMinusCos + y * sinAngle;
     result(1, 2) = y * z * oneMinusCos - x * sinAngle;
     result(2, 2) = cosAngle + z * z * oneMinusCos;
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> rotateLhX(T angle) {
   return rotateRhX<T, Option>(-angle);
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> rotateLhY(T angle) {
   return rotateRhY<T, Option>(-angle);
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> rotateLhZ(T angle) {
   return rotateRhZ<T, Option>(-angle);
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> rotateLh(T angleX, T angleY, T angleZ) {
   return rotateRh<T, Option>(-angleX, -angleY, -angleZ);
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> rotateLh(const Matrix<T, 3, 1, Option>& angles) {
   return rotateLh<T, Option>(angles.x(), angles.y(), angles.z());
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> rotateLh(const Matrix<T, 3, 1, Option>& axis, T angle) {
   return rotateRh<T, Option>(axis, -angle);
 }

// Transform
 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 1, Option> perspectiveDivide(const Matrix<T, 4, 1, Option>& vector, T tolerance =
 std::numeric_limits<T>::epsilon()) {
   if (1 != vector.w() && !Eigen::numext::isApprox(vector.w(), 0, tolerance)) {
     return vector / vector.w();
   }
   return vector;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 3, 1, Option> transformPoint(const Matrix<T, 3, 1, Option>& point, const Matrix<T, 4, 4, Option>& matrix) {
   Matrix<T, 4, 1, Option> result = Matrix<T, 4, 1, Option>(point.x(), point.y(), point.z(), 1);
   result = matrix * result;
   result = perspectiveDivide(result);
   return result.head<3>();
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 3, 1, Option> transformVector(const Matrix<T, 3, 1, Option>& vector, const Matrix<T, 4, 4, Option>& matrix)
 {
   Matrix<T, 4, 1, Option> result = Matrix<T, 4, 1, Option>(vector.x(), vector.y(), vector.z(), 0);
   result = matrix * result;
   return result.head<3>();
 }

 } // namespace Graphics
 } // namespace Eigen

 #endif // EIGEN_GRAPHICS_MODULE_H