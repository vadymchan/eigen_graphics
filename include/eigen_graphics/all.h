// all.h

#ifndef EIGEN_GRAPHICS_MODULE_H
#define EIGEN_GRAPHICS_MODULE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Eigen {
namespace Graphics {

// TODO:
// - use Eigen macro
// - Eigen::Matrix<T, 3, 1, Options> - won't work for row major vectors (check static_assert)
// - Add perspectiveDivide and use in transformPoint

template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> translate(T dx, T dy, T dz) {
  Eigen::Matrix<T, 4, 4, Options> translateMat = Eigen::Matrix<T, 4, 4, Options>::Identity();
  if (Options == StorageOptions::ColMajor) {
    // clang-format off
    translateMat <<
      1,   0,   0,   dx,
      0,   1,   0,   dy,
      0,   0,   1,   dz,
      0,   0,   0,   1;
    // clang-format on
  } else if (Options == StorageOptions::RowMajor) {
    // clang-format off
    translateMat <<
      1,   0,   0,   0,
      0,   1,   0,   0,
      0,   0,   1,   0,
      dx,  dy,  dz,  1;
    // clang-format on
  }
  return translateMat;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4, Derived::Options> translate(
    const Eigen::MatrixBase<Derived>& translation) {
  static_assert(Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
                "translate requires a 3x1 translation vector.");

  return translate(translation(0), translation(1), translation(2));
}

// template <StorageOptions Options = StorageOptions::ColMajor, typename T>
// void addTranslate(Eigen::Matrix<T, 4, 4, Options>& matrix, T dx, T dy, T dz) {
//   if (Options == StorageOptions::ColMajor) {
//     matrix(0, 3) += dx;
//     matrix(1, 3) += dy;
//     matrix(2, 3) += dz;
//   } else if (Options == StorageOptions::RowMajor) {
//     matrix(3, 0) += dx;
//     matrix(3, 1) += dy;
//     matrix(3, 2) += dz;
//   }
// }

template <typename Derived>
void addTranslate(Eigen::MatrixBase<Derived>& matrix, typename Derived::Scalar dx, typename Derived::Scalar dy,
                  typename Derived::Scalar dz) {
  static_assert(Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
                "addTranslate requires a 4x4 matrix.");
  if (Derived::Options == Eigen::ColMajor) {
    matrix(0, 3) += dx;
    matrix(1, 3) += dy;
    matrix(2, 3) += dz;
  } else if (Derived::Options == Eigen::RowMajor) {
    matrix(3, 0) += dx;
    matrix(3, 1) += dy;
    matrix(3, 2) += dz;
  }
}

template <typename DerivedMatrix, typename DerivedTranslation>
void addTranslate(Eigen::MatrixBase<DerivedMatrix>& matrix, const Eigen::MatrixBase<DerivedTranslation>& translation) {
  static_assert(DerivedMatrix::RowsAtCompileTime == 4 && DerivedMatrix::ColsAtCompileTime == 4,
                "addTranslate requires a 4x4 matrix.");
  static_assert(DerivedTranslation::RowsAtCompileTime == 3 && DerivedTranslation::ColsAtCompileTime == 1,
                "addTranslate requires a 3x1 translation vector.");

  addTranslate(matrix, translation(0), translation(1), translation(2));
}
// template <typename T, StorageOptions Options = StorageOptions::ColMajor>
// void addTranslate(Eigen::Matrix<T, 4, 4, Options>& matrix, const Eigen::Matrix<T, 3, 1, Options>& translation) {
//   addTranslate<Options, T>(matrix, translation(0), translation(1), translation(2));
// }

template <typename Derived>
void setTranslate(Eigen::MatrixBase<Derived>& matrix, typename Derived::Scalar dx, typename Derived::Scalar dy,
                  typename Derived::Scalar dz) {
  static_assert(Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
                "setTranslate requires a 4x4 matrix.");

  using TranslationVector = Eigen::Matrix<typename Derived::Scalar, 4, 1, Derived::Options>;
  TranslationVector translation(dx, dy, dz, matrix(3, 3));

  if (Derived::Options == Eigen::ColMajor) {
    matrix.col(3) = translation;
  } else if (Derived::Options == Eigen::RowMajor) {
    matrix.row(3) = translation.transpose();
  }
}

template <typename DerivedMatrix, typename DerivedVector>
void setTranslate(Eigen::MatrixBase<DerivedMatrix>& matrix, const Eigen::MatrixBase<DerivedVector>& translation) {
  static_assert(DerivedMatrix::RowsAtCompileTime == 4 && DerivedMatrix::ColsAtCompileTime == 4,
                "setTranslate requires a 4x4 matrix.");
  static_assert(DerivedVector::RowsAtCompileTime == 3 && DerivedVector::ColsAtCompileTime == 1,
                "setTranslate requires a 3x1 translation vector.");

  matrix.template block<3, 1>(0, 3) = translation;
}

template <typename T>
Eigen::Matrix<T, 4, 4> scale(T sx, T sy, T sz) {
  // clang-format off
  return (Eigen::Matrix<T, 4, 4>() << 
      sx, 0,  0,  0,
      0,  sy, 0,  0,
      0,  0,  sz, 0,
      0,  0,  0,  1).finished();
  // clang-format on
}

// template <typename T>
// Eigen::Matrix<T, 4, 4> scale(const Eigen::Matrix<T, 3, 1>& scale) {
//   return scale<T>(scale(0), scale(1), scale(2));
// }

template <typename T>
Eigen::Matrix<T, 4, 4> scale(const Eigen::Matrix<T, 3, 1>& scaleFactors) {
  return scale(scaleFactors(0), scaleFactors(1), scaleFactors(2));
}

// BEGIN: rotation matrix creation functions
// ----------------------------------------------------------------------------

template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> rotateRhX(T angle) {
  const T kCosAngle = std::cos(angle);
  const T kSinAngle = std::sin(angle);

  Eigen::Matrix<T, 4, 4, Options> rotateMat = Eigen::Matrix<T, 4, 4, Options>::Identity();

  if (Options == StorageOptions::ColMajor) {
    // clang-format off
    rotateMat <<
      1,   0,           0,          0,
      0,   kCosAngle,  -kSinAngle,  0,
      0,   kSinAngle,   kCosAngle,  0,
      0,   0,           0,          1;
    // clang-format on
  } else if (Options == StorageOptions::RowMajor) {
    // clang-format off
    rotateMat <<
      1,   0,           0,          0,
      0,   kCosAngle,   kSinAngle,  0,
      0,  -kSinAngle,   kCosAngle,  0,
      0,   0,           0,          1;
    // clang-format on
  }
  return rotateMat;
}

template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> rotateRhY(T angle) {
  const T kCosAngle = std::cos(angle);
  const T kSinAngle = std::sin(angle);

  Eigen::Matrix<T, 4, 4, Options> rotateMat = Eigen::Matrix<T, 4, 4, Options>::Identity();

  if (Options == StorageOptions::ColMajor) {
    // clang-format off
    rotateMat <<
      kCosAngle,   0,   kSinAngle,  0,
      0,           1,   0,          0,
     -kSinAngle,   0,   kCosAngle,  0,
      0,           0,   0,          1;
    // clang-format on
  } else if (Options == StorageOptions::RowMajor) {
    // clang-format off
    rotateMat <<
      kCosAngle,   0,  -kSinAngle,  0,
      0,           1,   0,          0,
      kSinAngle,   0,   kCosAngle,  0,
      0,           0,   0,          1;
    // clang-format on
  }
  return rotateMat;
}

template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> rotateRhZ(T angle) {
  const T kCosAngle = std::cos(angle);
  const T kSinAngle = std::sin(angle);

  Eigen::Matrix<T, 4, 4, Options> rotateMat = Eigen::Matrix<T, 4, 4, Options>::Identity();

  if (Options == StorageOptions::ColMajor) {
    // clang-format off
    rotateMat <<
      kCosAngle,  -kSinAngle,  0,  0,
      kSinAngle,   kCosAngle,  0,  0,
      0,           0,          1,  0,
      0,           0,          0,  1;
    // clang-format on
  } else if (Options == StorageOptions::RowMajor) {
    // clang-format off
    rotateMat <<
      kCosAngle,   kSinAngle,  0,  0,
     -kSinAngle,   kCosAngle,  0,  0,
      0,           0,          1,  0,
      0,           0,          0,  1;
    // clang-format on
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
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> rotateRh(T angleX, T angleY, T angleZ) {
  const T kSX = std::sin(angleX);
  const T kCX = std::cos(angleX);
  const T kSY = std::sin(angleY);
  const T kCY = std::cos(angleY);
  const T kSZ = std::sin(angleZ);
  const T kCZ = std::cos(angleZ);

  Eigen::Matrix<T, 4, 4, Options> rotateMat = Eigen::Matrix<T, 4, 4, Options>::Identity();

  if (Options == StorageOptions::ColMajor) {
    // clang-format off
    rotateMat <<
      kCY * kCZ + kSY * kSX * kSZ,    kCZ * kSY * kSX - kCY * kSZ,    kCX * kSY,   0,
      kCX * kSZ,                      kCX * kCZ,                     -kSX,         0,
      kCY * kSX * kSZ - kSY * kCZ,    kSY * kSZ + kCY * kSX * kCZ,    kCY * kCX,   0,
      0,                              0,                              0,           1;
    // clang-format on
  } else if (Options == StorageOptions::RowMajor) {
    // clang-format off
    rotateMat <<
      kCY * kCZ + kSY * kSX * kSZ,    kCX * kSZ,    kCY * kSX * kSZ - kSY * kCZ,   0,
      kCZ * kSY * kSX - kCY * kSZ,    kCX * kCZ,    kSY * kSZ + kCY * kSX * kCZ,   0,
      kCX * kSY,                     -kSX,          kCY * kCX,                     0,
      0,                              0,            0,                             1;
    // clang-format on
  }
  return rotateMat;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4, Derived::Options> rotateRh(const Eigen::MatrixBase<Derived>& angles) {
  static_assert(Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
                "rotateRh requires a 3x1 angles vector.");

  return rotateRh(angles(0), angles(1), angles(2));
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
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4, Derived::Options> rotateRh(const Eigen::MatrixBase<Derived>& axis,
                                                                         typename Derived::Scalar angle) {
  static_assert(Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
                "rotateRh requires a 3x1 axis vector.");

  using Scalar = typename Derived::Scalar;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 1, Derived::Options>;
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4, Derived::Options>;

  const Scalar kCosAngle = std::cos(angle);
  const Scalar kSinAngle = std::sin(angle);
  const Scalar kOneMinusCos = Scalar(1) - kCosAngle;

  Matrix3 normalizedAxis = axis.normalized();
  const Scalar& x = normalizedAxis(0);
  const Scalar& y = normalizedAxis(1);
  const Scalar& z = normalizedAxis(2);

  Matrix4 rotateMat = Matrix4::Identity();

  if (Derived::Options == Eigen::ColMajor) {
    // clang-format off
    rotateMat <<
      kCosAngle + x*x*kOneMinusCos,    y*x*kOneMinusCos + z*kSinAngle,  z*x*kOneMinusCos - y*kSinAngle,  Scalar(0),
      x*y*kOneMinusCos - z*kSinAngle,  kCosAngle + y*y*kOneMinusCos,    z*y*kOneMinusCos + x*kSinAngle,  Scalar(0),
      x*z*kOneMinusCos + y*kSinAngle,  y*z*kOneMinusCos - x*kSinAngle,  kCosAngle + z*z*kOneMinusCos,    Scalar(0),
      Scalar(0),                       Scalar(0),                       Scalar(0),                       Scalar(1);
    // clang-format on
  } else if (Derived::Options == Eigen::RowMajor) {
    // clang-format off
    rotateMat <<
      kCosAngle + x*x*kOneMinusCos,    x*y*kOneMinusCos - z*kSinAngle,  x*z*kOneMinusCos + y*kSinAngle,  Scalar(0),
      y*x*kOneMinusCos + z*kSinAngle,  kCosAngle + y*y*kOneMinusCos,    y*z*kOneMinusCos - x*kSinAngle,  Scalar(0),
      z*x*kOneMinusCos - y*kSinAngle,  z*y*kOneMinusCos + x*kSinAngle,  kCosAngle + z*z*kOneMinusCos,    Scalar(0),
      Scalar(0),                       Scalar(0),                       Scalar(0),                       Scalar(1);
    // clang-format on
  }

  return rotateMat;
}

template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> rotateLhX(T angle) {
  return rotateRhX<Options, T>(-angle);
}

template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> rotateLhY(T angle) {
  return rotateRhY<Options, T>(-angle);
}

template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> rotateLhZ(T angle) {
  return rotateRhZ<Options, T>(-angle);
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
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> rotateLh(T angleX, T angleY, T angleZ) {
  return rotateRh<Options, T>(-angleX, -angleY, -angleZ);
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4, Derived::Options> rotateLh(const Eigen::MatrixBase<Derived>& angles) {
  static_assert(Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
                "rotateLh requires a 3x1 angles vector.");
  return rotateLh(angles(0), angles(1), angles(2));
}

/**
 * @brief Creates a rotation matrix for rotation around an arbitrary axis in a
 * left-handed coordinate system.
 *
 * Utilizes Rodrigues' rotation formula to generate a 4x4 rotation matrix given
 * an arbitrary axis and rotation angle. The function inverts the angle for
 * adaptation to left-handed coordinate systems but maintains the axis
 * direction.
 *
 * @param axis The 3D vector representing the axis of rotation.
 * @param angle The rotation angle around the axis, in radians.
 *
 * @note This function normalizes the axis of rotation automatically.
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4, Derived::Options> rotateLh(const Eigen::MatrixBase<Derived>& axis,
                                                                         typename Derived::Scalar angle) {
  static_assert(Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
                "rotateLh requires a 3x1 axis vector.");
  return rotateRh<Derived>(axis, -angle);
}

// END: rotation matrix creation functions
// ----------------------------------------------------------------------------

// BEGIN: view matrix creation functions
// ----------------------------------------------------------------------------

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4, Derived::Options> lookAtRh(const Eigen::MatrixBase<Derived>& eye,
                                                                         const Eigen::MatrixBase<Derived>& target,
                                                                         const Eigen::MatrixBase<Derived>& worldUp) {
  static_assert(Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1, "lookAtRh requires 3x1 vectors.");

  using Scalar = typename Derived::Scalar;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1, Derived::Options>;
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4, Derived::Options>;

  Vector3 f = (target - eye).normalized();
  Vector3 r = worldUp.cross(f).normalized();
  Vector3 u = f.cross(r);

  Matrix4 viewMatrix;

  if constexpr (Derived::Options == Eigen::RowMajor) {
    // clang-format off
    viewMatrix <<
      r.x(),   u.x(),     -f.x(),  Scalar(0),
      r.y(),   u.y(),     -f.y(),  Scalar(0),
      r.z(),   u.z(),     -f.z(),  Scalar(0),
      -r.dot(eye),  -u.dot(eye),  -f.dot(eye),  Scalar(1);
    // clang-format on
  } else if constexpr (Derived::Options == Eigen::ColMajor) {
    // clang-format off
    viewMatrix <<
      r.x(),       r.y(),       r.z(),       -r.dot(eye),
      u.x(),       u.y(),       u.z(),       -u.dot(eye),
      -f.x(),      -f.y(),      -f.z(),      -f.dot(eye),
      Scalar(0),   Scalar(0),   Scalar(0),   Scalar(1);
    // clang-format on
  }

  return viewMatrix;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4, Derived::Options> lookAtLh(const Eigen::MatrixBase<Derived>& eye,
                                                                         const Eigen::MatrixBase<Derived>& target,
                                                                         const Eigen::MatrixBase<Derived>& worldUp) {
  static_assert(Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1, "lookAtLh requires 3x1 vectors.");

  using Scalar = typename Derived::Scalar;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1, Derived::Options>;
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4, Derived::Options>;

  Vector3 f = (target - eye).normalized();
  Vector3 r = worldUp.cross(f).normalized();
  Vector3 u = f.cross(r);

  Matrix4 viewMatrix;

  if constexpr (Derived::Options == Eigen::RowMajor) {
    // clang-format off
    viewMatrix <<
      r.x(),   u.x(),     f.x(),  Scalar(0),
      r.y(),   u.y(),     f.y(),  Scalar(0),
      r.z(),   u.z(),     f.z(),  Scalar(0),
      -r.dot(eye),  -u.dot(eye),  -f.dot(eye),  Scalar(1);
    // clang-format on
  } else if constexpr (Derived::Options == Eigen::ColMajor) {
    // clang-format off
    viewMatrix <<
      r.x(),       r.y(),       r.z(),       -r.dot(eye),
      u.x(),       u.y(),       u.z(),       -u.dot(eye),
      f.x(),       f.y(),       f.z(),       -f.dot(eye),
      Scalar(0),   Scalar(0),   Scalar(0),   Scalar(1);
    // clang-format on
  }

  return viewMatrix;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4, Derived::Options> lookToRh(const Eigen::MatrixBase<Derived>& eye,
                                                                         const Eigen::MatrixBase<Derived>& direction,
                                                                         const Eigen::MatrixBase<Derived>& worldUp) {
  static_assert(Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1, "lookToRh requires 3x1 vectors.");

  using Scalar = typename Derived::Scalar;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1, Derived::Options>;
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4, Derived::Options>;

  Vector3 f = direction.normalized();
  Vector3 r = worldUp.cross(f).normalized();
  Vector3 u = f.cross(r);

  Matrix4 viewMatrix;

  if constexpr (Derived::Options == Eigen::RowMajor) {
    // clang-format off
    viewMatrix <<
      r.x(),   u.x(),    -f.x(),  Scalar(0),
      r.y(),   u.y(),    -f.y(),  Scalar(0),
      r.z(),   u.z(),    -f.z(),  Scalar(0),
      -r.dot(eye),  -u.dot(eye),  -f.dot(eye),  Scalar(1);
    // clang-format on
  } else if constexpr (Derived::Options == Eigen::ColMajor) {
    // clang-format off
    viewMatrix <<
      r.x(),       r.y(),       r.z(),       -r.dot(eye),
      u.x(),       u.y(),       u.z(),       -u.dot(eye),
     -f.x(),      -f.y(),      -f.z(),      -f.dot(eye),
      Scalar(0),   Scalar(0),   Scalar(0),   Scalar(1);
    // clang-format on
  }

  return viewMatrix;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 4, Derived::Options> lookToLh(const Eigen::MatrixBase<Derived>& eye,
                                                                         const Eigen::MatrixBase<Derived>& direction,
                                                                         const Eigen::MatrixBase<Derived>& worldUp) {
  static_assert(Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1, "lookToLh requires 3x1 vectors.");

  using Scalar = typename Derived::Scalar;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1, Derived::Options>;
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4, Derived::Options>;

  Vector3 f = direction.normalized();
  Vector3 r = worldUp.cross(f).normalized();
  Vector3 u = f.cross(r);

  Matrix4 viewMatrix;

  if constexpr (Derived::Options == Eigen::RowMajor) {
    // clang-format off
    viewMatrix <<
      r.x(),   u.x(),     f.x(),  Scalar(0),
      r.y(),   u.y(),     f.y(),  Scalar(0),
      r.z(),   u.z(),     f.z(),  Scalar(0),
      -r.dot(eye),  -u.dot(eye),  -f.dot(eye),  Scalar(1);
    // clang-format on
  } else if constexpr (Derived::Options == Eigen::ColMajor) {
    // clang-format off
    viewMatrix <<
      r.x(),       r.y(),       r.z(),       -r.dot(eye),
      u.x(),       u.y(),       u.z(),       -u.dot(eye),
      f.x(),       f.y(),       f.z(),       -f.dot(eye),
      Scalar(0),   Scalar(0),   Scalar(0),   Scalar(1);
    // clang-format on
  }

  return viewMatrix;
}

// END: view matrix creation functions
// ----------------------------------------------------------------------------

// BEGIN: perspective projection creation matrix
// ----------------------------------------------------------------------------

/**
 * Generates a right-handed perspective projection matrix with a depth range of negative one to one.
 *
 * @note RH-NO - Right-Handed, Negative One to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> perspectiveRhNo(T fovY, T aspect, T zNear, T zFar) {
  // validate aspect ratio to prevent division by zero
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));
  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  const T scaleX = static_cast<T>(1) / (tanHalfFovY * aspect);
  const T scaleY = static_cast<T>(1) / (tanHalfFovY);
  // clang-format off
  const T scaleZ          = -(zFar + zNear) / (zFar - zNear);                     // not the same (depends on handness + NO / LO)
  const T translateZ      = -(static_cast<T>(2) * zFar * zNear) / (zFar - zNear); // depends on NO / LO
  const T handednessScale = -static_cast<T>(1);                                   // depends on handness (-z)
  // clang-format on

  Eigen::Matrix<T, 4, 4, Options> perspectiveMatrix;

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),         T(0),
      T(0),       scaleY,     T(0),         T(0),
      T(0),       T(0),       scaleZ,       handednessScale,
      T(0),       T(0),       translateZ,   T(0);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),           T(0),
      T(0),       scaleY,     T(0),           T(0),
      T(0),       T(0),       scaleZ,         translateZ,
      T(0),       T(0),       handednessScale, T(0);
    // clang-format on
  }

  return perspectiveMatrix;
}

/**
 * Generates a right-handed perspective projection matrix with a depth range of zero to one.
 *
 * @note RH-ZO - Right-Handed, Zero to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> perspectiveRhZo(T fovY, T aspect, T zNear, T zFar) {
  // validate aspect ratio to prevent division by zero
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));
  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  const T scaleX = static_cast<T>(1) / (tanHalfFovY * aspect);
  const T scaleY = static_cast<T>(1) / (tanHalfFovY);
  // clang-format off
  const T scaleZ          = -zFar / (zFar - zNear);           // not the same (depends on handness + NO / LO)
  const T translateZ      = -(zFar * zNear) / (zFar - zNear); // depends on NO / LO
  const T handednessScale = -static_cast<T>(1);               // depends on handness (-z)
  // clang-format on

  Eigen::Matrix<T, 4, 4, Options> perspectiveMatrix;

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),         T(0),
      T(0),       scaleY,     T(0),         T(0),
      T(0),       T(0),       scaleZ,       handednessScale,
      T(0),       T(0),       translateZ,   T(0);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),           T(0),
      T(0),       scaleY,     T(0),           T(0),
      T(0),       T(0),       scaleZ,         translateZ,
      T(0),       T(0),       handednessScale, T(0);
    // clang-format on
  }

  return perspectiveMatrix;
}

/**
 * Generates a left-handed perspective projection matrix with a depth range of negative one to one.
 *
 * @note LH-NO - Left-Handed, Negative One to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> perspectiveLhNo(T fovY, T aspect, T zNear, T zFar) {
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));
  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  const T scaleX = static_cast<T>(1) / (tanHalfFovY * aspect);
  const T scaleY = static_cast<T>(1) / (tanHalfFovY);
  // clang-format off
  const T scaleZ          = (zFar + zNear) / (zFar - zNear);                     // not the same (depends on handness + NO / LO)
  const T translateZ      = -(static_cast<T>(2) * zFar * zNear) / (zFar - zNear); // depends on NO / LO
  const T handednessScale = static_cast<T>(1);                                    // depends on handness (z, not -z)
  // clang-format on

  Eigen::Matrix<T, 4, 4, Options> perspectiveMatrix;

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),         T(0),
      T(0),       scaleY,     T(0),         T(0),
      T(0),       T(0),       scaleZ,       handednessScale,
      T(0),       T(0),       translateZ,   T(0);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),           T(0),
      T(0),       scaleY,     T(0),           T(0),
      T(0),       T(0),       scaleZ,         translateZ,
      T(0),       T(0),       handednessScale, T(0);
    // clang-format on
  }

  return perspectiveMatrix;
}

/**
 * Generates a left-handed perspective projection matrix with a depth range of zero to one.
 *
 * @note LH-ZO - Left-Handed, Zero to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> perspectiveLhZo(T fovY, T aspect, T zNear, T zFar) {
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));
  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  const T scaleX = static_cast<T>(1) / (tanHalfFovY * aspect);
  const T scaleY = static_cast<T>(1) / (tanHalfFovY);
  // clang-format off
  const T scaleZ          = zFar / (zFar - zNear);            // not the same (depends on handness + NO / LO)
  const T translateZ      = -(zFar * zNear) / (zFar - zNear); // depends on NO / LO
  const T handednessScale = static_cast<T>(1);                // depends on handness (z, not -z)
  // clang-format on

  Eigen::Matrix<T, 4, 4, Options> perspectiveMatrix;

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),         T(0),
      T(0),       scaleY,     T(0),         T(0),
      T(0),       T(0),       scaleZ,       handednessScale,
      T(0),       T(0),       translateZ,   T(0);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),           T(0),
      T(0),       scaleY,     T(0),           T(0),
      T(0),       T(0),       scaleZ,         translateZ,
      T(0),       T(0),       handednessScale, T(0);
    // clang-format on
  }

  return perspectiveMatrix;
}

/**
 * Generates a right-handed perspective projection matrix based on field of
 * view, width, and height with a depth range of negative one to one.
 *
 * @note RH-NO - Right-Handed, Negative One to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> perspectiveRhNo(T fovY, T width, T height, T zNear, T zFar) {
  auto aspectRatio = width / height;
  auto perspectiveMatrix = perspectiveRhNo<Options, T>(fovY, aspectRatio, zNear, zFar);
  return perspectiveMatrix;
}

/**
 * Generates a right-handed perspective projection matrix based on field of
 * view, width, and height with a depth range of zero to one.
 *
 * @note RH-ZO - Right-Handed, Zero to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> perspectiveRhZo(T fovY, T width, T height, T zNear, T zFar) {
  auto aspectRatio = width / height;
  auto perspectiveMatrix = perspectiveRhZo<Options, T>(fovY, aspectRatio, zNear, zFar);
  return perspectiveMatrix;
}

/**
 * Generates a left-handed perspective projection matrix based on field of view,
 * width, and height with a depth range of negative one to one.
 *
 * @note LH-NO - Left-Handed, Negative One to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> perspectiveLhNo(T fovY, T width, T height, T zNear, T zFar) {
  auto aspectRatio = width / height;
  auto perspectiveMatrix = perspectiveLhNo<Options, T>(fovY, aspectRatio, zNear, zFar);
  return perspectiveMatrix;
}

/**
 * Generates a left-handed perspective projection matrix based on field of view,
 * width, and height with a depth range of zero to one.
 *
 * @note LH-ZO - Left-Handed, Zero to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> perspectiveLhZo(T fovY, T width, T height, T zNear, T zFar) {
  auto aspectRatio = width / height;
  auto perspectiveMatrix = perspectiveLhZo<Options, T>(fovY, aspectRatio, zNear, zFar);
  return perspectiveMatrix;
}

/**
 * Generates a right-handed perspective projection matrix optimized for
 * rendering scenes with an infinite far plane.
 *
 * @note RH-NO-Inf - Right-Handed, Negative One to One depth range, Infinite far
 * plane.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> perspectiveRhNoInf(T fovY, T aspect, T zNear) {
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  const T scaleX = static_cast<T>(1) / (tanHalfFovY * aspect);
  const T scaleY = static_cast<T>(1) / tanHalfFovY;
  // depends on handness (-z)
  const T scaleZ = -static_cast<T>(1);
  // depends on NO / LO
  const T translateZ = -static_cast<T>(2) * zNear;
  // depends on handness (-z)
  const T handednessScale = -static_cast<T>(1);

  Eigen::Matrix<T, 4, 4, Options> perspectiveMatrix;

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),       T(0),
      T(0),       scaleY,     T(0),       T(0),
      T(0),       T(0),       scaleZ,     handednessScale,
      T(0),       T(0),       translateZ, T(0);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),           T(0),
      T(0),       scaleY,     T(0),           T(0),
      T(0),       T(0),       scaleZ,         translateZ,
      T(0),       T(0),       handednessScale, T(0);
    // clang-format on
  }

  return perspectiveMatrix;
}

/**
 * Generates a right-handed perspective projection matrix for scenes with an
 * infinite far plane, optimized for a zero to one depth range.
 *
 * @note RH-ZO-Inf - Right-Handed, Zero to One depth range, Infinite far plane.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> perspectiveRhZoInf(T fovY, T aspect, T zNear) {
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  const T scaleX = static_cast<T>(1) / (tanHalfFovY * aspect);
  const T scaleY = static_cast<T>(1) / tanHalfFovY;
  // depends on handness (-z)
  const T scaleZ = -static_cast<T>(1);
  // depends on NO / LO
  const T translateZ = -zNear;
  // depends on handness (-z)
  const T handednessScale = -static_cast<T>(1);

  Eigen::Matrix<T, 4, 4, Options> perspectiveMatrix;

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),       T(0),
      T(0),       scaleY,     T(0),       T(0),
      T(0),       T(0),       scaleZ,     handednessScale,
      T(0),       T(0),       translateZ, T(0);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),           T(0),
      T(0),       scaleY,     T(0),           T(0),
      T(0),       T(0),       scaleZ,         translateZ,
      T(0),       T(0),       handednessScale, T(0);
    // clang-format on
  }

  return perspectiveMatrix;
}

/**
 * Generates a left-handed perspective projection matrix for rendering with an infinite far plane, using a depth range
 * of negative one to one.
 *
 * @note LH-NO-Inf - Left-Handed, Negative One to One depth range, Infinite far
 * plane.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> perspectiveLhNoInf(T fovY, T aspect, T zNear) {
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  const T scaleX = static_cast<T>(1) / (tanHalfFovY * aspect);
  const T scaleY = static_cast<T>(1) / tanHalfFovY;
  // depends on handness (z, not -z)
  const T scaleZ = static_cast<T>(1);
  // depends on NO / LO
  const T translateZ = -static_cast<T>(2) * zNear;
  // depends on handness (z, not -z)
  const T handednessScale = static_cast<T>(1);

  Eigen::Matrix<T, 4, 4, Options> perspectiveMatrix;

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),       T(0),
      T(0),       scaleY,     T(0),       T(0),
      T(0),       T(0),       scaleZ,     handednessScale,
      T(0),       T(0),       translateZ, T(0);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),           T(0),
      T(0),       scaleY,     T(0),           T(0),
      T(0),       T(0),       scaleZ,         translateZ,
      T(0),       T(0),       handednessScale, T(0);
    // clang-format on
  }

  return perspectiveMatrix;
}

/**
 * Produces a left-handed perspective projection matrix optimized for scenes with an infinite far plane, and a depth
 * range of zero to one.
 *
 * @note LH-ZO-Inf - Left-Handed, Zero to One depth range, Infinite far plane.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> perspectiveLhZoInf(T fovY, T aspect, T zNear) {
  assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));
  const T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

  const T scaleX = static_cast<T>(1) / (tanHalfFovY * aspect);
  const T scaleY = static_cast<T>(1) / tanHalfFovY;
  // depends on handness (z, not -z)
  const T scaleZ = static_cast<T>(1);
  // depends on NO / LO
  const T translateZ = -zNear;
  // depends on handness (z, not -z)
  const T handednessScale = static_cast<T>(1);

  Eigen::Matrix<T, 4, 4, Options> perspectiveMatrix;

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),       T(0),
      T(0),       scaleY,     T(0),       T(0),
      T(0),       T(0),       scaleZ,     handednessScale,
      T(0),       T(0),       translateZ, T(0);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
    perspectiveMatrix <<
      scaleX,     T(0),       T(0),           T(0),
      T(0),       scaleY,     T(0),           T(0),
      T(0),       T(0),       scaleZ,         translateZ,
      T(0),       T(0),       handednessScale, T(0);
    // clang-format on
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
 * 
 * @note RH-ZO - Right-Handed, Zero to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> frustumRhZo(T left, T right, T bottom, T top, T nearVal, T farVal) {
  Eigen::Matrix<T, 4, 4, Options> frustum;
  const T scaleX = (static_cast<T>(2) * nearVal) / (right - left);
  const T scaleY = (static_cast<T>(2) * nearVal) / (top - bottom);
  // depends on NO / ZO
  const T scaleZ = farVal / (nearVal - farVal);

  // depends on handness
  const T offsetX = (right + left) / (right - left);
  // depends on handness
  const T offsetY = (top + bottom) / (top - bottom);

  // depends on NO / ZO
  const T translateZ = -(farVal * nearVal) / (farVal - nearVal);

  // depends on handness
  const T handedness = -static_cast<T>(1);

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
frustum <<
scaleX,     T(0),       T(0),       T(0),
T(0),       scaleY,     T(0),       T(0),
offsetX,    offsetY,    scaleZ,     handedness,
T(0),       T(0),       translateZ, T(0);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
frustum <<
scaleX,     T(0),       offsetX,    T(0),
T(0),       scaleY,     offsetY,    T(0),
T(0),       T(0),       scaleZ,     translateZ,
T(0),       T(0),       handedness, T(0);
    // clang-format on
  }

  return frustum;
}

/**
* Generates a right-handed frustum projection matrix with a depth range of
* negative one to one.
* 
* @note RH-NO - Right-Handed, Negative One to One depth range. 
*/
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> frustumRhNo(T left, T right, T bottom, T top, T nearVal, T farVal) {
  Eigen::Matrix<T, 4, 4, Options> frustum;
  const T scaleX = (static_cast<T>(2) * nearVal) / (right - left);
  const T scaleY = (static_cast<T>(2) * nearVal) / (top - bottom);
  // depends on NO / ZO
  const T scaleZ = -(farVal + nearVal) / (farVal - nearVal);

  // depends on handness
  const T offsetX = (right + left) / (right - left);
  // depends on handness
  const T offsetY = (top + bottom) / (top - bottom);

  // depends on NO / ZO
  const T translateZ = -(static_cast<T>(2) * farVal * nearVal) / (farVal - nearVal);

  // depends on handness
  const T handedness = -static_cast<T>(1);

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
frustum <<
scaleX,     T(0),       T(0),         T(0),
T(0),       scaleY,     T(0),         T(0),
offsetX,    offsetY,    scaleZ,       handedness,
T(0),       T(0),       translateZ,   T(0);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
frustum <<
scaleX,     T(0),       offsetX,      T(0),
T(0),       scaleY,     offsetY,      T(0),
T(0),       T(0),       scaleZ,       translateZ,
T(0),       T(0),       handedness,   T(0);
    // clang-format on
  }

  return frustum;
}

/**
* Generates a left-handed frustum projection matrix with a depth range of zero
* to one.
* 
* @note LH-ZO - Left-Handed, Zero to One depth range. 
*/
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> frustumLhZo(T left, T right, T bottom, T top, T nearVal, T farVal) {
  Eigen::Matrix<T, 4, 4, Options> frustum;
  const T scaleX = (static_cast<T>(2) * nearVal) / (right - left);
  const T scaleY = (static_cast<T>(2) * nearVal) / (top - bottom);
  // depends on NO / ZO + handness
  const T scaleZ = farVal / (farVal - nearVal);

  // depends on handness
  const T offsetX = -(right + left) / (right - left);
  // depends on handness
  const T offsetY = -(top + bottom) / (top - bottom);

  // depends on NO / ZO
  const T translateZ = -(farVal * nearVal) / (farVal - nearVal);

  // depends on handness
  const T handedness = static_cast<T>(1);

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
frustum <<
scaleX,     T(0),       T(0),         T(0),
T(0),       scaleY,     T(0),         T(0),
offsetX,    offsetY,    scaleZ,       handedness,
T(0),       T(0),       translateZ,   T(0);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
frustum <<
scaleX,     T(0),       offsetX,      T(0),
T(0),       scaleY,     offsetY,      T(0),
T(0),       T(0),       scaleZ,       translateZ,
T(0),       T(0),       handedness,   T(0);
    // clang-format on
  }

  return frustum;
}

/**
* Generates a left-handed frustum projection matrix with a depth range of
* negative one to one.
* 
* @note LH-NO - Left-Handed, Negative One to One depth range. 
*/
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> frustumLhNo(T left, T right, T bottom, T top, T nearVal, T farVal) {
  Eigen::Matrix<T, 4, 4, Options> frustum;
  const T scaleX = (static_cast<T>(2) * nearVal) / (right - left);
  const T scaleY = (static_cast<T>(2) * nearVal) / (top - bottom);
  // depends on NO / ZO
  const T scaleZ = (farVal + nearVal) / (farVal - nearVal);

  // depends on handness
  const T offsetX = -(right + left) / (right - left);
  // depends on handness
  const T offsetY = -(top + bottom) / (top - bottom);

  // depends on NO / ZO
  const T translateZ = -(static_cast<T>(2) * farVal * nearVal) / (farVal - nearVal);

  // depends on handness
  const T handedness = static_cast<T>(1);

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
    frustum <<
    scaleX,     T(0),       T(0),         T(0),
    T(0),       scaleY,     T(0),         T(0),
    offsetX,    offsetY,    scaleZ,       handedness,
    T(0),       T(0),       translateZ,   T(0);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
    frustum <<
    scaleX,     T(0),       offsetX,      T(0),
    T(0),       scaleY,     offsetY,      T(0),
    T(0),       T(0),       scaleZ,       translateZ,
    T(0),       T(0),       handedness,   T(0);
    // clang-format on
  }

  return frustum;
}

// END: frustrum (perspective projection matrix that off center) creation functions
// ----------------------------------------------------------------------------

// BEGIN: orthographic projection creation matrix
// ----------------------------------------------------------------------------

// TODO: add ortho functions (LH/RH) that takes (left, right, bottom, top) w/o near / far

/**
 * Generates a left-handed orthographic projection matrix with a depth range of
 * zero to one.
 *
 * @note LH-ZO - Left-Handed, Zero to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> orthoLhZo(T left, T right, T bottom, T top, T zNear, T zFar) {
  Eigen::Matrix<T, 4, 4, Options> orthographicMat;

  const T scaleX = static_cast<T>(2) / (right - left);
  const T scaleY = static_cast<T>(2) / (top - bottom);
  // depends on handness + ZO / NO
  const T scaleZ = static_cast<T>(1) / (zFar - zNear);

  const T translateX = -(right + left) / (right - left);
  const T translateY = -(top + bottom) / (top - bottom);
  // depends on ZO / NO
  const T translateZ = -zNear / (zFar - zNear);

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
    orthographicMat <<
      scaleX,     T(0),     T(0),       T(0),
      T(0),       scaleY,   T(0),       T(0),
      T(0),       T(0),     scaleZ,     T(0),
      translateX, translateY, translateZ, T(1);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
    orthographicMat <<
      scaleX,     T(0),       T(0),         translateX,
      T(0),       scaleY,     T(0),         translateY,
      T(0),       T(0),       scaleZ,       translateZ,
      T(0),       T(0),       T(0),         T(1);
    // clang-format on
  }

  return orthographicMat;
}

/**
 * Generates a left-handed orthographic projection matrix with a depth range of
 * negative one to one.
 *
 * @note LH-NO - Left-Handed, Negative One to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> orthoLhNo(T left, T right, T bottom, T top, T zNear, T zFar) {
  Eigen::Matrix<T, 4, 4, Options> orthographicMat;

  const T scaleX = static_cast<T>(2) / (right - left);
  const T scaleY = static_cast<T>(2) / (top - bottom);
  // depends on handness + ZO / NO
  const T scaleZ = static_cast<T>(2) / (zFar - zNear);

  const T translateX = -(right + left) / (right - left);
  const T translateY = -(top + bottom) / (top - bottom);
  // depends on ZO / NO
  const T translateZ = -(zFar + zNear) / (zFar - zNear);

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
    orthographicMat <<
      scaleX,     T(0),       T(0),       T(0),
      T(0),       scaleY,     T(0),       T(0),
      T(0),       T(0),       scaleZ,     T(0),
      translateX, translateY, translateZ, T(1);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
    orthographicMat <<
      scaleX,     T(0),       T(0),         translateX,
      T(0),       scaleY,     T(0),         translateY,
      T(0),       T(0),       scaleZ,       translateZ,
      T(0),       T(0),       T(0),         T(1);
    // clang-format on
  }

  return orthographicMat;
}

/**
 * Generates a right-handed orthographic projection matrix with a depth range of
 * zero to one.
 *
 * @note RH-ZO - Right-Handed, Zero to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> orthoRhZo(T left, T right, T bottom, T top, T zNear, T zFar) {
  Eigen::Matrix<T, 4, 4, Options> orthographicMat;

  const T scaleX = static_cast<T>(2) / (right - left);
  const T scaleY = static_cast<T>(2) / (top - bottom);
  // depends on handness + ZO / NO
  const T scaleZ = -static_cast<T>(1) / (zFar - zNear);

  const T translateX = -(right + left) / (right - left);
  const T translateY = -(top + bottom) / (top - bottom);
  // depends on ZO / NO
  const T translateZ = -zNear / (zFar - zNear);

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
    orthographicMat <<
      scaleX,     T(0),       T(0),       T(0),
      T(0),       scaleY,     T(0),       T(0),
      T(0),       T(0),       scaleZ,     T(0),
      translateX, translateY, translateZ, T(1);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
    orthographicMat <<
      scaleX,     T(0),       T(0),         translateX,
      T(0),       scaleY,     T(0),         translateY,
      T(0),       T(0),       scaleZ,       translateZ,
      T(0),       T(0),       T(0),         T(1);
    // clang-format on
  }

  return orthographicMat;
}

/**
 * Generates a right-handed orthographic projection matrix with a depth range of
 * negative one to one.
 *
 * @note RH-NO - Right-Handed, Negative One to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> orthoRhNo(T left, T right, T bottom, T top, T zNear, T zFar) {
  Eigen::Matrix<T, 4, 4, Options> orthographicMat;

  const T scaleX = static_cast<T>(2) / (right - left);
  const T scaleY = static_cast<T>(2) / (top - bottom);
  // depends on handness + ZO / NO
  const T scaleZ = -static_cast<T>(2) / (zFar - zNear);

  const T translateX = -(right + left) / (right - left);
  const T translateY = -(top + bottom) / (top - bottom);
  // depends on ZO / NO
  const T translateZ = -(zFar + zNear) / (zFar - zNear);

  if constexpr (Options == StorageOptions::RowMajor) {
    // clang-format off
    orthographicMat <<
      scaleX,     T(0),       T(0),       T(0),
      T(0),       scaleY,     T(0),       T(0),
      T(0),       T(0),       scaleZ,     T(0),
      translateX, translateY, translateZ, T(1);
    // clang-format on
  } else if constexpr (Options == StorageOptions::ColMajor) {
    // clang-format off
    orthographicMat <<
      scaleX,     T(0),       T(0),         translateX,
      T(0),       scaleY,     T(0),         translateY,
      T(0),       T(0),       scaleZ,       translateZ,
      T(0),       T(0),       T(0),         T(1);
    // clang-format on
  }

  return orthographicMat;
}

// TODO: remove code duplication from the functions below

/**
 * Generates a left-handed orthographic projection matrix based on the given width and height,
 * with a depth range of zero to one. This simplifies setting up the projection by directly
 * specifying the viewport dimensions and the near and far clipping planes.
 *
 * @note LH-ZO - Left-Handed, Zero to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> orthoLhZo(T width, T height, T zNear, T zFar) {
  T halfWidth = width / static_cast<T>(2);
  T halfHeight = height / static_cast<T>(2);

  T left = -halfWidth;
  T right = halfWidth;
  T bottom = -halfHeight;
  T top = halfHeight;

  return orthoLhZo<Options, T>(left, right, bottom, top, zNear, zFar);
}

/**
 * Generates a left-handed orthographic projection matrix based on the given width and height,
 * with a depth range of negative one to one. This simplifies setting up the projection by
 * directly specifying the viewport dimensions and the near and far clipping planes.
 *
 * @note LH-NO - Left-Handed, Negative One to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> orthoLhNo(T width, T height, T zNear, T zFar) {
  T halfWidth = width / static_cast<T>(2);
  T halfHeight = height / static_cast<T>(2);

  T left = -halfWidth;
  T right = halfWidth;
  T bottom = -halfHeight;
  T top = halfHeight;

  return orthoLhNo<Options, T>(left, right, bottom, top, zNear, zFar);
}

/**
 * Generates a right-handed orthographic projection matrix based on the given width and height,
 * with a depth range of zero to one. This simplifies setting up the projection by directly
 * specifying the viewport dimensions and the near and far clipping planes.
 *
 * @note RH-ZO - Right-Handed, Zero to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> orthoRhZo(T width, T height, T zNear, T zFar) {
  T halfWidth = width / static_cast<T>(2);
  T halfHeight = height / static_cast<T>(2);

  T left = -halfWidth;
  T right = halfWidth;
  T bottom = -halfHeight;
  T top = halfHeight;

  return orthoRhZo<Options, T>(left, right, bottom, top, zNear, zFar);
}

/**
 * Generates a right-handed orthographic projection matrix based on the given width and height,
 * with a depth range of negative one to one. This simplifies setting up the projection by
 * directly specifying the viewport dimensions and the near and far clipping planes.
 *
 * @note RH-NO - Right-Handed, Negative One to One depth range.
 */
template <StorageOptions Options = StorageOptions::ColMajor, typename T>
Eigen::Matrix<T, 4, 4, Options> orthoRhNo(T width, T height, T zNear, T zFar) {
  T halfWidth = width / static_cast<T>(2);
  T halfHeight = height / static_cast<T>(2);

  T left = -halfWidth;
  T right = halfWidth;
  T bottom = -halfHeight;
  T top = halfHeight;

  return orthoRhNo<Options, T>(left, right, bottom, top, zNear, zFar);
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

#endif  // EIGEN_GRAPHICS_MODULE_H