// eigen_graphics.h

#ifndef EIGEN_GRAPHICS_MODULE_H
#define EIGEN_GRAPHICS_MODULE_H

#include <Eigen/Core>

namespace Eigen {
namespace Graphics {

// Matrix creation functions

// View matrix
template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> lookAtRh(const Matrix<T, 3, 1, Option>& eye,
                                 const Matrix<T, 3, 1, Option>& target,
                                 const Matrix<T, 3, 1, Option>& up);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> lookAtLh(const Matrix<T, 3, 1, Option>& eye,
                                 const Matrix<T, 3, 1, Option>& target,
                                 const Matrix<T, 3, 1, Option>& up);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> lookToRh(const Matrix<T, 3, 1, Option>& eye,
                                 const Matrix<T, 3, 1, Option>& direction,
                                 const Matrix<T, 3, 1, Option>& up);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> lookToLh(const Matrix<T, 3, 1, Option>& eye,
                                 const Matrix<T, 3, 1, Option>& direction,
                                 const Matrix<T, 3, 1, Option>& up);

// Projection matrix
template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> perspectiveRhNo(T fovY, T aspect, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> perspectiveRhZo(T fovY, T aspect, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> perspectiveLhNo(T fovY, T aspect, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> perspectiveLhZo(T fovY, T aspect, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> perspectiveRhNo(T fovY, T width, T height, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> perspectiveRhZo(T fovY, T width, T height, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> perspectiveLhNo(T fovY, T width, T height, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> perspectiveLhZo(T fovY, T width, T height, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> perspectiveRhNoInf(T fovY, T aspect, T zNear);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> perspectiveRhZoInf(T fovY, T aspect, T zNear);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> perspectiveLhNoInf(T fovY, T aspect, T zNear);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> perspectiveLhZoInf(T fovY, T aspect, T zNear);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> orthoRhNo(T left, T right, T bottom, T top, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> orthoRhZo(T left, T right, T bottom, T top, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> orthoLhNo(T left, T right, T bottom, T top, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> orthoLhZo(T left, T right, T bottom, T top, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> orthoRhNo(T width, T height, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> orthoRhZo(T width, T height, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> orthoLhNo(T width, T height, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> orthoLhZo(T width, T height, T zNear, T zFar);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> frustumRhNo(T left, T right, T bottom, T top, T nearVal, T farVal);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> frustumRhZo(T left, T right, T bottom, T top, T nearVal, T farVal);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> frustumLhNo(T left, T right, T bottom, T top, T nearVal, T farVal);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> frustumLhZo(T left, T right, T bottom, T top, T nearVal, T farVal);

// Translation
template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> translate(T dx, T dy, T dz);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> translate(const Matrix<T, 3, 1, Option>& translation);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
void addTranslate(Matrix<T, 4, 4, Option>& matrix, T dx, T dy, T dz);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
void addTranslate(Matrix<T, 4, 4, Option>& matrix, const Matrix<T, 3, 1, Option>& translation);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
void setTranslate(Matrix<T, 4, 4, Option>& matrix, T dx, T dy, T dz);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
void setTranslate(Matrix<T, 4, 4, Option>& matrix, const Matrix<T, 3, 1, Option>& translation);

// Scale
template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> scale(T sx, T sy, T sz);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> scale(const Matrix<T, 3, 1, Option>& scale);

// Rotation
template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> rotateRhX(T angle);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> rotateRhY(T angle);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> rotateRhZ(T angle);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> rotateRh(T angleX, T angleY, T angleZ);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> rotateRh(const Matrix<T, 3, 1, Option>& angles);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> rotateRh(const Matrix<T, 3, 1, Option>& axis, T angle);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> rotateLhX(T angle);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> rotateLhY(T angle);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> rotateLhZ(T angle);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> rotateLh(T angleX, T angleY, T angleZ);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> rotateLh(const Matrix<T, 3, 1, Option>& angles);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 4, Option> rotateLh(const Matrix<T, 3, 1, Option>& axis, T angle);

// Transform
template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 4, 1, Option> perspectiveDivide(const Matrix<T, 4, 1, Option>& vector, T tolerance = std::numeric_limits<T>::epsilon());

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 3, 1, Option> transformPoint(const Matrix<T, 3, 1, Option>& point, const Matrix<T, 4, 4, Option>& matrix);

template <typename T, StorageOptions Option = StorageOptions::RowMajor>
Matrix<T, 3, 1, Option> transformVector(const Matrix<T, 3, 1, Option>& vector, const Matrix<T, 4, 4, Option>& matrix);

} // namespace Graphics
} // namespace Eigen

#endif // EIGEN_GRAPHICS_MODULE_H