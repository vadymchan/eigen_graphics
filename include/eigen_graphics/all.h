// eigen_graphics.h

#ifndef EIGEN_GRAPHICS_MODULE_H
#define EIGEN_GRAPHICS_MODULE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

 namespace Eigen {
 namespace Graphics {

// Matrix creation functions

// View matrix
 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> lookAtRh(const Matrix<T, 3, 1, Option>& eye,
                                  const Matrix<T, 3, 1, Option>& target,
                                  const Matrix<T, 3, 1, Option>& up) {
   Matrix<T, 3, 1, Option> forward = (target - eye).normalized();
   Matrix<T, 3, 1, Option> right = up.cross(forward).normalized();
   Matrix<T, 3, 1, Option> upVec = forward.cross(right);

   Matrix<T, 4, 4, Option> viewMatrix = Matrix<T, 4, 4, Option>::Identity();

   if constexpr (Option == StorageOptions::RowMajor) {
     viewMatrix.row(0).head<3>() = right.transpose();
     viewMatrix.row(1).head<3>() = upVec.transpose();
     viewMatrix.row(2).head<3>() = -forward.transpose();
     viewMatrix.row(3).head<3>() = -right.dot(eye), -upVec.dot(eye), -forward.dot(eye);
   } else {
     viewMatrix.col(0).head<3>() = right;
     viewMatrix.col(1).head<3>() = upVec;
     viewMatrix.col(2).head<3>() = -forward;
     viewMatrix.col(3).head<3>() = -right.dot(eye), -upVec.dot(eye), -forward.dot(eye);
   }

   return viewMatrix;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> lookAtLh(const Matrix<T, 3, 1, Option>& eye,
                                  const Matrix<T, 3, 1, Option>& target,
                                  const Matrix<T, 3, 1, Option>& up) {
   Matrix<T, 3, 1, Option> forward = (target - eye).normalized();
   Matrix<T, 3, 1, Option> right = up.cross(forward).normalized();
   Matrix<T, 3, 1, Option> upVec = forward.cross(right);

   Matrix<T, 4, 4, Option> viewMatrix = Matrix<T, 4, 4, Option>::Identity();

   if constexpr (Option == StorageOptions::RowMajor) {
     viewMatrix.row(0).head<3>() = right.transpose();
     viewMatrix.row(1).head<3>() = upVec.transpose();
     viewMatrix.row(2).head<3>() = forward.transpose();
     viewMatrix.row(3).head<3>() = -right.dot(eye), -upVec.dot(eye), -forward.dot(eye);
   } else {
     viewMatrix.col(0).head<3>() = right;
     viewMatrix.col(1).head<3>() = upVec;
     viewMatrix.col(2).head<3>() = forward;
     viewMatrix.col(3).head<3>() = -right.dot(eye), -upVec.dot(eye), -forward.dot(eye);
   }

   return viewMatrix;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> lookToRh(const Matrix<T, 3, 1, Option>& eye,
                                  const Matrix<T, 3, 1, Option>& direction,
                                  const Matrix<T, 3, 1, Option>& up) {
   Matrix<T, 3, 1, Option> forward = direction.normalized();
   Matrix<T, 3, 1, Option> right = up.cross(forward).normalized();
   Matrix<T, 3, 1, Option> upVec = forward.cross(right);

   Matrix<T, 4, 4, Option> viewMatrix = Matrix<T, 4, 4, Option>::Identity();

   if constexpr (Option == StorageOptions::RowMajor) {
     viewMatrix.row(0).head<3>() = right.transpose();
     viewMatrix.row(1).head<3>() = upVec.transpose();
     viewMatrix.row(2).head<3>() = -forward.transpose();
     viewMatrix.row(3).head<3>() = -right.dot(eye), -upVec.dot(eye), -forward.dot(eye);
   } else {
     viewMatrix.col(0).head<3>() = right;
     viewMatrix.col(1).head<3>() = upVec;
     viewMatrix.col(2).head<3>() = -forward;
     viewMatrix.col(3).head<3>() = -right.dot(eye), -upVec.dot(eye), -forward.dot(eye);
   }

   return viewMatrix;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> lookToLh(const Matrix<T, 3, 1, Option>& eye,
                                  const Matrix<T, 3, 1, Option>& direction,
                                  const Matrix<T, 3, 1, Option>& up) {
   Matrix<T, 3, 1, Option> forward = direction.normalized();
   Matrix<T, 3, 1, Option> right = up.cross(forward).normalized();
   Matrix<T, 3, 1, Option> upVec = forward.cross(right);

   Matrix<T, 4, 4, Option> viewMatrix = Matrix<T, 4, 4, Option>::Identity();

   if constexpr (Option == StorageOptions::RowMajor) {
     viewMatrix.row(0).head<3>() = right.transpose();
     viewMatrix.row(1).head<3>() = upVec.transpose();
     viewMatrix.row(2).head<3>() = forward.transpose();
     viewMatrix.row(3).head<3>() = -right.dot(eye), -upVec.dot(eye), -forward.dot(eye);
   } else {
     viewMatrix.col(0).head<3>() = right;
     viewMatrix.col(1).head<3>() = upVec;
     viewMatrix.col(2).head<3>() = forward;
     viewMatrix.col(3).head<3>() = -right.dot(eye), -upVec.dot(eye), -forward.dot(eye);
   }

   return viewMatrix;
 }

// Projection matrix
 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveRhNo(T fovY, T aspect, T zNear, T zFar) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T const tanHalfFovY = tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = -(zFar + zNear) / (zFar - zNear);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(3, 2) = -(static_cast<T>(2) * zFar * zNear) / (zFar - zNear);
     result(2, 3) = -static_cast<T>(1);
   } else {
     result(2, 3) = -(static_cast<T>(2) * zFar * zNear) / (zFar - zNear);
     result(3, 2) = -static_cast<T>(1);
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveRhZo(T fovY, T aspect, T zNear, T zFar) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T const tanHalfFovY = tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = -zFar / (zFar - zNear);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(3, 2) = -(zFar * zNear) / (zFar - zNear);
     result(2, 3) = -static_cast<T>(1);
   } else {
     result(2, 3) = -(zFar * zNear) / (zFar - zNear);
     result(3, 2) = -static_cast<T>(1);
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveLhNo(T fovY, T aspect, T zNear, T zFar) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T const tanHalfFovY = tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = (zFar + zNear) / (zFar - zNear);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(3, 2) = -(static_cast<T>(2) * zFar * zNear) / (zFar - zNear);
     result(2, 3) = static_cast<T>(1);
   } else {
     result(2, 3) = -(static_cast<T>(2) * zFar * zNear) / (zFar - zNear);
     result(3, 2) = static_cast<T>(1);
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveLhZo(T fovY, T aspect, T zNear, T zFar) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T const tanHalfFovY = tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = zFar / (zFar - zNear);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(3, 2) = -(zFar * zNear) / (zFar - zNear);
     result(2, 3) = static_cast<T>(1);
   } else {
     result(2, 3) = -(zFar * zNear) / (zFar - zNear);
     result(3, 2) = static_cast<T>(1);
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveRhNo(T fovY, T width, T height, T zNear, T zFar) {
   T aspect = width / height;
   return perspectiveRhNo(fovY, aspect, zNear, zFar);
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveRhZo(T fovY, T width, T height, T zNear, T zFar) {
   T aspect = width / height;
   return perspectiveRhZo(fovY, aspect, zNear, zFar);
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveLhNo(T fovY, T width, T height, T zNear, T zFar) {
   T aspect = width / height;
   return perspectiveLhNo(fovY, aspect, zNear, zFar);
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveLhZo(T fovY, T width, T height, T zNear, T zFar) {
   T aspect = width / height;
   return perspectiveLhZo(fovY, aspect, zNear, zFar);
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveRhNoInf(T fovY, T aspect, T zNear) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T const tanHalfFovY = tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = -static_cast<T>(1);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(2, 3) = -static_cast<T>(1);
     result(3, 2) = -static_cast<T>(2) * zNear;
   } else {
     result(3, 2) = -static_cast<T>(1);
     result(2, 3) = -static_cast<T>(2) * zNear;
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveRhZoInf(T fovY, T aspect, T zNear) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T const tanHalfFovY = tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = -static_cast<T>(1);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(2, 3) = -static_cast<T>(1);
     result(3, 2) = -zNear;
   } else {
     result(3, 2) = -static_cast<T>(1);
     result(2, 3) = -zNear;
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveLhNoInf(T fovY, T aspect, T zNear) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T const tanHalfFovY = tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = static_cast<T>(1);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(2, 3) = static_cast<T>(1);
     result(3, 2) = -static_cast<T>(2) * zNear;
   } else {
     result(3, 2) = static_cast<T>(1);
     result(2, 3) = -static_cast<T>(2) * zNear;
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveLhZoInf(T fovY, T aspect, T zNear) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T const tanHalfFovY = tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = static_cast<T>(1);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(2, 3) = static_cast<T>(1);
     result(3, 2) = -zNear;
   } else {
     result(3, 2) = static_cast<T>(1);
     result(2, 3) = -zNear;
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> orthoRhNo(T left, T right, T bottom, T top, T zNear, T zFar) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   result(0, 0) = static_cast<T>(2) / (right - left);
   result(1, 1) = static_cast<T>(2) / (top - bottom);
   result(2, 2) = -static_cast<T>(2) / (zFar - zNear);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(3, 0) = -(right + left) / (right - left);
     result(3, 1) = -(top + bottom) / (top - bottom);
     result(3, 2) = -(zFar + zNear) / (zFar - zNear);
   } else {
     result(0, 3) = -(right + left) / (right - left);
     result(1, 3) = -(top + bottom) / (top - bottom);
     result(2, 3) = -(zFar + zNear) / (zFar - zNear);
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> orthoRhZo(T left, T right, T bottom, T top, T zNear, T zFar) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   result(0, 0) = static_cast<T>(2) / (right - left);
   result(1, 1) = static_cast<T>(2) / (top - bottom);
   result(2, 2) = -static_cast<T>(1) / (zFar - zNear);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(3, 0) = -(right + left) / (right - left);
     result(3, 1) = -(top + bottom) / (top - bottom);
     result(3, 2) = -zNear / (zFar - zNear);
   } else {
     result(0, 3) = -(right + left) / (right - left);
     result(1, 3) = -(top + bottom) / (top - bottom);
     result(2, 3) = -zNear / (zFar - zNear);
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> orthoLhNo(T left, T right, T bottom, T top, T zNear, T zFar) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   result(0, 0) = static_cast<T>(2) / (right - left);
   result(1, 1) = static_cast<T>(2) / (top - bottom);
   result(2, 2) = static_cast<T>(2) / (zFar - zNear);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(3, 0) = -(right + left) / (right - left);
     result(3, 1) = -(top + bottom) / (top - bottom);
     result(3, 2) = -(zFar + zNear) / (zFar - zNear);
   } else {
     result(0, 3) = -(right + left) / (right - left);
     result(1, 3) = -(top + bottom) / (top - bottom);
     result(2, 3) = -(zFar + zNear) / (zFar - zNear);
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> orthoLhZo(T left, T right, T bottom, T top, T zNear, T zFar) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   result(0, 0) = static_cast<T>(2) / (right - left);
   result(1, 1) = static_cast<T>(2) / (top - bottom);
   result(2, 2) = static_cast<T>(1) / (zFar - zNear);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(3, 0) = -(right + left) / (right - left);
     result(3, 1) = -(top + bottom) / (top - bottom);
     result(3, 2) = -zNear / (zFar - zNear);
   } else {
     result(0, 3) = -(right + left) / (right - left);
     result(1, 3) = -(top + bottom) / (top - bottom);
     result(2, 3) = -zNear / (zFar - zNear);
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> orthoRhNo(T width, T height, T zNear, T zFar) {
   T halfWidth = width / static_cast<T>(2);
   T halfHeight = height / static_cast<T>(2);

   T left = -halfWidth;
   T right = halfWidth;
   T bottom = -halfHeight;
   T top = halfHeight;

   return orthoRhNo<T, Option>(left, right, bottom, top, zNear, zFar);
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> orthoRhZo(T width, T height, T zNear, T zFar) {
   T halfWidth = width / static_cast<T>(2);
   T halfHeight = height / static_cast<T>(2);

   T left = -halfWidth;
   T right = halfWidth;
   T bottom = -halfHeight;
   T top = halfHeight;

   return orthoRhZo<T, Option>(left, right, bottom, top, zNear, zFar);
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> orthoLhNo(T width, T height, T zNear, T zFar) {
   T halfWidth = width / static_cast<T>(2);
   T halfHeight = height / static_cast<T>(2);

   T left = -halfWidth;
   T right = halfWidth;
   T bottom = -halfHeight;
   T top = halfHeight;

   return orthoLhNo<T, Option>(left, right, bottom, top, zNear, zFar);
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> orthoLhZo(T width, T height, T zNear, T zFar) {
   T halfWidth = width / static_cast<T>(2);
   T halfHeight = height / static_cast<T>(2);

   T left = -halfWidth;
   T right = halfWidth;
   T bottom = -halfHeight;
   T top = halfHeight;

   return orthoLhZo<T, Option>(left, right, bottom, top, zNear, zFar);
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> frustumRhNo(T left, T right, T bottom, T top, T nearVal, T farVal) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = (static_cast<T>(2) * nearVal) / (right - left);
   result(1, 1) = (static_cast<T>(2) * nearVal) / (top - bottom);
   result(2, 2) = -(farVal + nearVal) / (farVal - nearVal);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(2, 0) = (right + left) / (right - left);
     result(2, 1) = (top + bottom) / (top - bottom);
     result(3, 2) = -(static_cast<T>(2) * farVal * nearVal) / (farVal - nearVal);
     result(2, 3) = -static_cast<T>(1);
   } else {
     result(0, 2) = (right + left) / (right - left);
     result(1, 2) = (top + bottom) / (top - bottom);
     result(2, 3) = -(static_cast<T>(2) * farVal * nearVal) / (farVal - nearVal);
     result(3, 2) = -static_cast<T>(1);
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> frustumRhZo(T left, T right, T bottom, T top, T nearVal, T farVal) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = (static_cast<T>(2) * nearVal) / (right - left);
   result(1, 1) = (static_cast<T>(2) * nearVal) / (top - bottom);
   result(2, 2) = farVal / (nearVal - farVal);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(2, 0) = (right + left) / (right - left);
     result(2, 1) = (top + bottom) / (top - bottom);
     result(3, 2) = -(farVal * nearVal) / (farVal - nearVal);
     result(2, 3) = -static_cast<T>(1);
   } else {
     result(0, 2) = (right + left) / (right - left);
     result(1, 2) = (top + bottom) / (top - bottom);
     result(2, 3) = -(farVal * nearVal) / (farVal - nearVal);
     result(3, 2) = -static_cast<T>(1);
   }
   return result;
 }

 template <typename T, StorageOptions Option = StorageOptions::RowMajor>
 Matrix<T, 4, 4, Option> frustumLhNo(T left, T right, T bottom, T top, T nearVal, T farVal) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = (static_cast<T>(2) * nearVal) / (right - left);
   result(1, 1) = (static_cast<T>(2) * nearVal) / (top - bottom);
   result(2, 2) = (farVal + nearVal) / (farVal - nearVal);
   if constexpr (Option == StorageOptions::RowMajor) {
     result(2, 0) = -(right + left) / (right - left);
     result(2, 1) = -(top + bottom) / (top - bottom);
     result(3, 2) = -(static_cast<T>(2) * farVal * nearVal) / (farVal - nearVal);
     result(2, 3) = static_cast<T>(1);
   } else {
     result(0, 2) = -(right + left) / (right - left);
     result(1, 2) = -(top + bottom) / (top - bottom);
     result(2, 3) = -(static_cast<T>(2) * farVal * nearVal) / (farVal - nearVal);
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