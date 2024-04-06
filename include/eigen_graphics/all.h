// eigen_graphics.h

 #ifndef EIGEN_GRAPHICS_MODULE_H
 #define EIGEN_GRAPHICS_MODULE_H

 #include <Eigen/Core>

 namespace Eigen {
 namespace Graphics {

// Constants
 template <typename T>
 constexpr T kDefaultTolerance = std::numeric_limits<T>::epsilon();

// Utility functions
 template <typename T>
 bool isNearlyZero(T value, T tolerance = kDefaultTolerance) {
   return std::abs(value) <= tolerance;
 }

// Matrix creation functions

// View matrix
 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> lookAtRh(const Matrix<T, 3, 1, Option>& eye,
                                  const Matrix<T, 3, 1, Option>& target,
                                  const Matrix<T, 3, 1, Option>& up) {
   auto forward = (target - eye).normalized();
   auto right = up.cross(forward).normalized();
   auto newUp = forward.cross(right);

   Matrix<T, 4, 4, Option> viewMatrix = Matrix<T, 4, 4, Option>::Identity();

   viewMatrix.template block<1, 3>(0, 0) = right.transpose();
   viewMatrix.template block<1, 3>(1, 0) = newUp.transpose();
   viewMatrix.template block<1, 3>(2, 0) = -forward.transpose();

   viewMatrix.template block<3, 1>(0, 3) = -(viewMatrix.template block<3, 3>(0, 0) * eye);

   return viewMatrix;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> lookAtLh(const Matrix<T, 3, 1, Option>& eye,
                                  const Matrix<T, 3, 1, Option>& target,
                                  const Matrix<T, 3, 1, Option>& up) {
   auto forward = (target - eye).normalized();
   auto right = up.cross(forward).normalized();
   auto newUp = forward.cross(right);

   Matrix<T, 4, 4, Option> viewMatrix = Matrix<T, 4, 4, Option>::Identity();

   viewMatrix.template block<1, 3>(0, 0) = right.transpose();
   viewMatrix.template block<1, 3>(1, 0) = newUp.transpose();
   viewMatrix.template block<1, 3>(2, 0) = forward.transpose();

   viewMatrix.template block<3, 1>(0, 3) = -(viewMatrix.template block<3, 3>(0, 0) * eye);

   return viewMatrix;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> lookToRh(const Matrix<T, 3, 1, Option>& eye,
                                  const Matrix<T, 3, 1, Option>& direction,
                                  const Matrix<T, 3, 1, Option>& up) {
   auto forward = direction.normalized();
   auto right = up.cross(forward).normalized();
   auto newUp = forward.cross(right);

   Matrix<T, 4, 4, Option> viewMatrix = Matrix<T, 4, 4, Option>::Identity();

   viewMatrix.template block<1, 3>(0, 0) = right.transpose();
   viewMatrix.template block<1, 3>(1, 0) = newUp.transpose();
   viewMatrix.template block<1, 3>(2, 0) = -forward.transpose();

   viewMatrix.template block<3, 1>(0, 3) = -(viewMatrix.template block<3, 3>(0, 0) * eye);

   return viewMatrix;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> lookToLh(const Matrix<T, 3, 1, Option>& eye,
                                  const Matrix<T, 3, 1, Option>& direction,
                                  const Matrix<T, 3, 1, Option>& up) {
   auto forward = direction.normalized();
   auto right = up.cross(forward).normalized();
   auto newUp = forward.cross(right);

   Matrix<T, 4, 4, Option> viewMatrix = Matrix<T, 4, 4, Option>::Identity();

   viewMatrix.template block<1, 3>(0, 0) = right.transpose();
   viewMatrix.template block<1, 3>(1, 0) = newUp.transpose();
   viewMatrix.template block<1, 3>(2, 0) = forward.transpose();

   viewMatrix.template block<3, 1>(0, 3) = -(viewMatrix.template block<3, 3>(0, 0) * eye);

   return viewMatrix;
 }

// Projection matrix
 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveRhNo(T fovY, T aspect, T zNear, T zFar) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = -(zFar + zNear) / (zFar - zNear);
   result(2, 3) = -static_cast<T>(1);
   result(3, 2) = -(static_cast<T>(2) * zFar * zNear) / (zFar - zNear);
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveRhZo(T fovY, T aspect, T zNear, T zFar) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = zFar / (zNear - zFar);
   result(2, 3) = -static_cast<T>(1);
   result(3, 2) = -(zFar * zNear) / (zFar - zNear);
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveLhNo(T fovY, T aspect, T zNear, T zFar) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = (zFar + zNear) / (zFar - zNear);
   result(2, 3) = static_cast<T>(1);
   result(3, 2) = -(static_cast<T>(2) * zFar * zNear) / (zFar - zNear);
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveLhZo(T fovY, T aspect, T zNear, T zFar) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = zFar / (zFar - zNear);
   result(2, 3) = static_cast<T>(1);
   result(3, 2) = -(zFar * zNear) / (zFar - zNear);
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveRhNo(T fovY, T width, T height, T zNear, T zFar) {
   T aspect = width / height;
   return perspectiveRhNo(fovY, aspect, zNear, zFar);
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveRhZo(T fovY, T width, T height, T zNear, T zFar) {
   T aspect = width / height;
   return perspectiveRhZo(fovY, aspect, zNear, zFar);
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveLhNo(T fovY, T width, T height, T zNear, T zFar) {
   T aspect = width / height;
   return perspectiveLhNo(fovY, aspect, zNear, zFar);
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveLhZo(T fovY, T width, T height, T zNear, T zFar) {
   T aspect = width / height;
   return perspectiveLhZo(fovY, aspect, zNear, zFar);
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveRhNoInf(T fovY, T aspect, T zNear) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = -static_cast<T>(1);
   result(2, 3) = -static_cast<T>(1);
   result(3, 2) = -static_cast<T>(2) * zNear;
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveRhZoInf(T fovY, T aspect, T zNear) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = -static_cast<T>(1);
   result(2, 3) = -static_cast<T>(1);
   result(3, 2) = -zNear;
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveLhNoInf(T fovY, T aspect, T zNear) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = static_cast<T>(1);
   result(2, 3) = static_cast<T>(1);
   result(3, 2) = -static_cast<T>(2) * zNear;
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> perspectiveLhZoInf(T fovY, T aspect, T zNear) {
   assert(std::abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

   T tanHalfFovY = std::tan(fovY / static_cast<T>(2));

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = static_cast<T>(1) / (aspect * tanHalfFovY);
   result(1, 1) = static_cast<T>(1) / (tanHalfFovY);
   result(2, 2) = static_cast<T>(1);
   result(2, 3) = static_cast<T>(1);
   result(3, 2) = -zNear;
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> orthoRhNo(T left, T right, T bottom, T top, T zNear, T zFar) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   result(0, 0) = static_cast<T>(2) / (right - left);
   result(1, 1) = static_cast<T>(2) / (top - bottom);
   result(2, 2) = -static_cast<T>(2) / (zFar - zNear);
   result(0, 3) = -(right + left) / (right - left);
   result(1, 3) = -(top + bottom) / (top - bottom);
   result(2, 3) = -(zFar + zNear) / (zFar - zNear);
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> orthoRhZo(T left, T right, T bottom, T top, T zNear, T zFar) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   result(0, 0) = static_cast<T>(2) / (right - left);
   result(1, 1) = static_cast<T>(2) / (top - bottom);
   result(2, 2) = -static_cast<T>(1) / (zFar - zNear);
   result(0, 3) = -(right + left) / (right - left);
   result(1, 3) = -(top + bottom) / (top - bottom);
   result(2, 3) = -zNear / (zFar - zNear);
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> orthoLhNo(T left, T right, T bottom, T top, T zNear, T zFar) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   result(0, 0) = static_cast<T>(2) / (right - left);
   result(1, 1) = static_cast<T>(2) / (top - bottom);
   result(2, 2) = static_cast<T>(2) / (zFar - zNear);
   result(0, 3) = -(right + left) / (right - left);
   result(1, 3) = -(top + bottom) / (top - bottom);
   result(2, 3) = -(zFar + zNear) / (zFar - zNear);
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> orthoLhZo(T left, T right, T bottom, T top, T zNear, T zFar) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   result(0, 0) = static_cast<T>(2) / (right - left);
   result(1, 1) = static_cast<T>(2) / (top - bottom);
   result(2, 2) = static_cast<T>(1) / (zFar - zNear);
   result(0, 3) = -(right + left) / (right - left);
   result(1, 3) = -(top + bottom) / (top - bottom);
   result(2, 3) = -zNear / (zFar - zNear);
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> orthoRhNo(T width, T height, T zNear, T zFar) {
   T halfWidth = width / static_cast<T>(2);
   T halfHeight = height / static_cast<T>(2);
   return orthoRhNo(-halfWidth, halfWidth, -halfHeight, halfHeight, zNear, zFar);
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> orthoRhZo(T width, T height, T zNear, T zFar) {
   T halfWidth = width / static_cast<T>(2);
   T halfHeight = height / static_cast<T>(2);
   return orthoRhZo(-halfWidth, halfWidth, -halfHeight, halfHeight, zNear, zFar);
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> orthoLhNo(T width, T height, T zNear, T zFar) {
   T halfWidth = width / static_cast<T>(2);
   T halfHeight = height / static_cast<T>(2);
   return orthoLhNo(-halfWidth, halfWidth, -halfHeight, halfHeight, zNear, zFar);
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> orthoLhZo(T width, T height, T zNear, T zFar) {
   T halfWidth = width / static_cast<T>(2);
   T halfHeight = height / static_cast<T>(2);
   return orthoLhZo(-halfWidth, halfWidth, -halfHeight, halfHeight, zNear, zFar);
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> frustumRhNo(T left, T right, T bottom, T top, T nearVal, T farVal) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = (static_cast<T>(2) * nearVal) / (right - left);
   result(1, 1) = (static_cast<T>(2) * nearVal) / (top - bottom);
   result(0, 2) = (right + left) / (right - left);
   result(1, 2) = (top + bottom) / (top - bottom);
   result(2, 2) = -(farVal + nearVal) / (farVal - nearVal);
   result(2, 3) = -static_cast<T>(1);
   result(3, 2) = -(static_cast<T>(2) * farVal * nearVal) / (farVal - nearVal);
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> frustumRhZo(T left, T right, T bottom, T top, T nearVal, T farVal) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = (static_cast<T>(2) * nearVal) / (right - left);
   result(1, 1) = (static_cast<T>(2) * nearVal) / (top - bottom);
   result(0, 2) = (right + left) / (right - left);
   result(1, 2) = (top + bottom) / (top - bottom);
   result(2, 2) = farVal / (nearVal - farVal);
   result(2, 3) = -static_cast<T>(1);
   result(3, 2) = -(farVal * nearVal) / (farVal - nearVal);
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> frustumLhNo(T left, T right, T bottom, T top, T nearVal, T farVal) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = (static_cast<T>(2) * nearVal) / (right - left);
   result(1, 1) = (static_cast<T>(2) * nearVal) / (top - bottom);
   result(0, 2) = -(right + left) / (right - left);
   result(1, 2) = -(top + bottom) / (top - bottom);
   result(2, 2) = (farVal + nearVal) / (farVal - nearVal);
   result(2, 3) = static_cast<T>(1);
   result(3, 2) = -(static_cast<T>(2) * farVal * nearVal) / (farVal - nearVal);
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> frustumLhZo(T left, T right, T bottom, T top, T nearVal, T farVal) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Zero();
   result(0, 0) = (static_cast<T>(2) * nearVal) / (right - left);
   result(1, 1) = (static_cast<T>(2) * nearVal) / (top - bottom);
   result(0, 2) = -(right + left) / (right - left);
   result(1, 2) = -(top + bottom) / (top - bottom);
   result(2, 2) = farVal / (farVal - nearVal);
   result(2, 3) = static_cast<T>(1);
   result(3, 2) = -(farVal * nearVal) / (farVal - nearVal);
   return result;
 }

// Translation
 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> translate(T dx, T dy, T dz) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   result(0, 3) = dx;
   result(1, 3) = dy;
   result(2, 3) = dz;
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> translate(const Matrix<T, 3, 1, Option>& translation) {
   return translate(translation.x(), translation.y(), translation.z());
 }

 template <typename T, int Option = Eigen::RowMajor>
 void addTranslate(Matrix<T, 4, 4, Option>& matrix, T dx, T dy, T dz) {
   matrix(0, 3) += dx;
   matrix(1, 3) += dy;
   matrix(2, 3) += dz;
 }

 template <typename T, int Option = Eigen::RowMajor>
 void addTranslate(Matrix<T, 4, 4, Option>& matrix, const Matrix<T, 3, 1, Option>& translation) {
   addTranslate(matrix, translation.x(), translation.y(), translation.z());
 }

 template <typename T, int Option = Eigen::RowMajor>
 void setTranslate(Matrix<T, 4, 4, Option>& matrix, T dx, T dy, T dz) {
   matrix.template block<3, 1>(0, 3) = Matrix<T, 3, 1, Option>(dx, dy, dz);
 }

 template <typename T, int Option = Eigen::RowMajor>
 void setTranslate(Matrix<T, 4, 4, Option>& matrix, const Matrix<T, 3, 1, Option>& translation) {
   setTranslate(matrix, translation.x(), translation.y(), translation.z());
 }

// Scale
 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> scale(T sx, T sy, T sz) {
   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   result(0, 0) = sx;
   result(1, 1) = sy;
   result(2, 2) = sz;
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> scale(const Matrix<T, 3, 1, Option>& scale) {
   return scale(scale.x(), scale.y(), scale.z());
 }

// Rotation
 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> rotateRhX(T angle) {
   T cosAngle = std::cos(angle);
   T sinAngle = std::sin(angle);

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   result(1, 1) = cosAngle;
   result(1, 2) = sinAngle;
   result(2, 1) = -sinAngle;
   result(2, 2) = cosAngle;
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> rotateRhY(T angle) {
   T cosAngle = std::cos(angle);
   T sinAngle = std::sin(angle);

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   result(0, 0) = cosAngle;
   result(0, 2) = -sinAngle;
   result(2, 0) = sinAngle;
   result(2, 2) = cosAngle;
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> rotateRhZ(T angle) {
   T cosAngle = std::cos(angle);
   T sinAngle = std::sin(angle);

   Matrix<T, 4, 4, Option> result = Matrix<T, 4, 4, Option>::Identity();
   result(0, 0) = cosAngle;
   result(0, 1) = sinAngle;
   result(1, 0) = -sinAngle;
   result(1, 1) = cosAngle;
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> rotateRh(T angleX, T angleY, T angleZ) {
   T cx = std::cos(angleX);
   T sx = std::sin(angleX);
   T cy = std::cos(angleY);
   T sy = std::sin(angleY);
   T cz = std::cos(angleZ);
   T sz = std::sin(angleZ);

   Matrix<T, 4, 4, Option> result;
   result(0, 0) = cy * cz;
   result(0, 1) = cy * sz;
   result(0, 2) = -sy;
   result(1, 0) = sx * sy * cz - cx * sz;
   result(1, 1) = sx * sy * sz + cx * cz;
   result(1, 2) = sx * cy;
   result(2, 0) = cx * sy * cz + sx * sz;
   result(2, 1) = cx * sy * sz - sx * cz;
   result(2, 2) = cx * cy;
   result.template block<1, 4>(3, 0) = Matrix<T, 1, 4, Option>::Identity();
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> rotateRh(const Matrix<T, 3, 1, Option>& angles) {
   return rotateRh(angles.x(), angles.y(), angles.z());
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> rotateRh(const Matrix<T, 3, 1, Option>& axis, T angle) {
   T cosAngle = std::cos(angle);
   T sinAngle = std::sin(angle);
   T oneMinusCos = static_cast<T>(1) - cosAngle;

   Matrix<T, 3, 1, Option> normalizedAxis = axis.normalized();

   T x = normalizedAxis.x();
   T y = normalizedAxis.y();
   T z = normalizedAxis.z();

   Matrix<T, 4, 4, Option> result;
   result(0, 0) = cosAngle + x * x * oneMinusCos;
   result(0, 1) = x * y * oneMinusCos - z * sinAngle;
   result(0, 2) = x * z * oneMinusCos + y * sinAngle;
   result(1, 0) = y * x * oneMinusCos + z * sinAngle;
   result(1, 1) = cosAngle + y * y * oneMinusCos;
   result(1, 2) = y * z * oneMinusCos - x * sinAngle;
   result(2, 0) = z * x * oneMinusCos - y * sinAngle;
   result(2, 1) = z * y * oneMinusCos + x * sinAngle;
   result(2, 2) = cosAngle + z * z * oneMinusCos;
   result.template block<1, 4>(3, 0) = Matrix<T, 1, 4, Option>::Identity();
   return result;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> rotateLhX(T angle) {
   return rotateRhX(-angle);
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> rotateLhY(T angle) {
   return rotateRhY(-angle);
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> rotateLhZ(T angle) {
   return rotateRhZ(-angle);
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> rotateLh(T angleX, T angleY, T angleZ) {
   return rotateRh(-angleX, -angleY, -angleZ);
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> rotateLh(const Matrix<T, 3, 1, Option>& angles) {
   return rotateLh(angles.x(), angles.y(), angles.z());
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 4, Option> rotateLh(const Matrix<T, 3, 1, Option>& axis, T angle) {
   return rotateRh(axis, -angle);
 }

// Transform
 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 4, 1, Option> perspectiveDivide(const Matrix<T, 4, 1, Option>& vector, T tolerance = kDefaultTolerance) {
   if (!isNearlyZero(vector.w(), tolerance)) {
     return vector / vector.w();
   }
   return vector;
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 3, 1, Option> transformPoint(const Matrix<T, 3, 1, Option>& point, const Matrix<T, 4, 4, Option>& matrix) {
   Matrix<T, 4, 1, Option> homogeneousPoint(point.x(), point.y(), point.z(), static_cast<T>(1));
   Matrix<T, 4, 1, Option> transformedHomogeneousPoint = matrix * homogeneousPoint;
   Matrix<T, 4, 1, Option> result = perspectiveDivide(transformedHomogeneousPoint);
   return result.template block<3, 1>(0, 0);
 }

 template <typename T, int Option = Eigen::RowMajor>
 Matrix<T, 3, 1, Option> transformVector(const Matrix<T, 3, 1, Option>& vector, const Matrix<T, 4, 4, Option>& matrix)
 {
   Matrix<T, 4, 1, Option> homogeneousVector(vector.x(), vector.y(), vector.z(), static_cast<T>(0));
   Matrix<T, 4, 1, Option> transformedHomogeneousVector = matrix * homogeneousVector;
   return transformedHomogeneousVector.template block<3, 1>(0, 0);
 }

 } // namespace Graphics
 } // namespace Eigen

 #endif // EIGEN_GRAPHICS_MODULE_H