#include <eigen_graphics/all.h>
#include <iostream>

int main()
{
    // Create two random matrices
    Eigen::MatrixXd mat1 = Eigen::MatrixXd::Random(3, 3);
    Eigen::MatrixXd mat2 = Eigen::MatrixXd::Random(3, 3);

    // Perform matrix multiplication
    Eigen::MatrixXd result = mat1 * mat2;

    // Print the input matrices and the result
    std::cout << "Matrix 1:\n" << mat1 << "\n\n";
    std::cout << "Matrix 2:\n" << mat2 << "\n\n";
    std::cout << "Multiplication result:\n" << result << "\n";

    return 0;
}