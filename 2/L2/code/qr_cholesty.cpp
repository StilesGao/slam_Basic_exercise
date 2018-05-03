#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 100

int main() {

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_100;
    matrix_100 = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix<double , MATRIX_SIZE, 1 >  v;
    v = Eigen::MatrixXd::Random( MATRIX_SIZE, 1 );
    Eigen::Matrix<double , MATRIX_SIZE, 1 > x, y1, y2;

    x = matrix_100.colPivHouseholderQr().solve(v);

    std::cout << "OR :x = " <<std::endl << x <<std::endl ;

    y1 = matrix_100.llt().solve(v);

    y2 = matrix_100.ldlt().solve(v);

    std::cout << "cholesty中llt分解：" << std::endl << y1 <<std::endl
              << "cholesty中ldlt分解：" << std::endl << y2  <<std::endl;

    return 0;
}