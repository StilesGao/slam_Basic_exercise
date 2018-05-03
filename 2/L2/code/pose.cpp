#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char** argv){
    Eigen::Quaterniond q1( 0.55,0.3,0.2,0.2 );
    Eigen::Quaterniond q2( -0.1,0.3,-0.7,0.2 );

    std::cout << "q1 = " << q1.coeffs()<< std::endl;
    std::cout << "q2 = " << q2.coeffs()<< std::endl;

    Eigen::Vector3d t1( 0.7,1.1,0.2 );
    Eigen::Vector3d t2( -0.1,0.4,0.8 );

    std::cout << "t1 = " << t1 << std::endl;
    std::cout << "t2 = " << t2 << std::endl;

    Eigen::Vector3d p1( 0.5,-0.1,0.2 );
    Eigen::Vector3d p2;

    q1.normalize();
    q2.normalize();

    Eigen::Isometry3d Tcw1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d Tcw2 = Eigen::Isometry3d::Identity();

    Tcw1.rotate(q1);
    Tcw1.pretranslate(t1);

    std::cout << "transform matrix Tcw1 = " << std::endl
                                       << Tcw1.matrix() << std::endl;

    Tcw2.rotate(q2);
    Tcw2.pretranslate(t2);

    std::cout << "transform matrix Tcw2 = " << std::endl
              << Tcw1.matrix() << std::endl;

    p2 = Tcw2 * Tcw1.inverse() * p1;

    std::cout << "p2 = " << std::endl << p2 << std::endl;

//    Eigen::Vector3d v1(1,2,1);
//    v1.normalize();
//    std::cout << "normalize v1" <<std::endl << v1<< std::endl;
//
//    Eigen::Vector3d v2(1,2,1);
//    v2.normalized();
//    std::cout << "normalized v2:" << std::endl << v2 << std::endl;

    return 0;
}

