#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream fp3d, fp2d;
    fp3d.open(p3d_file.c_str());
    fp2d.open(p2d_file.c_str());

    if(!fp2d.is_open() || !fp3d.is_open())
        cout << "open file is failure" << endl;

    while (!fp3d.eof())
    {
        Vector3d p3;
        string s;
        getline(fp3d, s);
        stringstream ss;
        ss << s;
        ss>> p3[0] >> p3[1] >> p3[2];
        p3d.push_back(p3);
    }
    while (!fp2d.eof())
    {
        Vector2d p2;
        string s;
        getline(fp2d, s);
        stringstream ss;
        ss << s;
        ss>> p2[0] >> p2[1];
        p2d.push_back(p2);
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
        // compute cost for p3d[I] and p2d[I]
        // START YOUR CODE HERE

            Vector4d p = T_esti.matrix() * Vector4d(p3d[i][0], p3d[i][1], p3d[i][2], 1);
            Vector2d e ( p2d[i][0] - fx * p[0] /p[2]-cx, p2d[i][1] -fy * p[1] / p[2]-cy );
            cost = e[0] * e[0] + e[1] * e[1];

	    // END YOUR CODE HERE

	    // compute jacobian

        // START YOUR CODE HERE
            Matrix<double, 2, 6> J;
            J(0,0) = -fx / p[2];
            J(0,1) = 0;
            J(0,2) = fx * p[0] / (p[2] *p[2]);
            J(0,3) = fx * p[0] *p[1] / (p[2] * p[2]);
            J(0,4) = -fx - fx * p[0] *p[0] / (p[2] *p[2]);
            J(0,5) = fx*p[1] / p[2];

            J(1,0) = 0;
            J(1,1) = -fy / p[2];
            J(1,2) = fy*p[1]/(p[2] *p[2]);
            J(1,3) = fy+fy*p[1]*p[1] / (p[2] *p[2]);
            J(1,4) = -fy*p[0]*p[1]/(p[2] *p[2]);
            J(1,5) = -fy*p[0]/p[2];


            // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	// solve dx 
        Vector6d dx;

        // START YOUR CODE HERE

        dx = H.ldlt().solve(b);

        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti = Sophus::SE3::exp(dx) * T_esti;
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
