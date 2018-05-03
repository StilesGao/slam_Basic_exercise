
#include <iostream>

using namespace std;

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>
#include <boost/format.hpp>

typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> VecSE3;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3d;

// global variables
string pose_file = "./poses.txt";
string points_file = "./points.txt";

// intrinsics
float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}

// g2o vertex that use sophus::SE3 as pose
class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSophus() {}

    ~VertexSophus() {}

    bool read(std::istream &is) {}

    bool write(std::ostream &os) const {}

    virtual void setToOriginImpl() {
        _estimate = Sophus::SE3();
    }

    virtual void oplusImpl(const double *update_) {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
        setEstimate(Sophus::SE3::exp(update) * estimate());
    }
};

// TODO edge of projection error, implement it
// 16x1 error, which is the errors in patch
typedef Eigen::Matrix<double,16,1> Vector16d;
class EdgeDirectProjection : public g2o::BaseBinaryEdge<16, Vector16d, g2o::VertexSBAPointXYZ, VertexSophus> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeDirectProjection(float *color, cv::Mat &target) {
        this->origColor = color;
        this->targetImg = target;
    }

    ~EdgeDirectProjection() {}

    virtual void computeError() override {
        // TODO START YOUR CODE HERE
        // compute projection error ...
        const VertexSophus* cam = dynamic_cast<const VertexSophus*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* point = dynamic_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        int count = 0;

        Eigen::Vector4d P = cam->estimate().matrix() * Eigen::Vector4d(point->estimate()[0],point->estimate()[1], point->estimate()[2],1);
        Eigen::Vector2d pixel;
        pixel[0] = fx*P[0]/P[2] +cx;
        pixel[1] = fy*P[1]/P[2] +cy;

        if(pixel[0] > 0 || pixel[0]-2 > 0 || pixel[0] < targetImg.cols-1
           || pixel[1] > 0 || pixel[1]-2 > 0 || pixel[1] < targetImg.rows-1)
        {
            for(int u = -2; u < 2; u++)
                for(int v = -2; v < 2; v++)
                {
                    _error[count] = _measurement[count] - GetPixelValue(targetImg, pixel[0]+u,pixel[1]+v);
                    cout << "_error["<<count <<"]"<<_error[count] << endl;
                    cout << "_measurement["<<count <<"]"<< _measurement[count] << endl;
                    count++;
                }
        }

        // END YOUR CODE HERE
    }

    // Let g2o compute jacobian for you

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}

private:
    cv::Mat targetImg;  // the target image
    float *origColor = nullptr;   // 16 floats, the color of this point
};

// plot the poses and points for you, need pangolin
void Draw(const VecSE3 &poses, const VecVec3d &points);

int main(int argc, char **argv) {

    // read poses and points
    VecSE3 poses;
    VecVec3d points;
    ifstream fin(pose_file);

    //cout << "pose:"<<endl;
    while (!fin.eof()) {
        double timestamp = 0;
        fin >> timestamp;
        if (timestamp == 0) break;
        double data[7];
        for (auto &d: data) fin >> d;
        Sophus::SE3 pose(Sophus::SE3(
                Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                Eigen::Vector3d(data[0], data[1], data[2])
        ));
        //cout << pose.matrix()<<endl;

        poses.push_back(Sophus::SE3(
                Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                Eigen::Vector3d(data[0], data[1], data[2])
        ));
        if (!fin.good()) break;
    }
    fin.close();


    vector<float *> color;
    fin.open(points_file);
    //cout << "points:"<<endl;
    while (!fin.eof()) {
        double xyz[3] = {0};
        for (int i = 0; i < 3; i++) fin >> xyz[i];
        if (xyz[0] == 0) break;
        points.push_back(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
        //cout << Eigen::Vector3d(xyz[0], xyz[1], xyz[2])<<", ";

        float *c = new float[16];
        for (int i = 0; i < 16; i++) fin >> c[i];
        color.push_back(c);

        if (fin.good() == false)
            break;
    }
    fin.close();

    cout << "poses: " << poses.size() << ", points: " << points.size() << endl;

    // read images
    vector<cv::Mat> images;
    boost::format fmt("./%d.png");
    for (int i = 0; i < 7; i++) {
        images.push_back(cv::imread((fmt % i).str(), 0));
    }
    cout <<"images:" << images.size() << endl;
    // build optimization problem
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> DirectBlock;  // 求解的向量是6＊1的
    DirectBlock::LinearSolverType *linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
    DirectBlock *solver_ptr = new DirectBlock(std::unique_ptr<DirectBlock::LinearSolverType>(linearSolver));
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<DirectBlock>(solver_ptr)); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);//输出调试信息

    // TODO add vertices, edges into the graph optimizer
    // START YOUR CODE HERE

    //添加ｃａｍｅｒａｐｏｓｅ
    for(int i = 0; i < poses.size(); i++) {
        VertexSophus *pose = new VertexSophus();
        pose->setId(i);
        if( i == 0 ){
            pose->setFixed(true);
        }
        pose->setToOriginImpl();
        optimizer.addVertex(pose);
    }
    //添加观测点
    for(int j = 0; j < points.size(); j++)
    {
        g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();
        point->setId(j+poses.size());
        point->setFixed(true);
        point->setEstimate(Eigen::Vector3d(points[j][0], points[j][1],points[j][2]));
        point->setMarginalized(true);
        optimizer.addVertex(point);
    }
    //添加边（误差）
    //cout << "obr:"<<endl;
    for(int i = 0; i < 7; i++) {
        for (int k = 0; k < points.size(); k++) {
            EdgeDirectProjection *edge = new EdgeDirectProjection(color[k], images[i]);
            int camera_id = k;
            int point_id = k + poses.size();
            Eigen::Matrix<double,16,1> obr;
            obr << color[k][0],color[k][1],color[k][2], color[k][3],color[k][4],color[k][5],
                    color[k][6],color[k][7],color[k][8],color[k][9],color[k][10],color[k][11],
                    color[k][12],color[k][13],color[k][14],color[k][15];

            //cout << obr.transpose();

            edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(point_id)));
            edge->setVertex(1, dynamic_cast<VertexSophus*>(optimizer.vertex(camera_id)));
            cout <<"optimizer.vertex:"<<optimizer.vertex(point_id);
            edge->setMeasurement(obr);
            //edge->setParameterId(0,0);
            edge->setInformation(Eigen::Matrix<double,16,16>::Identity());
            //核函数
            edge->setRobustKernel(new g2o::RobustKernelHuber());
            optimizer.addEdge(edge);
        }
    }
    // END YOUR CODE HERE

    // perform optimization
    optimizer.initializeOptimization();
    optimizer.optimize(200);

    // TODO fetch data from the optimizer
    // START YOUR CODE HERE

    poses.clear();points.clear();
    Sophus::SE3 NewCameraVec;
    cout << "new camera pose:"<<endl;
    for(int i = 0; i < poses.size(); i++)
    {
        VertexSophus* pCamera = dynamic_cast<VertexSophus*>(optimizer.vertex(i));
        NewCameraVec = pCamera->estimate();
        poses.push_back(NewCameraVec);
        cout << NewCameraVec << ", ";
    }
    Eigen::Vector3d NewPoint;
    cout << "new point :"<<endl;
    for(int j = 0; j < points.size(); j++)
    {
        g2o::VertexSBAPointXYZ* pPoint = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(j+poses.size()));
        NewPoint = pPoint->estimate();
        points.push_back(NewPoint);
        cout << NewPoint << ", ";
    }
    // END YOUR CODE HERE

    // plot the optimized points and poses
    Draw(poses, points);

    // delete color data
    for (auto &c: color) delete[] c;
    return 0;
}

void Draw(const VecSE3 &poses, const VecVec3d &points) {
    if (poses.empty() || points.empty()) {
        cerr << "parameter is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        for (auto &Tcw: poses) {
            glPushMatrix();
            Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
            glMultMatrixf((GLfloat *) m.data());
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glEnd();
            glPopMatrix();
        }

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); i++) {
            glColor3f(0.0, points[i][2]/4, 1.0-points[i][2]/4);
            glVertex3d(points[i][0], points[i][1], points[i][2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

