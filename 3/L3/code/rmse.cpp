#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <se3.h>
#include <pangolin/pangolin.h>

using namespace std;

void draw_trace(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> readfile(string&);

string groundtruth = "/home/stiles/桌面/slam-shenlan/3/L3/code/groundtruth.txt";

string estimated = "/home/stiles/桌面/slam-shenlan/3/L3/code/estimated.txt";

int main(int argv, char** argc)
{
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g, poses_e;
    typedef Eigen::Matrix<double , 6 ,1> Vector6d;

    Vector6d e;
    Sophus::SE3 SE3_g, SE3_e;

    double nRMSE_2= 0;
    double RMSE = 0;
    int n = 0;

    poses_g = readfile(groundtruth);

    poses_e = readfile(estimated);

    //draw_trace(poses_e, poses_g);

    if( poses_e.size() >= poses_g.size() )
    {
        n = poses_g.size();
        for( int i = 0; i < poses_g.size(); i++ )
        {
            SE3_g = poses_g[i];//李代数se(3) 是一个六维向量
                                        //在Sophus中，se(3)的平移在前，旋转在后.
            SE3_e = poses_e[i];
            e = (SE3_g.inverse()*SE3_e).log();
            nRMSE_2 += e.transpose()*e;
        }
        RMSE = sqrt(nRMSE_2/n);

        cout << "RMSE = " << RMSE << endl;
    }
    else
    {
        n = poses_e.size();
        for (int i = 0; i < poses_e.size(); i++)
        {
            SE3_g = poses_g[i];//李代数se(3) 是一个六维向量
            //在Sophus中，se(3)的平移在前，旋转在后.
            SE3_e = poses_e[i];
            e = (SE3_g.inverse() * SE3_e).log();
            nRMSE_2 += e.transpose() * e;
        }
        RMSE = sqrt(nRMSE_2 / n);

        cout << "RMSE = " << RMSE << endl;
    }
    return  0;
}

void draw_trace(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose1, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose2)
{
    if(pose1.empty() || pose2.empty())
    {
        cerr << "trace is empty" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("trace", 1024,768);
    glEnable( GL_DEPTH_TEST );//启用深度测试,根据坐标的远近自动隐藏被遮住的图形（材料）
    glEnable( GL_BLEND );//启用颜色混合。例如实现半透明效果
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );//表示源颜色乘以自身的alpha 值，
    // 目标颜色乘以1.0减去源颜色的alpha值，
    // 这样一来，源颜色的alpha值越大，则产生的新颜色中源颜色所占比例就越大，
    // 而目标颜色所占比例则减 小。
    // 这种情况下，我们可以简单的将源颜色的alpha值理解为“不透明度”。
    // 这也是混合时最常用的方式。

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0,-10,0.1, 0,0,0, pangolin::AxisNegY)
    );//对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
    pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0,1.0,pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

    while(pangolin::ShouldQuit() == false)
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f, 1.0f);

        glLineWidth(2);
        for( size_t i = 0; i < pose1.size() - 1; i++ )
        {
            glColor3f(0.0f, 0.0f, (float)i / pose2.size());
            glBegin(GL_LINES);
            auto p1 = pose1[i], p2 = pose1[i+1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);

            glColor3f(0.0f, (float)i / pose2.size(), 0.0f);
            glBegin(GL_LINES);
            p1 = pose2[i], p2 = pose2[i+1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(500);
    }

}

vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> readfile( string &filename)
{
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose;
    ifstream infile;
    infile.open(filename.c_str());
    if(!infile.is_open())
    {
        cout << "open file is failure" <<endl;
    }

    while(!infile.eof())
    {
        double t, tx, ty, tz, qx,qy, qz, qw;
        string s;
        stringstream ss;
        getline(infile, s);
        ss << s;
        ss >> t >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Vector3d t1(tx, ty, tz);
        Eigen::Quaterniond q(qw, qx, qy, qz);
        Sophus::SE3 SE3_qt(q, t1);
        pose.push_back(SE3_qt);
    }

    return pose;

}