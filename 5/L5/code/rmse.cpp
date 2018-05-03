#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <se3.h>
#include <pangolin/pangolin.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;

void draw_trace(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

vector<vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>> readfile(string&);

vector<vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>> readfile_change( string & , vector<Eigen::Vector3d>&);

void ICP(vector<cv::Point3f>&,vector<cv::Point3f>&,Eigen::Matrix3d&,Eigen::Vector3d&);

string compare = "./compare.txt";

vector<Eigen::Vector3d> p_e, p_g;

Eigen::Matrix3d R;
Eigen::Vector3d t;

int main(int argv, char** argc)
{
    vector<vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>> poses;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g, poses_e;
    typedef Eigen::Matrix<double , 6 ,1> Vector6d;

    Vector6d e;
    Sophus::SE3 SE3_g, SE3_e;

    double nRMSE_2= 0;
    double RMSE = 0;
    int n = 0;

    cv::Point3f p1, p2;
    vector<cv::Point3f> pts1, pts2;

    poses = readfile(compare);
    poses_g = poses[0];
    poses_e = poses[1];

    //draw_trace(poses_e, poses_g);

    for(int i = 0; i < p_e.size(); i++)
    {
        p1.x = p_e[i][0];
        p1.y = p_e[i][1];
        p1.z = p_e[i][2];
        pts1.push_back(p1);

        p2.x = p_g[i][0];
        p2.y = p_g[i][1];
        p2.z = p_g[i][2];
        pts2.push_back(p2);
    }

    ICP(pts1, pts2, R, t);

    Sophus::SE3 SE3_translate(R,t);

    for(int ii = 0; ii < p_g.size(); ii++)
    {
        p_g[ii] = SE3_translate*p_g[ii];
    }
    poses = readfile_change(compare,p_g);
    poses_g = poses[0];
    poses_e = poses[1];

    draw_trace(poses_e, poses_g);

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
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        glLineWidth(2);
        for( size_t i = 0; i < pose1.size() - 1; i++ )
        {
            glColor3f(0.0f, 0.0f, (float)i / pose2.size());
            glBegin(GL_LINES);
            auto p1 = pose1[i], p2 = pose1[i+1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);

            glColor3f((float)i / pose2.size(), 0.0f, 0.0f);
            glBegin(GL_LINES);
            p1 = pose2[i], p2 = pose2[i+1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);
    }

}

vector<vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>> readfile( string &filename)
{
    vector<vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>> pose;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_e, pose_g;
    ifstream infile;
    infile.open(filename.c_str());
    if(!infile.is_open())
    {
        cout << "open file is failure" <<endl;
    }

    while(!infile.eof())
    {
        double d[16];
        string s;
        stringstream ss;
        getline(infile, s);
        ss << s;
        ss >>d[0] >> d[1] >> d[2] >> d[3]>>d[4]>>d[5]>>d[6]>>d[7]>>d[8]>>d[9]
                >>d[10]>>d[11]>>d[12]>>d[13]>>d[14]>>d[15];
        Eigen::Vector3d te(d[1], d[2], d[3]);
        Eigen::Quaterniond qe(d[4], d[5], d[6], d[7]);
        Sophus::SE3 SE3_qt_e(qe, te);
        pose_e.push_back(SE3_qt_e);
        p_e.push_back(te);

        Eigen::Vector3d tg(d[9], d[10],d[11]);
        Eigen::Quaterniond qg(d[12], d[13], d[14], d[15]);
        Sophus::SE3 SE3_qt_g(qg, tg);
        pose_g.push_back(SE3_qt_g);
        p_g.push_back(tg);

    }
    pose.push_back(pose_e);
    pose.push_back(pose_g);
    return pose;

}

void ICP(vector<cv::Point3f>&pts1,vector<cv::Point3f>&pts2,Eigen::Matrix3d&R,Eigen::Vector3d &t)
{
    cv::Point3f p1,p2;
    int N = pts1.size();

    for(int i = 0; i< N ; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }

    p1 = cv::Point3f( cv::Vec3f(p1) /  N);
    p2 = cv::Point3f( cv::Vec3f(p2) / N);
    vector<cv::Point3f>     q1 ( N ), q2 ( N ); // remove the center
    for ( int i=0; i<N; i++ )
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
    cout<<"W="<<W<<endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout<<"U="<<U<<endl;
    cout<<"V="<<V<<endl;

    R = U* ( V.transpose() );
    t = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R * Eigen::Vector3d ( p2.x, p2.y, p2.z );

    return;
}

vector<vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>> readfile_change( string &filename, vector<Eigen::Vector3d> &p_g)
{
    vector<vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>> pose;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_e, pose_g;
    ifstream infile;
    infile.open(filename.c_str());
    if(!infile.is_open())
    {
        cout << "open file is failure" <<endl;
    }
    int ii = 0;
    while(!infile.eof())
    {
        double d[16];
        string s;
        stringstream ss;
        getline(infile, s);
        ss << s;
        ss >>d[0] >> d[1] >> d[2] >> d[3]>>d[4]>>d[5]>>d[6]>>d[7]>>d[8]>>d[9]
           >>d[10]>>d[11]>>d[12]>>d[13]>>d[14]>>d[15];
        Eigen::Vector3d te(d[1], d[2], d[3]);
        Eigen::Quaterniond qe(d[4], d[5], d[6], d[7]);
        Sophus::SE3 SE3_qt_e(qe, te);
        pose_e.push_back(SE3_qt_e);
        p_e.push_back(te);

        Eigen::Vector3d tg = p_g[ii];
        Eigen::Quaterniond qg(d[12], d[13], d[14], d[15]);
        Sophus::SE3 SE3_qt_g(qg, tg);
        pose_g.push_back(SE3_qt_g);
        ++ii;
    }
    pose.push_back(pose_e);
    pose.push_back(pose_g);
    return pose;

}