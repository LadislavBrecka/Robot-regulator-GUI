#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include<windows.h>
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
#include "ckobuki.h"
#include "rplidar.h"
/*#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"*/

struct Point {
    float x;
    float y;

private:
    float m_invalidX = 9999.0;
    float m_invalidY = 9999.0;

public:
    Point(float setX, float setY)
    {
        x = setX;
        y = setY;
    }

    Point()
    {
        x = m_invalidX;
        y = m_invalidY;
    }

    bool isValid()
    {
        if (x == m_invalidX && y == m_invalidY)
            return false;
        else
            return true;
    }
};

class FifoQueue {

private:
    std::vector<Point> m_targets;

public:

    void In(Point p)
    {
        m_targets.push_back(p);
    }

    Point Out()
    {
        if (!m_targets.empty())
        {
            return m_targets.front();;
        }
        else
            return Point();
    }

    void Pop()
    {
        if (!m_targets.empty())
        {
            m_targets.erase(m_targets.begin());
        }
    }

    std::vector<Point> GetPoints()
    {
        return m_targets;
    }
};


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool useCamera;
//    cv::VideoCapture cap;

    int actIndex;
//    cv::Mat frame[3];

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void robotprocess();
    void laserprocess();
    void processThisLidar(LaserMeasurement &laserData);

    void processThisRobot();
    HANDLE robotthreadHandle; // handle na vlakno
    DWORD robotthreadID;  // id vlakna
    static DWORD WINAPI robotUDPVlakno(void *param)
    {
        ((MainWindow*)param)->robotprocess();
        return 0;
    }
    HANDLE laserthreadHandle; // handle na vlakno
    DWORD laserthreadID;  // id vlakna
    static DWORD WINAPI laserUDPVlakno(void *param)
    {
        ((MainWindow*)param)->laserprocess();

        return 0;
    }
    //veci na broadcast laser
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;

    int las_s,  las_recv_len;
    unsigned int las_slen;
    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;

    int rob_s,  rob_recv_len;
    unsigned int rob_slen;

private slots:

    // utility functions
    std::pair<double, double> GetTargetOffset(Point actual, Point target);
    double RadToDegree(double radians);
    double DegreeToRad(double degrees);
    void   PrintTargetQueue();

    // robot control functions
    void RobotSetTranslationSpeed(float speed);
    void RobotSetRotationSpeed(float speed);
    void RegulatorRotation(double dTheta);
    void RegulatorTranslation(double distance, double dTheta);
    void EvaluateRegulation(double distance, double dTheta);

    // qt realted functions
    void getNewFrame();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();
    void on_pushButton_5_clicked();
    void on_pushButton_6_clicked();
    void on_pushButton_7_clicked();
    void on_pushButton_9_clicked();
    void on_pushButton_10_clicked();
    void on_pushButton_11_clicked();

private:

     JOYINFO joystickInfo;
     Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     CKobuki robot;
     TKobukiData robotdata;
     int datacounter;
     QTimer *timer;

     // ----------------------//
     // switch pre inicializaciu dat z robota po pripojeni
     bool initParam = true;
     bool navigate = false;

     // lokalizacia - stavove premenne
     double l_r, l_r_prev, l_l, l_l_prev, l_k;
     double x, x_prev, y, y_prev;
     double f_k, f_k_prev, d_alfa;
     unsigned int enc_l_prev, enc_r_prev;
     double total_l, total_r;

     // hodnoty mrtvych pasiem
     float pa1, pa2;
     // dead zone pre porovnanie s nulou
     double epsilon;
     // mrtve pasmo pre distane
     double pd;

     // fifo queue pre target pozicie;
     FifoQueue fifoTargets;

     bool rotationLock;
     int rotationDir;

     float P_distance;
     float P_angle;

     float prevRotSpeed, rotSpeed;
     float prevTransSpeed, transSpeed;

     float speedDifferenceLimit;

public slots:
     void setUiValues(double robotX,double robotY,double robotFi);

signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo
};

#endif // MAINWINDOW_H
