#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include<windows.h>
#include<iostream>
#include<queue>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
#include "ckobuki.h"
#include "rplidar.h"

#define MAX_SPEED_LIMIT 400
#define MAX_START_SPEED 50

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

#define MAP_WIDTH 144
#define MAP_HEIGHT 144
#define MAP_STEP 12

class Map
{
public:
   int map[MAP_WIDTH][MAP_HEIGHT];
   // Direction vectors
   int drow[4] = { -1, 0, 1, 0 };
   int dcol[4] = { 0, 1, 0, -1 };

public:
    Map()
    {
        for(int i = 0; i < MAP_HEIGHT; i++)
        {
            for(int j = 0; j < MAP_WIDTH; j++)
            {
                map[i][j] = 0;
            }
        }

        map[MAP_WIDTH/2][MAP_HEIGHT/2] = -1;
    }

    void setWall(Point lidar)
    {
        int x = round(lidar.x * MAP_STEP + MAP_WIDTH  / 2);
        int y = MAP_HEIGHT - round(lidar.y * MAP_STEP + MAP_HEIGHT / 2);
        map[y][x] = 1;
    }

    void fillNeightbours(int start_y, int start_x)
    {

        std::queue<std::pair<int,int>> q;
        q.push( {start_y, start_x} );

        int x, y, xx, yy;
        while (!q.empty())
        {
            auto c = q.front();
            y = c.first;
            x = c.second;

            if (map[y][x] == 1)
                continue;

            for(int i = 0; i < 4; i++)
            {
                xx = x + drow[i]; // travel in an adiacent cell from the current position
                yy = y + dcol[i];
                if (xx >= 0 && xx < MAP_WIDTH && yy >= 0 && yy < MAP_HEIGHT && map[yy][xx] == 0)
                {
                    if (map[yy][xx] != 1)
                    {
                        q.push( {yy, xx} );
                        map[yy][xx] = map[y][x] + 1; // you usually mark that you have been to this position in the matrix
                    }
                }
            }
            q.pop();
        }
    }

    void floodFillDistances(Point target)
    {
        int x = round(target.x * MAP_STEP + MAP_WIDTH  / 2);
        int y = MAP_HEIGHT - round(target.y * MAP_STEP + MAP_HEIGHT / 2);
        map[y][x] = 2;

        // floodfill
        fillNeightbours(y, x);
    }

    void saveToFile(std::string filename="map.txt")
    {
        QFile file(filename.c_str());
        file.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream out(&file);

        for (int i = 0; i < MAP_HEIGHT; ++i)
        {
            for (int j = 0; j < MAP_WIDTH; ++j)
            {
                if (map[i][j] == 1)
                    out << "*";
                else
                    out << " ";
            }
            out << "\n";
        }
        std::cout << "Map saved to file!" << std::endl;
        file.close();
    }

    void loadFromFile(std::string filename)
    {
        QFile file(filename.c_str());
        file.open(QIODevice::ReadOnly | QIODevice::Text);
        QTextStream in(&file);
        char temp;
        for (int i = 0; i < MAP_HEIGHT; ++i)
        {
            for (int j = 0; j < MAP_WIDTH; ++j)
            {
                in >> temp;
                if (temp == '*')
                    map[i][j] = 1;
                else if (temp == ' ')
                    map[i][j] = 0;
            }
            in >> temp;
        }

        std::cout << "Map load from file!" << std::endl;
    }

    void saveToFileRaw(std::string filename="map_raw.txt")
    {
        QFile file(filename.c_str());
        file.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream out(&file);

        for (int i = 0; i < MAP_HEIGHT; ++i)
        {
            for (int j = 0; j < MAP_WIDTH; ++j)
            {
                if (map[i][j] < 10)
                    out << "  " << map[i][j] << " ";
                else if (map[i][j] < 100)
                    out << " " << map[i][j] << " ";
                else
                    out << map[i][j];
            }
            out << "\n";
        }
        std::cout << "Raw map saved to file!" << std::endl;
        file.close();
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

    int actIndex;

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
    void mapping();

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

    void on_pushButton_8_clicked();

    void on_pushButton_clicked();

    void on_checkBox_clicked(bool checked);

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

     // switche
     bool initParam = true;
     bool navigate = false;
     bool map_mode = false;
     bool isRotating = false;
     bool maping_nav = false;

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
     // mrtve pasmo pre distance
     double pd;

     // fifo queue pre target pozicie;
     FifoQueue fifoTargets;
     Map map;

     bool rotationLock;
     int rotationDir;

     float P_distance;
     float P_angle;

     float prevRotSpeed, rotSpeed;
     float prevTransSpeed, transSpeed;

     float speedDifferenceLimit;
     float speedLimit;

public slots:
     void setUiValues(double robotX,double robotY,double robotFi);

signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo
};

#endif // MAINWINDOW_H
