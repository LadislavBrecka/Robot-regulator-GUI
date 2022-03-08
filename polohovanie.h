#ifndef POLOHOVANIE_H
#define POLOHOVANIE_H

#include<iostream>
#include<vector>
#include "CKobuki.h"

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


class Polohovanie
{
private:
    CKobuki robot;
    TKobukiData robotdata;

    // ----------------------//
    // switch pre inicializaciu dat z robota po pripojeni
    bool initParam = true;

    // lokalizacia - stavove premenne
    double l_r, l_r_prev, l_l, l_l_prev, l_k;
    double x, x_prev, y, y_prev;
    double f_k, f_k_prev, d_alfa;
    unsigned int enc_l_prev, enc_r_prev;
    double total_l, total_r;

    // hodnoty mrtvych pasiem
    float pa1, pa2;
    // mrtve pasmo pre distane
    double pd;

    // fifo queue pre target pozicie;
    FifoQueue fifoTargets;

    bool rotationLock;
    int rotationDir;

public:
    Polohovanie();

    // utility functions
    std::pair<double, double> GetTargetOffset(Point actual, Point target);
    double RadToDegree(double radians);
    double DegreeToRad(double degrees);
    void   PrintTargetQueue();

    // robot control functions
    void RobotSetTranslationSpeed(float speed);
    void RobotSetRotationSpeed(float speed);
    void RegulatorRotation();
    void RegulatorTranslation();
};

#endif // POLOHOVANIE_H
