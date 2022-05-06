#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <cstdlib>


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="127.0.0.1";
    ui->setupUi(this);
    datacounter=0;
    actIndex=-1;
    useCamera=false;

    datacounter=0;

    P_distance = 500.0f;
    P_angle = 2.5f;
    speedDifferenceLimit = MAX_START_SPEED;
    speedLimit = MAX_SPEED_LIMIT;

    prevRotSpeed = rotSpeed = 0.0f;
    prevTransSpeed = transSpeed = 0.0f;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    RobotSetTranslationSpeed(500);
}

void MainWindow::on_pushButton_3_clicked() //back
{
    RobotSetTranslationSpeed(-250);
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    RobotSetTranslationSpeed(0);
}

void MainWindow::on_pushButton_5_clicked()//right
{
    RobotSetRotationSpeed(-3.14159/2);
}

void MainWindow::on_pushButton_6_clicked() //left
{
    RobotSetRotationSpeed(3.14159/2);
}

void MainWindow::on_pushButton_clicked()
{
    mapping();
}

void MainWindow::on_checkBox_1_clicked(bool checked)
{
    maping_nav = checked;
    if (reactive_nav)
    {
        reactive_nav = false;
        ui->checkBox_2->setChecked(false);
    }

    ui->checkBox_3->setChecked(false);
}

void MainWindow::on_checkBox_2_clicked(bool checked)
{
    reactive_nav = checked;
    if (maping_nav)
    {
        maping_nav = false;
        ui->checkBox_1->setChecked(false);
    }

    ui->checkBox_3->setChecked(false);
}

void MainWindow::on_checkBox_3_clicked(bool checked)
{
    if (reactive_nav)
    {
        reactive_nav = false;
        ui->checkBox_2->setChecked(false);
    }

    if (maping_nav)
    {
        maping_nav = false;
        ui->checkBox_1->setChecked(false);
    }
}

void MainWindow::on_pushButton_9_clicked() //start button
{
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    laserthreadHandle=CreateThread(NULL,0,laserUDPVlakno, (void *)this,0,&laserthreadID);
    robotthreadHandle=CreateThread(NULL,0, robotUDPVlakno, (void *)this,0,&robotthreadID);
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));
}

// add point to queue
void MainWindow::on_pushButton_10_clicked()
{
    QString xString = ui->lineEdit_5->text();
    QString yString = ui->lineEdit_6->text();
    Point desirePoint = {xString.toFloat(), yString.toFloat()};

    // mapping mode
    if (maping_nav)
    {
        int drow[4] = { -1, 0, 1, 0 };
        int dcol[4] = { 0, 1, 0, -1 };
        std::queue<std::pair<int,int>> q;

        // for storing only points on the edge
        std::vector<Point> desired_points;

        int tx, ty, xx, yy, next_x, next_y, dir, prev_dir;

        dir = 0;
        prev_dir = 0;

        // floodfilling BSF
        map.loadFromFile("map_enlarged.txt");
        map.floodFillDistances(desirePoint);
        map.saveToFileRaw("map_flodfill.txt");

        int robot_map_x = round(x * MAP_STEP + MAP_WIDTH  / 2);
        int robot_mapp_y = MAP_HEIGHT - round(y * MAP_STEP + MAP_HEIGHT / 2);
        q.push( {robot_mapp_y, robot_map_x} );

        while (!q.empty())
        {
            auto c = q.front();
            ty = c.first;
            tx = c.second;

            int min_value = 100000;
            for(int i = 0; i < 4; i++)
            {
                xx = tx + drow[i]; // travel in an adiacent cell from the current position
                yy = ty + dcol[i];
                if (xx >= 0 && xx < MAP_WIDTH && yy >= 0 && yy < MAP_HEIGHT && map.map[yy][xx] > 1)
                {
                    if (min_value > map.map[yy][xx])
                    {
                        min_value = map.map[yy][xx];
                        next_x = xx;
                        next_y = yy;
                        dir = i;
                    }
                }
            }

            if (map.map[next_y][next_x] > 2)
            {
                q.push( {next_y, next_x} );

                if (prev_dir != dir)
                {
                    float real_x = (float)(tx - MAP_WIDTH/2.0f) / MAP_STEP;
                    float real_y = ((float)(ty - MAP_HEIGHT) + (float)(MAP_HEIGHT/2.0f)) / MAP_STEP;
                    real_x = int(real_x * 100) / 100.0f;
                    real_y = int(real_y * 100) / 100.0f;

                    map.map[ty][tx] = 0;
                    if (real_x != 0 || real_y != 0)
                        fifoTargets.In(Point(real_x,-real_y));
                }
            }

            prev_dir = dir;
            q.pop();
        }

        fifoTargets.In(desirePoint);
        map.saveToFileRaw("map_path.txt");
    }
    else if (reactive_nav)
    {
        pTarget = desirePoint;
    }

    else
    {
        fifoTargets.In(desirePoint);
    }

    navigate = !navigate;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);
    QRect rect(20,120,700,500);
    rect= ui->frame->geometry();
    rect.translate(0,15);
    painter.drawRect(rect);

    PrintTargetQueue();

//    if(useCamera==true)
//    {
//        std::cout<<actIndex<<std::endl;
//        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );
//        painter.drawImage(rect,image.rgbSwapped());
//    }
//    else
    {
        if(updateLaserPicture==1)
        {
            updateLaserPicture=0;

            painter.setPen(pero);
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                int dist=copyOfLaserData.Data[k].scanDistance/30;
                int xp=rect.width()-(rect.width()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0 + f_k))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0 + f_k))+rect.topLeft().y();
                if(rect.contains(xp,yp))
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }
        }
    }

//    auto left_offset = GetTargetOffset(Point(x, y), pLeftObstacle);
//    auto right_offset = GetTargetOffset(Point(x, y), pRightObstacle);

//    painter.setPen(QPen(Qt::red));

//    int dist=left_offset.first/30;
//    int xp=rect.width()-(rect.width()/2+dist*2*sin(left_offset.second + f_k))+rect.topLeft().x();
//    int yp=rect.height()-(rect.height()/2+dist*2*cos(left_offset.second + f_k))+rect.topLeft().y();
//    if(rect.contains(xp,yp))
//        painter.drawEllipse(QPoint(xp, yp),2,2);

//    dist=right_offset.first/30;
//    xp=rect.width()-(rect.width()/2+dist*2*sin(right_offset.second + f_k))+rect.topLeft().x();
//    yp=rect.height()-(rect.height()/2+dist*2*cos(right_offset.second + f_k))+rect.topLeft().y();
//    if(rect.contains(xp,yp))
//        painter.drawEllipse(QPoint(xp, yp),2,2);


}

void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
}


void MainWindow::processThisRobot()
{
    // inicializacia pri prvom spusteni
    if (initParam)
    {
        // inicializacia mojich premennych
        x = 0.0f;
        y = 0.0f;

        enc_l_prev = robotdata.EncoderLeft;
        enc_r_prev = robotdata.EncoderRight;
        f_k_prev = 0.0f;
        x_prev = 0.0f;
        y_prev = 0.0f;
        total_l = 0.0f;
        total_r = 0.0f;

        pa1 = 0.012;
        pa2 = 0.04;
        pd = 0.02f;

        rotationLock = false;
        rotationDir = 0;

        initParam = false;
    }

    // pridavok enkoderov oboch kolies
    int enc_r_diff = robotdata.EncoderRight - enc_r_prev;
    int enc_l_diff = robotdata.EncoderLeft - enc_l_prev;

    // pretecenie praveho enkodera
    if (enc_r_diff < -60000)
        enc_r_diff = 65535 + enc_r_diff;
    else if (enc_r_diff > 60000)
        enc_r_diff = enc_r_diff - 65535;

    // pretecenie laveho enkodera
    if (enc_l_diff < -60000)
        enc_l_diff = 65535 + enc_l_diff;
    else if (enc_l_diff > 60000)
        enc_l_diff = enc_l_diff - 65535;

    // pridavok vzdialenosti oboch kolies
    l_r = robot.tickToMeter * (enc_r_diff);
    l_l = robot.tickToMeter * (enc_l_diff);

    // celkova prejdena vzdialenost oboch kolies
    total_r += abs(l_r);
    total_l += abs(l_l);

    // vzdialenost l_k a uhol f_k
    l_k = (l_r + l_l) / 2;
    d_alfa = (l_r - l_l) / robot.b;
    f_k = f_k_prev + d_alfa;

    // pretecenie uhla
    if (f_k > 2*PI)
        f_k -= 2*PI;
    else if (f_k < 0.0f)
        f_k += 2*PI;

    // suradnice x a y
    x = x_prev + l_k * cos(f_k);
    y = y_prev + l_k * sin(f_k);

    // vypis do GUI
    emit uiValuesChanged(x, y, (int)(RadToDegree(f_k)));

    // ulozenie aktualnych do predoslych
    enc_l_prev = robotdata.EncoderLeft;
    enc_r_prev = robotdata.EncoderRight;
    f_k_prev = f_k;
    x_prev = x;
    y_prev = y;

    // reactive navigation
    if (navigate && reactive_nav && fifoTargets.GetPoints().empty() && !directPossible)
    {
        unreachable = !PointCanBeReached(pTarget);

        if (unreachable)
        {
            std::vector<Point> vLeftObstacles;
            std::vector<Point> vRightObstacles;

            auto offsetToTarget = GetTargetOffset(Point(x, y), pTarget);
            int offset_theta = RadToDegree(offsetToTarget.second);

            if (offset_theta < 0)
                offset_theta = 360 + offset_theta;

            // compute percentage from 0 to 360
            double offset_percentage = (100.0 * offset_theta) / 360.0;
            int olp = (copyOfLaserData.numberOfScans * offset_percentage) / 100.0f;
            std::cout << "Theta percentage from robot: " << offset_percentage << std::endl;

            // left side of obstacle
            double prev_D = copyOfLaserData.Data[olp-1].scanDistance / 1000.0f;
            for(int k = olp; k <= copyOfLaserData.numberOfScans; k++)
            {
                double dist  = copyOfLaserData.Data[k].scanDistance/ 1000.0f;

                if (dist > 0.1f && abs(prev_D - dist) > 0.8f)
                {
                    double dist_prev  = copyOfLaserData.Data[k-1].scanDistance / 1000.0f;
                    double angle_prev = DegreeToRad(360.0f - copyOfLaserData.Data[k-1].scanAngle);

                    double dist_act  = copyOfLaserData.Data[k].scanDistance / 1000.0f;
                    double angle_act = DegreeToRad(360.0f - copyOfLaserData.Data[k].scanAngle);

                    if (angle_prev >= 2*PI)     angle_prev -= 2*PI;
                    else if (angle_prev < 0.0f) angle_prev  += 2*PI;

                    if (angle_act >= 2*PI)     angle_act -= 2*PI;
                    else if (angle_act < 0.0f) angle_act  += 2*PI;

                    float foundObstacleX_prev = x + dist_prev * cos(f_k + angle_prev);
                    float foundObstacleY_prev = y + dist_prev * sin(f_k + angle_prev);

                    float foundObstacleX_act = x + dist_act * cos(f_k + angle_act);
                    float foundObstacleY_act = y + dist_act * sin(f_k + angle_act);

                    std::cout << "Found LEFT k-1 point at: [" << foundObstacleX_prev << ", " << foundObstacleY_prev << "]" << std::endl;
                    std::cout << "Found LEFT k point at: [" << foundObstacleX_act << ", " << foundObstacleY_act << "]" << std::endl;

                    // k-1 point
                    double new_a;
                    if (y > foundObstacleY_prev && x < foundObstacleX_prev+0.1)
                        new_a = (f_k + angle_prev) - atan2(1.5f * robot.b, dist_prev);
                    else
                        new_a = (f_k + angle_prev) + atan2(1.5f * robot.b, dist_prev);
                    double p = sqrt(pow(dist_prev, 2) + pow(1.5f * robot.b, 2)) + 2.0f * robot.b;
                    float foundObstacleX_offset_prev = x + p * cos(new_a);
                    float foundObstacleY_offset_prev = y + p * sin(new_a);

                    // k point
                    if (y > foundObstacleY_act && x < foundObstacleX_act+0.1)
                        new_a = (f_k + angle_act) - atan2(1.5f * robot.b, dist_act);
                    else
                        new_a = (f_k + angle_act) + atan2(1.5f * robot.b, dist_act);
                    p = sqrt(pow(dist_act, 2) + pow(1.5f * robot.b, 2)) + 2.0f * robot.b;
                    float foundObstacleX_offset_act = x + p * cos(new_a);
                    float foundObstacleY_offset_act = y + p * sin(new_a);

                    vLeftObstacles.push_back({foundObstacleX_offset_prev, foundObstacleY_offset_prev});
                    vLeftObstacles.push_back({foundObstacleX_offset_act, foundObstacleY_offset_act});
                    break;
                }
                prev_D = dist;
            }

            // right side of obstacle
            prev_D = copyOfLaserData.Data[olp+1].scanDistance / 1000.0f;
            for(int k = olp; k > 0; k--)
            {
                double dist  = copyOfLaserData.Data[k].scanDistance / 1000.0f;

                if (dist > 0.1f && abs(prev_D - dist) > 0.8f)
                {
                    double dist_prev  = copyOfLaserData.Data[k+1].scanDistance / 1000.0f;
                    double angle_prev = DegreeToRad(360.0f - copyOfLaserData.Data[k+1].scanAngle);

                    double dist_act  = copyOfLaserData.Data[k].scanDistance / 1000.0f;
                    double angle_act = DegreeToRad(360.0f - copyOfLaserData.Data[k].scanAngle);

                    if (angle_prev >= 2*PI)     angle_prev -= 2*PI;
                    else if (angle_prev < 0.0f) angle_prev  += 2*PI;

                    if (angle_act >= 2*PI)     angle_act -= 2*PI;
                    else if (angle_act < 0.0f) angle_act  += 2*PI;

                    float foundObstacleX_prev = x + dist_prev * cos(f_k + angle_prev);
                    float foundObstacleY_prev = y + dist_prev * sin(f_k + angle_prev);

                    float foundObstacleX_act = x + dist_act * cos(f_k + angle_act);
                    float foundObstacleY_act = y + dist_act * sin(f_k + angle_act);

                    std::cout << "Found RIGHT k+1 point at: [" << foundObstacleX_prev << ", " << foundObstacleY_prev << "]" << std::endl;
                    std::cout << "Found RIGHT k point at: [" << foundObstacleX_act << ", " << foundObstacleY_act << "]" << std::endl;

                    // k-1 point
                    double new_a;
                    if (y > foundObstacleY_prev && x < foundObstacleX_prev+0.1)
                        new_a = (f_k + angle_prev) + atan2(1.5f * robot.b, dist_prev);
                    else
                        new_a = (f_k + angle_prev) - atan2(1.5f * robot.b, dist_prev);
                    double p = sqrt(pow(dist_prev, 2) + pow(1.5f * robot.b, 2)) + 2.0f * robot.b;
                    float foundObstacleX_offset_prev = x + p * cos(new_a);
                    float foundObstacleY_offset_prev = y + p * sin(new_a);

                    // k point
                    if (y > foundObstacleY_act && x < foundObstacleX_act+0.1)
                        new_a = (f_k + angle_act) + atan2(1.5f * robot.b, dist_act);
                    else
                        new_a = (f_k + angle_act) - atan2(1.5f * robot.b, dist_act);
                    p = sqrt(pow(dist_act, 2) + pow(1.5f * robot.b, 2)) + 2.0f * robot.b;
                    float foundObstacleX_offset_act = x + p * cos(new_a);
                    float foundObstacleY_offset_act = y + p * sin(new_a);

                    vRightObstacles.push_back({foundObstacleX_offset_prev, foundObstacleY_offset_prev});
                    vRightObstacles.push_back({foundObstacleX_offset_act, foundObstacleY_offset_act});
                    break;
                }
                prev_D = dist;
            }

            double leftMin = 9999.9;
            for (Point pLeft : vLeftObstacles)
            {
                auto robotToLeftObstacleDistance  = GetTargetOffset(Point(x,y), pLeft);
                auto leftObstacleToTargetDistance  = GetTargetOffset(pLeft, pTarget);
                double leftObstacleDistance = robotToLeftObstacleDistance.first + leftObstacleToTargetDistance.first;
                if (leftObstacleDistance < leftMin)
                {
                    leftMin = leftObstacleDistance;
                    pLeftObstacle = pLeft;
                }
            }

            double righttMin = 9999.9;
            for (Point pRight : vRightObstacles)
            {
                auto robotToRightObstacleDistance  = GetTargetOffset(Point(x,y), pRight);
                auto rightObstacleToTargetDistance  = GetTargetOffset(pRight, pTarget);
                double rightObstacleDistance = robotToRightObstacleDistance.first + rightObstacleToTargetDistance.first;
                if (rightObstacleDistance < righttMin)
                {
                    righttMin = rightObstacleDistance;
                    pRightObstacle = pRight;
                }
            }

            if (leftMin < righttMin)
            {
                std::cout << "Left wins: [" << pLeftObstacle.x << ", " << pLeftObstacle.y << "]" << std::endl;
                if (pLeftObstacle.isValid())
                {
                    fifoTargets.Pop();
                    fifoTargets.In(pLeftObstacle);
                }
            }
            else
            {
                std::cout << "Right wins: [" << pRightObstacle.x << ", " << pRightObstacle.y << "]" << std::endl;
                if (pRightObstacle.isValid())
                {
                    fifoTargets.Pop();
                    fifoTargets.In(pRightObstacle);
                }
            }
        }
        else
        {
            fifoTargets.In(pTarget);
        }

    }

//    unreachable = !PointCanBeReached(pTarget);


    // REGULATION
    if (!fifoTargets.GetPoints().empty() && (navigate || map_mode))
    {
        // first in, first out -> dat mi prvy element zo zasobnika
        Point target = fifoTargets.Out();
        Point actual(x, y);
        // ziskanie chyby polohy
        auto targetOffset = GetTargetOffset(actual, target);
        // regulacia rotacie
        RegulatorRotation(targetOffset.second);
        // ak nerotujeme, regulujeme doprednu rychlost
        RegulatorTranslation(targetOffset.first, targetOffset.second);
        // vyhodnotenie, ci splname poziadavky polohy
        EvaluateRegulation(targetOffset.first, targetOffset.second);
    }

    // CREATING MAP
    if (!isRotating && map_mode)
    {
        for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
        {
            double dist  = copyOfLaserData.Data[k].scanDistance;

            if (dist > 150.0f && dist < 1500.0f)
            {
                double angle = copyOfLaserData.Data[k].scanAngle;

                double angle_sum = f_k + DegreeToRad(360.0f - angle);

                if (angle_sum >= 2*PI)
                    angle_sum -= 2*PI;
                else if (angle_sum < 0.0f)
                    angle_sum += 2*PI;

                double x_lidar = x + (dist / 1000.0f) * cos(angle_sum);
                double y_lidar = y + (dist / 1000.0f) * sin(angle_sum);

                map.setWall(Point(x_lidar, y_lidar));
            }
        }
    }
    datacounter++;
}


// key moving
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_W)
    {
        RobotSetTranslationSpeed(MAX_SPEED_LIMIT);
    }

    if(event->key() == Qt::Key_A)
    {
        RobotSetRotationSpeed(PI/6.0f);
    }

    if(event->key() == Qt::Key_S)
    {
        RobotSetTranslationSpeed(0.0f);
    }

    if(event->key() == Qt::Key_D)
    {
        RobotSetRotationSpeed(-PI/6.0f);
    }

    if(event->key() == Qt::Key_X)
    {
        RobotSetTranslationSpeed(-MAX_SPEED_LIMIT);
    }
}

void MainWindow::processThisLidar(LaserMeasurement &laserData)
{
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    // tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture=1;
    // tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia
    update();
}

void MainWindow::laserprocess()
{
    WSADATA wsaData = {0};
    int iResult = 0;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    las_slen = sizeof(las_si_other);
    if ((las_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    int las_broadcastene=1;
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,(char*)&las_broadcastene,sizeof(las_broadcastene));
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);
    las_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());//htonl(INADDR_BROADCAST);
    bind(las_s, (struct sockaddr*)&las_si_me, sizeof(las_si_me));
    char command=0x00;

    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, las_slen) == -1)
    {

    }

    LaserMeasurement measure;

    while(1)
    {
        if ((las_recv_len = recvfrom(las_s, (char*)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other, (int*)&las_slen)) == -1)
        {
            continue;
        }

        measure.numberOfScans=las_recv_len/sizeof(LaserData);
        // tu mame data..zavolame si funkciu
        // memcpy(&sens,buff,sizeof(sens));
        processThisLidar(measure);
    }
}


void MainWindow::robotprocess()
{
    WSADATA wsaData = {0};
    int iResult = 0;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);

    if ((rob_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char rob_broadcastene=1;
    setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr =inet_addr(ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();

    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }

    Sleep(100);
    mess=robot.setSound(440,1000);

    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }

    unsigned char buff[50000];

    while(1)
    {
        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other,(int*) &rob_slen)) == -1)
        {

            continue;
        }

        int returnval=robot.fillData(robotdata,(unsigned char*)buff);
        if(returnval==0)
        {
            processThisRobot();
        }
    }
}

void MainWindow::getNewFrame()
{

}

void MainWindow::RobotSetTranslationSpeed(float speed)
{
    isRotating = false;
    //pohyb dopredu
    std::vector<unsigned char> mess=robot.setTranslationSpeed(speed);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::RobotSetRotationSpeed(float speed)
{
    if (speed != 0.0)
        isRotating = true;

    std::vector<unsigned char> mess=robot.setRotationSpeed(speed);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

std::pair<double, double> MainWindow::GetTargetOffset(Point actual, Point target)
{
    float dx = target.x - actual.x;
    float dy = target.y - actual.y;
    double alfa;

    // ak je zmena v osi y nulova
    if (dy == 0.0f)
    {
        alfa = 0.0f;
    }
    //ak je zmena v osi x nulova
    else if (dx == 0.0f)
    {
        alfa = PI/2.0f;
    }
    // ak ani jeden predosli pripad nenastal, je delenie bezpecne
    else
    {
        alfa = atan(fabs(dy)/fabs(dx));
    }

    bool negativeX =  dx < 0.0f;
    bool negativeY =  dy < 0.0f;

    // umiestnenie uhla do spravneho kvadrantu -> theta e <0, 360>
    // 2. kvadrant
    if (negativeX && !negativeY)
    {
        alfa = PI - alfa;
    }
    // 3. kvadrant
    else if (negativeX && negativeY)
    {
       alfa = alfa + PI;
    }
    // 4. kvadrant
    else if (!negativeX && negativeY)
    {
       alfa = 2*PI - alfa;
    }

    float distance = sqrt(pow(dx, 2) + pow(dy, 2));
    double thetaOffset = 0.0f;
    if (f_k < alfa)
    {
        thetaOffset = alfa - f_k;
        if (thetaOffset > PI)
        {
            thetaOffset = 2*PI - thetaOffset;
            thetaOffset *= (-1);
        }
    }
    else if (f_k > alfa)
    {
        thetaOffset = f_k - alfa;
        if (thetaOffset <= PI)
        {
            thetaOffset *= (-1);
        }
        else if (thetaOffset > PI)
        {
            thetaOffset = 2*PI - thetaOffset;
        }
    }

   return std::pair<double, double>(distance, thetaOffset);
}

void MainWindow::RegulatorRotation(double dTheta)
{
    // ak je uhol vacsi ako mrtve pasmo pa2, regulator reguluje uhol natocenia
    if (fabs(dTheta) > pa2)
    {
        // P Regulator s rampou
        float idealSpeed = P_angle * dTheta;
        if (idealSpeed > prevRotSpeed + speedDifferenceLimit)
            rotSpeed += speedDifferenceLimit;
        else
            rotSpeed = idealSpeed;
    }

    // ak je uhol mansie ako vnutorne mrtve pasmo pa1, ukoncujeme regulaciu uhla natocenia
    else if (fabs(dTheta) < pa1)
    {
       rotSpeed = 0.0f;
    }

    if (rotSpeed > speedLimit)
        rotSpeed = speedLimit;

    RobotSetRotationSpeed(rotSpeed);
    prevRotSpeed = rotSpeed;
}

void MainWindow::RegulatorTranslation(double distance, double dTheta)
{
    // ak je robot spravne natoceny a neprebieha rotacia, regulator reguluje doprednu rychlost
    if (distance > pd && abs(dTheta) <= pa2)
    {
        float idealSpeed = P_distance * distance;

        if (idealSpeed > prevTransSpeed + speedDifferenceLimit)
            transSpeed += speedDifferenceLimit;
        else
            transSpeed = idealSpeed;

        if (transSpeed > speedLimit)
            transSpeed = speedLimit;

        RobotSetTranslationSpeed(transSpeed);
    }

    prevTransSpeed = transSpeed;
}

void MainWindow::EvaluateRegulation(double distance, double theta)
{
    if (distance <= pd)
    {
        // dosiahnutie ciela
        transSpeed = 0.0f;
        rotSpeed   = 0.0f;

        // tato funkcia nastavi 0 rychlost na obe kolesa
        RobotSetTranslationSpeed(transSpeed);

        // vymazanie bodu zo zasobnika
        if (!map_mode)
            std::cout << std::endl << "Poping element!" << std::endl << std::endl;
        fifoTargets.Pop();

        if (fifoTargets.GetPoints().empty() && !map_mode && !reactive_nav)
            navigate = false;

        if (directPossible)
        {
            navigate = false;
            directPossible = false;
        }

        if (fifoTargets.GetPoints().empty() && map_mode)
        {
            map_mode = false;
            speedLimit = MAX_SPEED_LIMIT;
            speedDifferenceLimit = MAX_START_SPEED;
            map.saveToFile("map.txt");

            // enlarge walls
            int new_map[MAP_WIDTH][MAP_HEIGHT];
            for(int i = 0; i < MAP_HEIGHT; i++)
            {
                for(int j = 0; j < MAP_WIDTH; j++)
                {
                    new_map[i][j] = map.map[i][j];
                }
            }

            for(int i = 0; i < MAP_HEIGHT; i++)
            {
                for(int j = 0; j < MAP_WIDTH; j++)
                {
                    if (map.map[i][j] == 1)
                    {
                        new_map[i+1][j] = 1;
                        new_map[i+2][j] = 1;

                        new_map[i-1][j] = 1;
                        new_map[i-2][j] = 1;

                        new_map[i][j-1] = 1;
                        new_map[i][j-2] = 1;

                        new_map[i][j+1] = 1;
                        new_map[i][j+2] = 1;

                        // diagonal
                        new_map[i+1][j+1] = 1;
                        new_map[i+2][j+2] = 1;

                        new_map[i-1][j-1] = 1;
                        new_map[i-2][j-2] = 1;

                        new_map[i-1][j+1] = 1;
                        new_map[i-2][j+2] = 1;

                        new_map[i+1][j-1] = 1;
                        new_map[i+1][j-2] = 1;


                    }
                }
            }

            for(int i = 0; i < MAP_HEIGHT; i++)
            {
                for(int j = 0; j < MAP_WIDTH; j++)
                {
                    map.map[i][j] = new_map[i][j];
                }
            }


            map.saveToFile("map_enlarged.txt");
        }

        prevTransSpeed = transSpeed;
        prevRotSpeed   = rotSpeed;
    }
}

bool MainWindow::PointCanBeReached(Point target)
{
    // check with zone if point is reachable
    Point actual(x, y);
    // ziskanie chyby polohy
    auto targetOffset = GetTargetOffset(actual, target);

    if (reactive_nav)
    {
        for(int k=0;k<copyOfLaserData.numberOfScans;k++)
        {
            float D = copyOfLaserData.Data[k].scanDistance / 1000.0;
            float a = DegreeToRad(360 - copyOfLaserData.Data[k].scanAngle);
            double targetOffsetAngle = targetOffset.second < 0 ? targetOffset.second + 2*PI : targetOffset.second;
            double pointAngleZone = abs(a - targetOffsetAngle);

            if (D > 0.15 && D < targetOffset.first && (pointAngleZone < PI/2 || pointAngleZone > (3/2)*PI))
            {
                float dCrit = 1.2*robot.b / sin(pointAngleZone);
                if (dCrit > D)
                {
                    std::cout << "Point cannot be reached!" << std::endl;
                    return false;
                }
            }
        }
    }

    std::cout << "Point can be reached!" << std::endl;
    directPossible = true;
    return true;
}

double MainWindow::RadToDegree(double radians)
{
    return radians * (180/PI);
}

double MainWindow::DegreeToRad(double degrees)
{
    return degrees * (PI/180);
}

void MainWindow::PrintTargetQueue()
{
    std::vector<Point> targets = fifoTargets.GetPoints();

    std::string message;

    if (!targets.empty())
    {
        for (auto v : targets)
                message += " --  X: " + std::to_string(v.x) + ", Y:" + std::to_string(v.y) + " --  ";
    }
}

void MainWindow::mapping()
{
    speedLimit = 150.0f;
    speedDifferenceLimit = 25.0;
    map_mode = true;
    fifoTargets.In(Point(1, 0.0));
    fifoTargets.In(Point(0, 0.5));
    fifoTargets.In(Point(0, 3.0));
    fifoTargets.In(Point(2.8, 3));
    fifoTargets.In(Point(2.8, 3.8));
    fifoTargets.In(Point(4.2, 3.8));  // 1 test uloha 4
    fifoTargets.In(Point(4.2, 3.0));
    fifoTargets.In(Point(4, 3.8));
    fifoTargets.In(Point(2.8, 3.8));  // 2 test uloha 4
    fifoTargets.In(Point(2.8, 0));
    fifoTargets.In(Point(4.5, 0.0));
    fifoTargets.In(Point(4.5, 2.0));
}


