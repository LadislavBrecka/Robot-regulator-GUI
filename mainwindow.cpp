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
    speedDifferenceLimit = 50.0f;
    speedLimit = 400.0f;

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

void MainWindow::on_pushButton_7_clicked() // start navigate
{
    if (!fifoTargets.GetPoints().empty())
        navigate = !navigate;
    else
        std::cout << "No points in queue" << std::endl;
}

void MainWindow::on_pushButton_8_clicked()
{
    map.printMapToConsole();
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
    fifoTargets.In(desirePoint);
}

// show queue
void MainWindow::on_pushButton_11_clicked()
{
    PrintTargetQueue();
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
                int dist=copyOfLaserData.Data[k].scanDistance/20;
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();
                if(rect.contains(xp,yp))
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }
        }
    }
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

        pa1 = 0.05;
        pa2 = 0.2;
        pd = 0.05f;

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

    if(datacounter%5)
    {
        // pridavok vzdialenosti oboch kolies
        l_r = robot.tickToMeter * (enc_r_diff) * 1.0;
        l_l = robot.tickToMeter * (enc_l_diff) * 1.0;

        // celkova prejdena vzdialenost oboch kolies
        total_r += abs(l_r);
        total_l += abs(l_l);

        // vzdialenost l_k a uhol f_k
        l_k = (l_r + l_l) / 2;
        d_alfa = (l_r - l_l) / robot.b;
        f_k = f_k_prev + d_alfa;

        if (f_k > 2*PI)
            f_k -= 2*PI;

        else if (f_k < 0.0f)
            f_k += 2*PI;

        // suradnice x a y
        x = x_prev + l_k * cos(f_k);
        y = y_prev + l_k * sin(f_k);

        emit uiValuesChanged(x, y, (int)(RadToDegree(f_k)));

        enc_l_prev = robotdata.EncoderLeft;
        enc_r_prev = robotdata.EncoderRight;
        f_k_prev = f_k;
        x_prev = x;
        y_prev = y;

        // PID REGULATION
        if (!fifoTargets.GetPoints().empty() && navigate)
        {
            // first in, first out -> get me first element in queue
            Point target = fifoTargets.Out();
            Point actual(x, y);
            auto targetOffset = GetTargetOffset(actual, target);
            // regulacia rotacie
            RegulatorRotation(targetOffset.second);
            // ak prave neotacame, regulujeme doprednu rychlost
            RegulatorTranslation(targetOffset.first, targetOffset.second);
            // vyhodnotenie, ci splname poziadavky polohy
            EvaluateRegulation(targetOffset.first, targetOffset.second);
        }

        // CREATING MAP
        if (!isRotating)
        {
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                double dist  = copyOfLaserData.Data[k].scanDistance;

                if (dist > 150.0f)
                {
                    double angle = copyOfLaserData.Data[k].scanAngle;

                    double angle_sum = f_k + DegreeToRad(360.0f - angle);

                    if (angle_sum >= 2*PI)
                        angle_sum -= 2*PI;
                    else if (angle_sum < 0.0f)
                        angle_sum += 2*PI;

                    double x_lidar = x + (dist / 1000.0f) * cos(angle_sum);
                    double y_lidar = y + (dist / 1000.0f) * sin(angle_sum);

                    map.fillSquare(Point(x_lidar, y_lidar));
                }
            }
        }
    }
    datacounter++;
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

//    s/*td::cout << "dx: " << dx << ", dx: " << dy << ", fk: " << RadToDegree(f_k) << ", ";
//    st*/d::cout << "Distance: " << distance << ", Theta: " << RadToDegree(thetaOffset) << ", Alfa: " << RadToDegree(alfa) << std::endl;
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

    std::cout << "RotSpeed: " << rotSpeed << std::endl;
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

        std::cout << "TranslationSpeed: " << transSpeed <<std::endl;
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
        std::cout << std::endl << "Poping element!" << std::endl << std::endl;
        fifoTargets.Pop();

        if (fifoTargets.GetPoints().empty())
            navigate = false;

        prevTransSpeed = transSpeed;
        prevRotSpeed   = rotSpeed;
    }
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

    ui->textEdit->setText(message.c_str());
}
