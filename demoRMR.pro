#-------------------------------------------------
#
# Project created by QtCreator 2018-02-11T14:35:38
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = demoRMR
TEMPLATE = app
LIBS += -lws2_32
LIBS += -lWinmm

    INCLUDEPATH += C:/opencv/include/

    LIBS +=-LC:/opencv/bin
    LIBS +=-LC:/opencv/lib
    #LIBS +=-LH:/opencv_qt_build/lib/ -llibopencv_core340
    #LIBS +=-LH:/opencv_qt_build/lib/ -llibopencv_highgui340
    #LIBS +=-LH:/opencv_qt_build/lib/ -llibopencv_imgcodecs340
    #LIBS +=-LH:/opencv_qt_build/lib/ -llibopencv_imgproc340
    #LIBS +=-LH:/opencv_qt_build/lib/ -llibopencv_features2d340
    #LIBS +=-LH:/opencv_qt_build/lib/ -llibopencv_calib3d340
    #LIBS +=-LH:/opencv_qt_build/lib/ -llibopencv_videoio340
    #LIBS+=-LH:/opencv_qt_build/bin

#    CONFIG(debug, debug|release) {
#        LIBS +=-LC:/opencv/lib/ -lopencv_core340d
#        LIBS +=-LC:/opencv/lib/ -lopencv_highgui340d
#        LIBS +=-LC:/opencv/lib/ -lopencv_imgcodecs340d
#        LIBS +=-LC:/opencv/lib/ -lopencv_imgproc340d
#        LIBS +=-LC:/opencv/lib/ -lopencv_features2d340d
#        LIBS +=-LC:/opencv/lib/ -lopencv_calib3d340d
#        LIBS +=-LC:/opencv/lib/ -lopencv_videoio340d
#        LIBS +=-LC:/opencv/lib/ -lopencv_ml340d
#        LIBS +=-LC:/opencv/lib/ -lopencv_dnn340d
#        LIBS +=-LC:/opencv/lib/ -lopencv_flann340d
#        LIBS +=-LC:/opencv/lib/ -lopencv_objdetect340d
#        LIBS +=-LC:/opencv/lib/ -lopencv_photo340d
#        LIBS +=-LC:/opencv/lib/ -lopencv_shape340d
#        LIBS +=-LC:/opencv/lib/ -lopencv_video340d
#    }
#    else {
#        LIBS +=-LC:/opencv/lib/ -lopencv_core340
#        LIBS +=-LC:/opencv/lib/ -lopencv_highgui340
#        LIBS +=-LC:/opencv/lib/ -lopencv_imgcodecs340
#        LIBS +=-LC:/opencv/lib/ -lopencv_imgproc340
#        LIBS +=-LC:/opencv/lib/ -lopencv_features2d340
#        LIBS +=-LC:/opencv/lib/ -lopencv_calib3d340
#        LIBS +=-LC:/opencv/lib/ -lopencv_videoio340
#        LIBS +=-LC:/opencv/lib/ -lopencv_ml340
#        LIBS +=-LC:/opencv/lib/ -lopencv_dnn340
#        LIBS +=-LC:/opencv/lib/ -lopencv_flann340
#        LIBS +=-LC:/opencv/lib/ -lopencv_objdetect340
#        LIBS +=-LC:/opencv/lib/ -lopencv_photo340
#        LIBS +=-LC:/opencv/lib/ -lopencv_shape340
#        LIBS +=-LC:/opencv/lib/ -lopencv_video340
#    }


SOURCES += main.cpp\
    mainwindow.cpp \
    rplidar.cpp \
    CKobuki.cpp

HEADERS  += \
    mainwindow.h \
    rplidar.h \
    CKobuki.h

FORMS    += \
    mainwindow.ui
