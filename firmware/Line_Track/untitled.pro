TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

INCLUDEPATH += /usr/include/opencv\
               /usr/include/eigen3

LIBS    += -lopencv_highgui \
            -lopencv_photo \
            -lopencv_calib3d \
            -lopencv_imgproc \
            -lopencv_stitching \
            -lopencv_contrib \
            -lopencv_legacy \
            -lopencv_superres \
            -lopencv_core \
            -lopencv_ml \
            -lopencv_video \
            -lopencv_features2d \
            -lopencv_videostab \
            -lopencv_flann \
            -lopencv_objdetect \
            -lopencv_gpu \
            -lopencv_ocl \
            -laruco \
            -lpthread\
            -lserial\
            -lwiringPi\

