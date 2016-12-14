MOC_DIR = .moc
OBJECTS_DIR = .obj

TEMPLATE = app
CONFIG += qt warn_on release thread

SOURCES = video-player-good.cpp
TARGET = video-player-good
CONFIG -= app_bundle

enseirb {
    INCLUDEPATH += /pot/opencv/include/opencv2
    QMAKE_LFLAGS += -Wl,-R/opt/opencv/lib -L/opt/opencv/lib
}

LIBS += \
-lopencv_core \
-lopencv_highgui \
-lopencv_imgproc \
-lopencv_features2d \
-lopencv_calib3d
