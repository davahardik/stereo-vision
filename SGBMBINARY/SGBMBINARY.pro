QT += core
QT -= gui

CONFIG += c++11

TARGET = Cameracalibration
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH +="usr/local/include/"
LIBS +=`pkg-config --libs opencv`
INCLUDEPATH += /usr/local/include/pcl-1.8/
INCLUDEPATH += /usr/local/include/vtk-6.3
INCLUDEPATH += /usr/local/include/eigen3/

#LIBS += -L/home/hemanth/pcl-pcl-1.8.0/buildLastQtVtk/install/lib
#LIBS += -L/home/hemanth/VTK-6.3.0/buildLastQT1/install/lib

INCLUDEPATH += /home/Documents/VTK-6.3.0/GUISupport/Qt
LIBS += -L/usr/local/lib

LIBS +=  -lpcl_visualization
LIBS += -lpcl_common
LIBS +=  -lboost_system
LIBS += -lpcl_io
LIBS += -lpcl_kdtree
LIBS += -lpcl_search
LIBS +=-lpcl_registration
LIBS +=  -lpcl_keypoints
LIBS += -lpcl_features
LIBS += -lpcl_filters
LIBS += -lpcl_surface

LIBS +=  -lvtkCommonDataModel-6.3
LIBS +=  -lvtkCommonCore-6.3
LIBS +=  -lvtkImagingCore-6.3
LIBS += -lvtkInteractionStyle-6.3
#LIBS += -lvtkFiltering-6.3
LIBS +=-lvtksys-6.3
LIBS +=  -lvtkCommonExecutionModel-6.3
LIBS +=  -lvtkRenderingLOD-6.3
LIBS +=  -lvtkRenderingCore-6.3
LIBS +=  -lvtkFiltersSources-6.3
LIBS +=  -lvtkalglib-6.3
#LIBS += -lvtkGUISupportQt-6.3
#LIBS += -lvtkRenderingAnnotation-6.3
LIBS +=  -lvtkIOCore-6.3
#LIBS +=  -lvtkIOImage-6.3
LIBS += -lvtkzlib-6.3
LIBS += -lvtkpng-6.3
LIBS += -lvtkIOPLY-6.3
LIBS += -lvtkViewsCore-6.3
LIBS += -lvtkCommonSystem-6.3
LIBS += -lvtkCommonMisc-6.3
LIBS += -lvtkCommonMath-6.3
LIBS += -lvtkCommonComputationalGeometry-6.3
LIBS += -lvtkCommonDataModel-6.3
LIBS += -lvtkFiltersCore-6.3
LIBS += -lvtkFiltersSources-6.3
LIBS += -lvtkFiltersGeneral-6.3
LIBS += -lvtkFiltersGeometry-6.3

LIBS += -L/home/hd/boost_1_54_0/build/install/lib
LIBS += -lboost_thread
LIBS += -lboost_system
LIBS += -lboost_filesystem
LIBS += -lboost_iostreams

HEADERS += \
    fastbilateral.h
