#-------------------------------------------------
#
# Project created by QtCreator 2012-02-16T11:13:42
#
#-------------------------------------------------

#QT       -= gui

TARGET = ePuckcpp
TEMPLATE = lib

DEFINES += EPUCKCPP_LIBRARY

SOURCES += \
    epuck.cpp \
    serialcomm.cpp

HEADERS +=\
    epuck.h \
    ePuck_global.h \
    serialcomm.h

symbian {
    MMP_RULES += EXPORTUNFROZEN
    TARGET.UID3 = 0xE56B18BA
    TARGET.CAPABILITY = 
    TARGET.EPOCALLOWDLLDATA = 1
    addFiles.sources = ePuckcpp.dll
    addFiles.path = !:/sys/bin
    DEPLOYMENT += addFiles
}

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}

OTHER_FILES += \
    serialport/src.pro.user \
    README.md \
    LICENSE.txt
