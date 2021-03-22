TEMPLATE = app
CONFIG += console c++11 thread
CONFIG -= app_bundle
CONFIG -= qt
INCLUDEPATH += ./freemodbus-v1.5.0/modbus/include
INCLUDEPATH += ./freemodbus-v1.5.0/modbus
INCLUDEPATH += ./port
INCLUDEPATH += ./freemodbus-v1.5.0/modbus/rtu
INCLUDEPATH += ./freemodbus-v1.5.0/modbus/tcp
DEFINES += RASPBERRY_PI_BUILD


SOURCES += \
        freemodbus-v1.5.0/modbus/functions/mbfunccoils.c \
        freemodbus-v1.5.0/modbus/functions/mbfuncdiag.c \
        freemodbus-v1.5.0/modbus/functions/mbfuncdisc.c \
        freemodbus-v1.5.0/modbus/functions/mbfuncholding.c \
        freemodbus-v1.5.0/modbus/functions/mbfuncinput.c \
        freemodbus-v1.5.0/modbus/functions/mbfuncother.c \
        freemodbus-v1.5.0/modbus/functions/mbutils.c \
        freemodbus-v1.5.0/modbus/mb.c \
        freemodbus-v1.5.0/modbus/tcp/mbtcp.c \
        main.c \
        port/portevent.c \
        port/portother.c \
        port/porttcp.c

HEADERS += \
    freemodbus-v1.5.0/modbus/include/mb.h \
    freemodbus-v1.5.0/modbus/include/mbconfig.h \
    freemodbus-v1.5.0/modbus/include/mbframe.h \
    freemodbus-v1.5.0/modbus/include/mbfunc.h \
    freemodbus-v1.5.0/modbus/include/mbport.h \
    freemodbus-v1.5.0/modbus/include/mbproto.h \
    freemodbus-v1.5.0/modbus/include/mbutils.h \
    freemodbus-v1.5.0/modbus/rtu/mbcrc.h \
    freemodbus-v1.5.0/modbus/rtu/mbrtu.h \
    freemodbus-v1.5.0/modbus/tcp/mbtcp.h \
    port/port.h
