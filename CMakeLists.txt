cmake_minimum_required(VERSION 3.5)
project(ModbusI2C)

include_directories(./freemodbus-v1.5.0/modbus/include)
include_directories(./freemodbus-v1.5.0/modbus)
include_directories(./port)
include_directories(./freemodbus-v1.5.0/modbus/rtu)
include_directories(./freemodbus-v1.5.0/modbus/tcp)
#add_definitions(-DRASPBERRY_PI_BUILD)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(LIB_SOURCES
        freemodbus-v1.5.0/modbus/functions/mbfunccoils.c 
        freemodbus-v1.5.0/modbus/functions/mbfuncdiag.c 
        freemodbus-v1.5.0/modbus/functions/mbfuncdisc.c 
        freemodbus-v1.5.0/modbus/functions/mbfuncholding.c 
        freemodbus-v1.5.0/modbus/functions/mbfuncinput.c 
        freemodbus-v1.5.0/modbus/functions/mbfuncother.c
        freemodbus-v1.5.0/modbus/functions/mbutils.c 
        freemodbus-v1.5.0/modbus/mb.c 
        freemodbus-v1.5.0/modbus/tcp/mbtcp.c 
        main.c 
        port/portevent.c 
        port/portother.c
        port/porttcp.c
)

add_executable(modbusI2c ${LIB_SOURCES})
target_link_libraries(modbusI2c pthread)
install( TARGETS modbusI2c DESTINATION /usr/local/bin)

