# ModbusI2C
This Freemodbus based MODBUS/TCP server maps to an I2C bus.

Use libmodbus as follows with this server:

#include <stdio.h>
#include <modbus/modbus-tcp.h>
#include <errno.h>
int main()
{
    modbus_t *ctx = modbus_new_tcp("192.168.0.121", 5002); // connect to the server
    modbus_set_debug(ctx, TRUE);
    //
    if (modbus_connect(ctx) == -1) {
         fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
         modbus_free(ctx);
         return -1;
     }
    //
    int rc = modbus_write_register(ctx, 0x100, 60); // set the slave address to 60 and opens the I2C bus - the 0xFF 
    //
    uint16_t dest[2];
    int reg = 2;
    int number_of_byte = 2;
    rc =  modbus_read_registers(ctx, reg, number_of_bytes, dest); // read I2C address, number of bytes - read into uint16_t - copy to uint8_t before use
    //
    printf("%04X %04X\n",dest[0],dest[1]);
    //
    // close the bus
    //
    rc = modbus_write_register(ctx, 0x100, 0xFF); // close the I2C bus
    //
    modbus_close(ctx);
    modbus_free(ctx);
    //
    return 0;
}


Reading and writing to holding register 0x200 is equivalent to a block read / write operation without the device register select.



