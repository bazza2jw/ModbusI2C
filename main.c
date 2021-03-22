
/*
 * FreeModbus Libary: Win32 Demo Application
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: demo.c,v 1.1 2007/09/12 10:15:56 wolti Exp $
 */

/**********************************************************
*	Linux TCP support.
*	Based on Walter's project.
*	Modified by Steven Guo <gotop167@163.com>
***********************************************************/

/* ----------------------- Standard C Libs includes --------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

// the i2c interface
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
//

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define PROG            "freemodbus"

#define REG_INPUT_START 0
#define REG_INPUT_NREGS 256
#define REG_HOLDING_START 0
#define REG_HOLDING_NREGS 256

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];
static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
static pthread_mutex_t xLock = PTHREAD_MUTEX_INITIALIZER;
static enum ThreadState
{
    STOPPED,
    RUNNING,
    SHUTDOWN
} ePollThreadState;

/* ----------------------- Static functions ---------------------------------*/
static BOOL     bCreatePollingThread( void );
static enum ThreadState eGetPollingThreadState( void );
static void     eSetPollingThreadState( enum ThreadState eNewState );
static void* pvPollingThread( void *pvParameter );

/* I2C interface */
static int i2cFd = -1; // current fd to the bus
static int slaveAddr = 0xFF; // current slave address
static UCHAR i2cBuffer[1024]; // buffer for I2C read/writes
#ifdef RASPBERRY_PI_BUILD
#define I2C_DEF_DEV "/dev/i2c-1"
#else
// USB - I2C adapter
// use udev rules to map across
#define I2C_DEF_DEV "/dev/i2c-ch341"
#endif

int openI2C(int slave);

/* ----------------------- Start implementation -----------------------------*/
int
main( int argc, char *argv[] )
{
    int             iExitCode;
    CHAR           cCh;
    BOOL            bDoExit;


    if( eMBTCPInit( 5002 ) != MB_ENOERR )
    {
        fprintf( stderr, "%s: can't initialize modbus stack!\r\n", PROG );
        iExitCode = EXIT_FAILURE;
    }
    else
    {
        eSetPollingThreadState( STOPPED );
        /* CLI interface. */
        printf(  "Type 'q' for quit or 'h' for help!\r\n"  );
        bDoExit = FALSE;
        do
        {
            printf(  "> "  );
            cCh = getchar(  );
            switch ( cCh )
            {
            case  'q' :
                bDoExit = TRUE;
                break;
            case  'd' :
                eSetPollingThreadState( SHUTDOWN );
                break;
            case  'e' :
                if( bCreatePollingThread(  ) != TRUE )
                {
                    printf(  "Can't start protocol stack! Already running?\r\n"  );
                }
                break;
            case  's' :
                switch ( eGetPollingThreadState(  ) )
                {
                case RUNNING:
                    printf(  "Protocol stack is running.\r\n"  );
                    break;
                case STOPPED:
                    printf(  "Protocol stack is stopped.\r\n"  );
                    break;
                case SHUTDOWN:
                    printf(  "Protocol stack is shuting down.\r\n"  );
                    break;
                }
                break;
            case  'h':
                printf(  "FreeModbus demo application help:\r\n" );
                printf(  "  'd' ... disable protocol stack.\r\n"  );
                printf(  "  'e' ... enabled the protocol stack\r\n"  );
                printf(  "  's' ... show current status\r\n"  );
                printf(  "  'q' ... quit applicationr\r\n"  );
                printf(  "  'h' ... this information\r\n"  );
                printf(  "\r\n"  );
                printf(  "Copyright 2007 Steven Guo <gotop167@163.com>\r\n"  );
                break;
            default:
                if( cCh != '\n' )
                {
                    printf(  "illegal command '%c'!\r\n", cCh );
                }
                break;
            }

            /* eat up everything untill return character. */
            while( cCh != '\n' )
            {
                cCh = getchar(  );
            }
        }
        while( !bDoExit );

        /* Release hardware resources. */
        ( void )eMBClose(  );
        iExitCode = EXIT_SUCCESS;
    }
    return iExitCode;
}

BOOL
bCreatePollingThread( void )
{
    BOOL            bResult;
    pthread_t       xThread;
    if( eGetPollingThreadState(  ) == STOPPED )
    {
        if( pthread_create( &xThread, NULL, pvPollingThread, NULL ) != 0 )
        {
            /* Can't create the polling thread. */
            bResult = FALSE;
        }
        else
        {
            bResult = TRUE;
        }
    }
    else
    {
        bResult = FALSE;
    }

    return bResult;
}

void* pvPollingThread( void *pvParameter )
{
    eSetPollingThreadState( RUNNING );

    if( eMBEnable(  ) == MB_ENOERR )
    {
        do
        {
            if( eMBPoll(  ) != MB_ENOERR )
                break;
        }
        while( eGetPollingThreadState(  ) != SHUTDOWN );
    }

    ( void )eMBDisable(  );

    eSetPollingThreadState( STOPPED );

    return 0;
}

enum ThreadState
eGetPollingThreadState(  )
{
    enum ThreadState eCurState;

    ( void )pthread_mutex_lock( &xLock );
    eCurState = ePollThreadState;
    ( void )pthread_mutex_unlock( &xLock );

    return eCurState;
}

void
eSetPollingThreadState( enum ThreadState eNewState )
{
    ( void )pthread_mutex_lock( &xLock );
    ePollThreadState = eNewState;
    ( void )pthread_mutex_unlock( &xLock );
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    //
    usAddress--;
    switch ( eMode )
    {
    /* Pass current register values to the protocol stack. */
    case MB_REG_READ:
    {
        if((i2cFd > 0) && (usNRegs > 0))
        {
            uint8_t r = (uint8_t)(usAddress & 0xFF);
            if (write(i2cFd, &r, sizeof(r)) == sizeof(r))
            {
                if(read(i2cFd, i2cBuffer, (size_t)usNRegs) == usNRegs)
                {
                    UCHAR *p = i2cBuffer;
                    for(int i = 0; i < usNRegs; i++, p++)
                    {
                        *pucRegBuffer++ = 0; // high byte
                        *pucRegBuffer++ = *p; // low byte
                    }
                    return MB_ENOERR;
                }
            }
        }
    }
    break;
    /* Update current register values with new values from the protocol stack. */
    case MB_REG_WRITE:
    {
        if(usAddress >= 0xFF)
        {
            pucRegBuffer++;
            slaveAddr = *pucRegBuffer;
            if(slaveAddr < 0x7F)
            {
                if(i2cFd > 0) close(i2cFd);
                // when the slave is selected open the i2c bus
                if ((i2cFd = openI2C(slaveAddr)) < 0)
                {
                    return MB_ENOREG;
                }
            }
            else
            {
                // close the i2c connection
                if(i2cFd > 0) close(i2cFd);
                slaveAddr = 0xFF;
                i2cFd = -1;
            }
            return MB_ENOERR;

        }
        else {

            if((i2cFd > 0) && (usNRegs > 0))
            {
                i2cBuffer[0] = (unsigned char)(usAddress & 0xFF);
                UCHAR *p = i2cBuffer;
                p++;
                for(int i = 0; i < usNRegs; i++,p++)
                {
                    pucRegBuffer++; // high byte
                    *p = *pucRegBuffer++; // low byte
                }

                if(write(i2cFd,i2cBuffer,(size_t)(usNRegs+1)) == (usNRegs + 1))
                {
                    return MB_ENOERR;
                }
            }
        }
    }
    break;

    default:
        break;
    }
    return MB_ENOREG;
}


int openI2C(int slave)
{
    int fd = -1;
    if ((fd = open(I2C_DEF_DEV, O_RDWR)) > 0)
    {
        if(ioctl(fd, I2C_SLAVE, slave) != 0)
        {
            close(fd);
            fd = -1;
        }
    }
    return fd;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    // usAddress is the slave address
    if(usAddress < 128)
    {
        usAddress--;
        //
        if(i2cFd < 0)
        {
            // when the slave is selected open the i2c bus
            slaveAddr = usAddress;
            if ((i2cFd = openI2C(slaveAddr)) > 0)
            {
                return MB_ENOERR;
            }
        }
        return MB_ENOREG;
    }
    else
    {
        // close the i2c connection
        if(i2cFd > 0) close(i2cFd);
        slaveAddr = 0xFF;
        i2cFd = -1;
    }
    return MB_ENOERR;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}
