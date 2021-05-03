/*!
 *
 *  Author:  B. J. Hill
 *
 */

#include "i2cinterface.h"
#include <wx/socket.h>


/*!
  * \brief MRL::I2cInterfaceBase::~I2cInterfaceBase
  */
 MRL::I2cInterfaceBase::~I2cInterfaceBase()
{
}

 /*!
 * \brief MRL::I2cInterfaceRemote::~I2cInterfaceRemote
 */
MRL::I2cInterfaceRemote::~I2cInterfaceRemote()
{
     end();
     if(_ctx)
     {
         modbus_free(_ctx);
     }
 }


 /*!
  * \brief MRL::I2cInterfaceRemote::start
  * \param slave
  * \return
  */
 bool MRL::I2cInterfaceRemote::start(int slave)
 {
     if(!_ctx)
     {
         const char *s = _dev.c_str();
         wxIPV4address addr;
         addr.Hostname(s);
         std::string a  = addr.IPAddress().ToStdString();
        _ctx = modbus_new_tcp(a.c_str(),_port);
     }
     //
    if(modbus_get_socket(_ctx) < 1)
    {
        if(modbus_connect(_ctx) == -1 )
        {
            modbus_free(_ctx);
            _ctx = nullptr;
            return false;
        }
    }
    return modbus_write_register(_ctx, 0x100, slave) == 1;
 }
 /*!
  * \brief close
  */
 void MRL::I2cInterfaceRemote::end()
 {
     if(_ctx)
     {
         modbus_write_register(_ctx, 0x100, 0xFF);
         modbus_close(_ctx);
     }
 }
 /*!
  * \brief write
  * \param device
  * \param reg
  * \param buf
  * \param len
  */
 bool MRL::I2cInterfaceRemote::writeBlock( int reg, const uint8_t * buf, int len)
 {
     // build the write record
     if(_ctx)
     {
         uint16_t b[len];
         for(int i = 0; i < len; i++) b[i] = buf[i];
         return  modbus_write_registers(_ctx, reg, len, b) != -1;
     }
     return false;
 }
 /*!
  * \brief read
  * \param device
  * \param address
  * \param buf
  * \param len
  * \return
  */
 bool  MRL::I2cInterfaceRemote::readBlock( int address, uint8_t *buf, int len)
 {
     if(_ctx)
     {
         uint16_t b[len];
         if( modbus_read_registers(_ctx, address, len, b) == len)
         {
             for(int i = 0; i < len; i++)
             {
                 *buf++ = (uint8_t)b[i];
             }
             return true;
         }
     }
     return false;
 }



 /*!
  * \brief blockRead
  * \param buf
  * \param len
  * \return
  */
 bool MRL::I2cInterfaceRemote::blockRead(uint8_t *buf, int len)
 {
     if(_ctx)
     {
         uint16_t b[len];
         if( modbus_read_registers(_ctx, 0x200, len, b) == len)
         {
             for(int i = 0; i < len; i++)
             {
                 *buf++ = (uint8_t)b[i];
             }
             return true;
         }
     }
     return false;
 }

 /*!
  * \brief blockWrite
  * \param buf
  * \param len
  * \return
  */
 bool MRL::I2cInterfaceRemote::blockWrite(const uint8_t *buf, int len)
 {
     if(_ctx)
     {
         uint16_t b[len];
         for(int i = 0; i < len; i++) b[i] = buf[i];
         return  modbus_write_registers(_ctx, 0x200, len, b) != -1;
     }
     return false;
 }


