#ifndef I2C_H
#define I2_C

unsigned char mmyi2c_read(char slave_address, char reg_address)
{
    char data;
    i2c_start();
    #asm("wdr")
    i2c_write(slave_address );
    #asm("wdr")
    i2c_write(reg_address);
    i2c_start();
    #asm("wdr")
    i2c_write(slave_address +1);
    data=i2c_read(0);
    #asm("wdr")
    i2c_stop();
    #asm("wdr")
    return data;
}
// Read a 16-bit register
unsigned int mmyi2c_read16(char slave_address, char reg) {
  unsigned int value;
  i2c_start();
  i2c_write(slave_address);
  i2c_write( reg );
  i2c_start();
  i2c_write(slave_address +1);
  value  = i2c_read(1) << 8;
  value |= i2c_read(0);
  i2c_stop();
  return value;
}

void mmyi2c_write(char slave_address, char reg_address, char data)
{
    i2c_start();
    i2c_write(slave_address);
    i2c_write(reg_address);
    i2c_write(data);
    i2c_stop();  
} 

void srf02ChangeAddr(char initAddr,char finalAddr) {
    mmyi2c_write(initAddr,0,0xA0);
    mmyi2c_write(initAddr,0,0xAA);
    mmyi2c_write(initAddr,0,0xA5);
    mmyi2c_write(initAddr,0,finalAddr);
}


#endif