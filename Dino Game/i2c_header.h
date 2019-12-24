
#define USE_AND_OR	// To enable AND_OR mask setting for I2C. 
//#include <i2c.h>
#include "plib.h"
#include "tft_master.h"


//=====i2Cparameters========
//datasheet: slave address associated toLSM6DS3 is 110101xb
//set x to 1 by connecting to a supply voltage
#define SLAVE_ADDRESS 0x6B
//gyro output registers 
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27

#define CTRL1_XL 0x10 //Linear acceleration sensor
//[7:4] = 1000 for 1.66kHz Output Data Rate (ODR)

#define CTRL9_XL 0x18
#define WHO_AM_I 0x0f

//accelerometer output registers
#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D

#define GYRO_SCALE 131.0 // lsb/(degrees/second)

float X_GYRO_OFF, Y_GYRO_OFF, Z_GYRO_OFF;

// Wait by executing nops
void i2c_wait(unsigned int cnt)
{
	while(--cnt)
	{
		asm( "nop" );
		asm( "nop" );
	}
}

// Write a number of chars from data specified by num to the specified address
void i2c_write(char address, char *data, int num)
{
    char i2c_header[2];
    i2c_header[0] = (SLAVE_ADDRESS<<1) | 0;	//device address & WR
	i2c_header[1] = address;            //register address

    StartI2C1();	//Send the Start Bit
	IdleI2C1();		//Wait to complete

    int i;
	for(i = 0; i < num + 2; i++)
	{
        if(i < 2)
            MasterWriteI2C1( i2c_header[i] );
        else
            MasterWriteI2C1( data[i - 2] );
		IdleI2C1();		//Wait to complete

		//ACKSTAT is 0 when slave acknowledge, 
		//if 1 then slave has not acknowledge the data.
		if( I2C1STATbits.ACKSTAT )
			break;
	}
    
    StopI2C1();	//Send the Stop condition
	IdleI2C1();	//Wait to complete
}

// Read a char from the register specified by address
char i2c_read(char address)
{
    char i2c_header[2];
    i2c_header[0] = ( (SLAVE_ADDRESS<<1) | 0 );	//device address & WR
	i2c_header[1] = address;                //register address

    StartI2C1();	//Send the Start Bit
	IdleI2C1();		//Wait to complete

    int i;
	for(i = 0; i < 2; i++)
	{
        MasterWriteI2C1( i2c_header[i] );
		IdleI2C1();		//Wait to complete

		//ACKSTAT is 0 when slave acknowledge, 
		//if 1 then slave has not acknowledge the data.
		if( I2C1STATbits.ACKSTAT )
        {
			break;
        }
	}
    
    //now send a start sequence again
	RestartI2C1();	//Send the Restart condition
	i2c_wait(10);
	//wait for this bit to go back to zero
	IdleI2C1();	//Wait to complete

	MasterWriteI2C1( (SLAVE_ADDRESS<<1) | 1 ); //transmit read command
	IdleI2C1();		//Wait to complete

	// read some bytes back
//	MastergetsI2C1(num, dataBuf, 20);
    char data = MasterReadI2C1();
	
	IdleI2C1();	//Wait to complete
    
    StopI2C1();	//Send the Stop condition
	IdleI2C1();	//Wait to complete
    
    return data;
}

// Read three-axis accelerometer and three-axis gyroscope from MPU 6050
// Return values in array pointed to by values
void readImuValues(float* values)
{
    int xAccelH = (int) i2c_read(OUTX_H_XL);
    int xAccelL = (int) i2c_read( OUTX_L_XL);
    int yAccelH = (int) i2c_read( OUTY_H_XL);
    int yAccelL = (int) i2c_read( OUTY_L_XL);
    int zAccelH = (int) i2c_read( OUTZ_H_XL);
    int zAccelL = (int) i2c_read( OUTZ_L_XL);
    
    //int whoAMI = (int) i2c_read(WHO_AM_I);

    values[0] = (float)((xAccelH << 8) + xAccelL);
    values[1] = (float)((yAccelH << 8) + yAccelL);
    values[2] = (float)((zAccelH << 8) + zAccelL);
    //values[3] = 0;
}


void readPartValues(float* values)
{   
    int whoAMI = (int) i2c_read(WHO_AM_I);
    values[0] = (float) whoAMI;
}
