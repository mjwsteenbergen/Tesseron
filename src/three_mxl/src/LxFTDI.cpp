#include <threemxl/C3mxl.h>
#include <threemxl/LxFTDI.h>
#include <ros/ros.h>

int main( int argc, const char* argv[] )
{
	
	
	C3mxl *motor = new C3mxl();
	LxSerial *serial_port = new LxFTDI();
	CDxlConfig *config = new CDxlConfig();
	
	serial_port->port_open("i:0x0403:0x6001", LxSerial::RS485_FTDI);
	serial_port->set_speed_int(921600);
	motor->setSerialPort(serial_port);
	motor->setPos(0);
	
	motor->setConfig(config->setID(107));
	motor->init(false);
	motor->setPWM(0.01,true);
	int aInt = motor->getPos ();
	int bInt = motor->getSensorVoltages ();
	int cInt = motor->getState ();
	printf("%i \n%i \n%i \n", aInt, bInt, cInt);
}


