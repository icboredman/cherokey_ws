
#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#include <unistd.h>

#include "serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

void my_sleep(unsigned long milliseconds) {
  usleep(milliseconds*1000); // 100 ms
}

void enumerate_ports()
{
  vector<serial::PortInfo> devices_found = serial::list_ports();

  vector<serial::PortInfo>::iterator iter = devices_found.begin();

  while( iter != devices_found.end() )
  {
    serial::PortInfo device = *iter++;

    printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
            device.hardware_id.c_str() );
  }
}


int main(int argc, char **argv)
{

//  string port(argv[1]);

  enumerate_ports();
  return 0;
}
