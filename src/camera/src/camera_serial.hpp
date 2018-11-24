/***********************************************************************************
 *  StereoCamera
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * ------------------------------
 *  boredman@BoredomProjects.net
 * ------------------------------
 *
 ***********************************************************************************/
#ifndef CAMERA_SERIAL_HPP
#define CAMERA_SERIAL_HPP

#include <opencv2/opencv.hpp>
//#include <omp.h>

#include <sstream>
#include <iostream>
#include <string>
#include <iostream>
#include <cstdio>
#include <stdio.h>
#include <pthread.h>

#include <time.h>
// OS Specific sleep
//#include <unistd.h>

#include "serial/serial.h"

/*
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
*/


using namespace std;
using namespace cv;



namespace serial
{
  class Camera : public Serial
  {
    using Serial::Serial;

    public:

    uint16_t camline;
    uint8_t camregs[12];

    static const unsigned int MAX_IMAGE_HEIGHT = 480;
    static const unsigned int MAX_IMAGE_WIDTH = 752;
    static const unsigned int LINE_LENGTH = MAX_IMAGE_WIDTH * 2 + sizeof(camline) + sizeof(camregs);

    struct camconfig
    {
      int exposure_us;	// uint32_t breaks nh.param()
      int analogGain;
      int digitalGain;
      int n_lines;
      int cpf;
      int aec_enable;
      int agc_enable;
      int cmp_enable;
    };

    camconfig config;

    size_t Configure(camconfig& cf)
    {
      // save new state for later
      memcpy(&config, &cf, sizeof(camconfig));
      // send state to device
      return write((uint8_t*)&cf, sizeof(camconfig));
    }

    int RecvLine(Mat& imgR, Mat& imgL)
    {
      uint8_t lineData[LINE_LENGTH];

      // read one row of image data
      if (read(lineData, LINE_LENGTH) != LINE_LENGTH)
        return -1;

      // get row number
      uint16_t y = ((uint16_t)(lineData[0] & 0x01) << 8) | lineData[1];
      y -= (MAX_IMAGE_HEIGHT - config.n_lines) / 2;
      if (y >= config.n_lines)
        return -2;
      camline = y;

      // copy regs
      for (int i = 0; i < sizeof(camregs); i++)
        camregs[i] = lineData[i + sizeof(camline)];

      // copy line data into left and right images
      uint8_t* p = imgR.ptr<uint8_t>(y);
      for (int x = 0; x < MAX_IMAGE_WIDTH; x++)
        p[x] = lineData[x + sizeof(camline) + sizeof(camregs)];

      p = imgL.ptr<uint8_t>(y);
      for (int x = 0; x < MAX_IMAGE_WIDTH; x++)
        p[x] = lineData[x + sizeof(camline) + sizeof(camregs) + MAX_IMAGE_WIDTH];

      // check for end of frame
      if (y == config.n_lines - 1)
        return 0;
      else
        return 1;
    }

  };
}


#endif
