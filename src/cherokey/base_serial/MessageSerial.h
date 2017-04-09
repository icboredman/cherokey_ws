// =============================================================================
// This code is based on PacketSerial <https://github.com/bakercp/PacketSerial>
// by Christopher Baker <http://christopherbaker.net>
// It was modified by adding helper class Message to generalize message format
// and CRC checking based on FastCRC library.
//
// Copyright (c) 2017 boredman <http://BoredomProjects.net>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// =============================================================================


#pragma once


#include "Encoding/COBS.h"
#include "Encoding/SLIP.h"

#include "FastCRC.h"


#if defined(ARDUINO)
  typedef Stream HSerial;
#elif defined(__unix__)
  typedef serial::Serial HSerial;
#endif



template<typename EncoderType, uint8_t PacketMarker = 0, size_t BufferSize = 256>
class MessageSerial_
{
public:

  enum {
    MAX_MSGS = 20,
    MAX_MSG_SIZE = 250
  };

  MessageSerial_(HSerial& serial)
  {
    _serial = &serial;
    receiveBufferIndex = 0;
    for( uint8_t id=0; id<MAX_MSGS; id++ )
    {
      msg[id].dt = NULL;
      msg[id].sz = 0;
    }
  }

  ~MessageSerial_()
  {
  }


  bool registerMsg(uint8_t id, uint8_t *data, uint8_t size, bool *flag)
  {
    if( id >= MAX_MSGS )
      return false;
    msg[id].dt = data;
    msg[id].sz = size;
    msg[id].fl = flag;
    return true;
  }


  void update()
  {
    if (_serial == 0) return;

    while (_serial->available() > 0)
    {
      uint8_t data;
#if defined(ARDUINO)
      data = _serial->read();
#elif defined(__unix__)
      _serial->read(&data, 1);
#endif
      if (data == PacketMarker)
      {
        uint8_t *decodeBuffer = new uint8_t[receiveBufferIndex];

        uint8_t numDecoded = EncoderType::decode(receiveBuffer,
                                                 receiveBufferIndex,
                                                 decodeBuffer);
        receiveBufferIndex = 0;

        // check CRC
        uint16_t crc = CRC16.ccitt(decodeBuffer, numDecoded-2);
        if ( decodeBuffer[numDecoded-2] == (crc >> 8)  &&
             decodeBuffer[numDecoded-1] == (crc & 0xff) )
        {
          uint8_t id = decodeBuffer[0];
          if( id < MAX_MSGS                            &&
              msg[id].dt != NULL                       &&
              (msg[id].sz == numDecoded-3 || id == 1)  &&
              *(msg[id].fl) == false                      )
          {
            memcpy(msg[id].dt, &decodeBuffer[1], numDecoded-3);
            if( id == 1 )
            { // if this is text message, place null-terminator at the end
              msg[id].dt[numDecoded-3] = '\0';
            }
            *(msg[id].fl) = true;
          }
        }
        delete[] decodeBuffer;
      }
      else
      {
        if ((receiveBufferIndex + 1) < BufferSize)
        {
           receiveBuffer[receiveBufferIndex++] = data;
        }
        else
        {
          // Error, buffer overflow if we write.
        }
      }
    }
  }


  bool send(uint8_t id)
  {
    uint8_t* msg_data = msg[id].dt;
    uint8_t  msg_size = id==1 ? strlen((char*)msg[id].dt) : msg[id].sz;

    if(_serial == 0 || msg_data == 0 || msg_size == 0)
      return false;

    uint8_t *buffer = new uint8_t[msg_size+3];  // id + data + crc
    buffer[0] = id;
    memcpy(&buffer[1], msg_data, msg_size);

    // CRC including id
    uint16_t crc = CRC16.ccitt(buffer, msg_size+1);
    buffer[msg_size+1] = crc >> 8;   // high byte
    buffer[msg_size+2] = crc & 0xff; // low byte

    uint8_t *encodeBuffer = new uint8_t[EncoderType::getEncodedBufferSize(msg_size+4)]; // + marker
    size_t numEncoded = EncoderType::encode(buffer, msg_size+3, encodeBuffer);

    encodeBuffer[numEncoded] = PacketMarker;
    size_t n = _serial->write(encodeBuffer, numEncoded+1);

    delete[] encodeBuffer;
    delete[] buffer;

    return (n == numEncoded+1);
  }


protected:

  typedef struct {
    uint8_t* dt;
    uint8_t sz;
    bool* fl;
  } tMsg;

  tMsg msg[MAX_MSGS];

  MessageSerial_(const MessageSerial_&);
  MessageSerial_& operator = (const MessageSerial_&);

  uint8_t receiveBuffer[BufferSize];
  size_t  receiveBufferIndex;

  HSerial* _serial;

  FastCRC16 CRC16;
};


typedef MessageSerial_<COBS> MessageSerial;
typedef MessageSerial_<COBS> COBSMessageSerial;
typedef MessageSerial_<SLIP, SLIP::END> SLIPMessageSerial;




//*********************************************************************
template <typename MSG_TYPE, uint8_t msg_id>
class Message
{
public:

  MSG_TYPE data;

  Message(MessageSerial& serial)
  {
    _serial = &serial;
    _serial->registerMsg(msg_id, (uint8_t*)&data, sizeof(data), &rx_full);
    rx_full = false;
  }

  bool send()
  {
    return _serial->send(msg_id);
  }

  bool available()
  {
    return rx_full;
  }

  void ready()
  {
    rx_full = false;
  }

protected:
  MessageSerial* _serial;
  bool rx_full;
};


//*********************************************************************


