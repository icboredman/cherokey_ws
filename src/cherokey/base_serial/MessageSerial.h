#include <PacketSerial.h>

// declaration of MessageSerial
class MessageSerial : public PacketSerial
{
public:
  MessageSerial();
#if defined(ARDUINO)
  void begin(unsigned long baud, size_t port=0);
  void begin(Stream* serial);
#elif defined(__unix__)
  void begin(serial::Serial* serial);
#endif
  bool registerMsg(uint8_t id, uint8_t *data, uint8_t size, bool *flag);
  void send(uint8_t id);
  void onPacket(const uint8_t* buffer, size_t size);
  enum {
    MAX_MSGS = 20,
    MAX_MSG_SIZE = 250
  };
protected:
  typedef struct {
    uint8_t* dt;
    uint8_t sz;
    bool* fl;
  } tMsg;
  tMsg msgs[MAX_MSGS];
};


//*********************************************************************
template <typename MSG_TYPE, uint8_t msg_id> class Message
{
public:
  MSG_TYPE data;
  Message(MessageSerial& serial)
  {
    _serial = &serial;
    _serial->registerMsg(msg_id, (uint8_t*)&data, sizeof(data), &rx_full);
    rx_full = false;
  }

  void send()
  {
    _serial->send(msg_id);
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
MessageSerial::MessageSerial()
{
  for( uint8_t id=0; id<MAX_MSGS; id++ )
  {
    msgs[id].dt = NULL;
    msgs[id].sz = 0;
  }
  PacketSerial();
}

#if defined(ARDUINO)
void MessageSerial::begin(unsigned long baud, size_t port)
{
  setPacketHandler((PacketHandlerFunction)&MessageSerial::onPacket);
  PacketSerial::begin(baud, port);
}
void MessageSerial::begin(Stream* serial)
{
  setPacketHandler((PacketHandlerFunction)&MessageSerial::onPacket);
  PacketSerial::begin(serial);
}
#elif defined(__unix__)
void MessageSerial::begin(serial::Serial* serial)
{
  setPacketHandler((PacketHandlerFunction)&MessageSerial::onPacket);
  PacketSerial::begin(serial);
}
#endif

bool MessageSerial::registerMsg(uint8_t id, uint8_t *data, uint8_t size, bool *flag)
{
  if( id >= MAX_MSGS )
    return false;
  msgs[id].dt = data;
  msgs[id].sz = size;
  msgs[id].fl = flag;
  return true;
}

void MessageSerial::send(uint8_t id)
{
  uint8_t buffer[msgs[id].sz+1];
  buffer[0] = id;
  memcpy(&buffer[1], msgs[id].dt, msgs[id].sz);
  PacketSerial::send(buffer, sizeof(buffer));
}

void MessageSerial::onPacket(const uint8_t* buffer, size_t size)
{
  // check CRC ...
  uint8_t id = buffer[0];
  if( id >= MAX_MSGS      ||
      msgs[id].dt == NULL ||
      msgs[id].sz == 0    ||
      *(msgs[id].fl) == false )
    return;
  memcpy(msgs[id].dt, &buffer[1], size-1);
  *(msgs[id].fl) = true;
}


