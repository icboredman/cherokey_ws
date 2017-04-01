#include <RH_Serial.h>

class Base_Serial : public RH_Serial
{
public:
  enum MsgID {
    ID_ODOM = 20,
    ID_POWER = 30,
    ID_DRIVE = 40,
  };
  struct Power {
    uint16_t battery_miV;
    int16_t  battery_miA;
    uint16_t vsupply_miV;
    uint8_t  charger_state;
    uint8_t  bat_percentage;
  } power;
  struct Odom {
    float theta;
    float dx_mm;
    float dth;
    uint16_t dt_ms;
  } odom;
  struct Drive {
    int16_t speed_mm_s;
    int16_t turn_mrad_s;
  } drive;

  Base_Serial(HardwareSerial& serial) : RH_Serial(serial) {}

  bool init()
  {
    if( ! RH_Serial::init() )
      return false;
    //flush Rx to avoid problems with incorrect first headerId
    while( available() )
    {
      uint8_t len = sizeof(buf);
      if( ! recv(buf, &len) )
        return false;
    }
    return true;
  }

  bool sendOdom()
  {
    setHeaderId(ID_ODOM);
    return send((uint8_t*)&odom, sizeof(odom));
  }

  bool sendPower()
  {
    setHeaderId(ID_POWER);
    return send((uint8_t*)&power, sizeof(power));
  }

  bool sendDrive()
  {
    setHeaderId(ID_DRIVE);
    return send((uint8_t*)&drive, sizeof(drive));
  }

  bool recvDrive()
  {
    uint8_t len = sizeof(buf);
    if( !available() || headerId() != ID_DRIVE )
      return false;
    if( !recv(buf, &len) )
      return false;
    if( len != sizeof(drive) )
      return false;
    memcpy((uint8_t*)&drive, buf, sizeof(drive));
    return true;
  }

  bool recvPower()
  {
    uint8_t len = sizeof(buf);
    if( !available() || headerId() != ID_POWER )
      return false;
    if( !recv(buf, &len) )
      return false;
    if( len != sizeof(power) )
      return false;
    memcpy((uint8_t*)&power, buf, sizeof(power));
    return true;
  }

  bool recvOdom()
  {
    uint8_t len = sizeof(buf);
    if( !available() || headerId() != ID_ODOM )
      return false;
    if( !recv(buf, &len) )
      return false;
    if( len != sizeof(odom) )
      return false;
    memcpy((uint8_t*)&odom, buf, sizeof(odom));
    return true;
  }

protected:
  uint8_t buf[RH_SERIAL_MAX_MESSAGE_LEN];

};

