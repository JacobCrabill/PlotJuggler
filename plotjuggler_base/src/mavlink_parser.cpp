#include "PlotJuggler/Parsers/mavlink_parser.h"

#ifdef __GNUG__
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif

#define MAVLINK_USE_MESSAGE_INFO
#include "volansi/mavlink.h"
#include "mavlink_helpers.h"
#include "mavlink_types.h"
#include "mavlink_get_info.h"

uint16_t readUint16(uint8_t* buf)
{
    return uint16_t(buf[0] & 0xff) | uint16_t((buf[1] & 0xff) << 8);
}

int16_t readInt16(uint8_t* buf)
{
    return int16_t(buf[0] & 0xff) | int16_t((buf[1] & 0xff) << 8);
}

uint32_t readUint32(uint8_t* buf)
{
    return uint32_t(buf[0] & 0xff) |
           uint32_t(buf[1] & 0xff) << 8 |
           uint32_t(buf[2] & 0xff) << 16 |
           uint32_t(buf[3] & 0xff) << 24;
}

int32_t readInt32(uint8_t* buf)
{
    return int32_t(buf[0] & 0xff) |
           int32_t(buf[1] & 0xff) << 8 |
           int32_t(buf[2] & 0xff) << 16 |
           int32_t(buf[3] & 0xff) << 24;
}

uint64_t readUint64(uint8_t* buf)
{
    return uint64_t(buf[0] & 0xff) |
           uint64_t(buf[1] & 0xff) << 8 |
           uint64_t(buf[2] & 0xff) << 16 |
           uint64_t(buf[3] & 0xff) << 24 |
           uint64_t(buf[4] & 0xff) << 32 |
           uint64_t(buf[5] & 0xff) << 40 |
           uint64_t(buf[6] & 0xff) << 48 |
           uint64_t(buf[7] & 0xff) << 56;
}

int64_t readInt64(uint8_t* buf)
{
    return int64_t(buf[0] & 0xff) |
           int64_t(buf[1] & 0xff) << 8 |
           int64_t(buf[2] & 0xff) << 16 |
           int64_t(buf[3] & 0xff) << 24 |
           int64_t(buf[4] & 0xff) << 32 |
           int64_t(buf[5] & 0xff) << 40 |
           int64_t(buf[6] & 0xff) << 48 |
           int64_t(buf[7] & 0xff) << 56;
}

bool MAVLink_Parser::parseMessage(const MessageRef msg,
                                  double& timestamp)
{
  bool ret = false;
  mavlink_message_t mav;
  mavlink_status_t stat;
  for (int i = 0; i < msg.size(); i++)
  {
    uint8_t c = msg.data()[i];
    if (mavlink_parse_char(0, c, &mav, &stat))
    {
      const mavlink_message_info_t* minfo = mavlink_get_message_info(&mav);

      if (!minfo) continue;

      std::string prefix = "tlog." + std::string(minfo->name);
      uint8_t num_fields = minfo->num_fields;

      for (int j = 0; j < num_fields; j++)
      {
        mavlink_field_info_t finfo = minfo->fields[j];
        uint32_t idx = finfo.wire_offset; // offset, in bytes, within msg.payload64
        double numeric_value = 0;
        uint8_t* payload = reinterpret_cast<uint8_t*>(mav.payload64);
        switch (finfo.type)
        {
            case MAVLINK_TYPE_CHAR:
                numeric_value = static_cast<char>(payload[idx]);
                break;

            case MAVLINK_TYPE_UINT8_T:
                numeric_value = payload[idx];
                break;

            case MAVLINK_TYPE_INT8_T:
                numeric_value = (int8_t)payload[idx];
                break;

            case MAVLINK_TYPE_UINT16_T:
                numeric_value = readUint16(&payload[idx]);
                break;

            case MAVLINK_TYPE_INT16_T:
                numeric_value = readInt16(&payload[idx]);
                break;

            case MAVLINK_TYPE_UINT32_T:
                numeric_value = readUint16(&payload[idx]);
                break;

            case MAVLINK_TYPE_INT32_T:
                numeric_value = readInt32(&payload[idx]);
                break;

            case MAVLINK_TYPE_UINT64_T:
                numeric_value = readUint64(&payload[idx]);
                break;

            case MAVLINK_TYPE_INT64_T:
                numeric_value = readInt64(&payload[idx]);
                break;

            case MAVLINK_TYPE_FLOAT:
                numeric_value = *(float*)(&payload[idx]);
                break;

            case MAVLINK_TYPE_DOUBLE:
                numeric_value = *(double*)(&payload[idx]);
                break;

            default:
                numeric_value = NAN;
                break;
        }

        ret = true;
        std::string field_name = prefix + "." + std::string(finfo.name);
        auto plot_data = &(getSeries(field_name));
        plot_data->pushBack( {timestamp, numeric_value} );

      }

    }
  }

  return ret;
}
