#include "nlohmann_parsers.h"

#include "PlotJuggler/fmt/format.h"

bool MAVLink_Parser::parseMessage(const MessageRef msg,
                                 double& timestamp)
{
  // TODO
  // usage: msg.data(), msg.size() -- pointer to packet start; size in bytes
  
  /* ==== Definitions:
   */
   typedef struct __mavlink_field_info {
     const char *name;                 // name of this field
     const char *print_format;         // printing format hint, or NULL
     mavlink_message_type_t type;      // type of this field
     unsigned int array_length;        // if non-zero, field is an array
     unsigned int wire_offset;         // offset of each field in the payload
     unsigned int structure_offset;    // offset in a C structure
   } mavlink_field_info_t;
   
   // note that in this structure the order of fields is the order
   // in the XML file, not necessary the wire order
   typedef struct __mavlink_message_info {
   	uint32_t msgid;               // message ID
   	const char *name;             // name of the message
   	unsigned num_fields;          // how many fields in this message
   	mavlink_field_info_t fields[MAVLINK_MAX_FIELDS];       // field information
   } mavlink_message_info_t;
   
   /* ==== Plan of Attack:
   *
   * + Use this as a C++ wrapper around mavlink_parse_char()
   * + Pass in timestamp from the class doing the parsing:
   *    - If from tlog file: The preceding 8 byte big-endian UTC timestamp
   *    - If from UDP/TCP stream: The current system time
   * + Decode the message info:
   *    - mavlink_message_info_t info = mavlink_get_message_info_by_id(msgid);
   *    - [for f in num_fields] mavlink_message_info_t finfo = info.fields[f]
   *    - if finfo.array_length == 1:
   *       - series_data.pushBack({info.name + finfo.name, msg[finfo.wire_offset]});
   *    - else:
   *       - <same, but loop over every element and append %i suffix to the name>
   *
   * ==== For TLogs:
   * Create a DataLoadTLog plugin that loads a TLog and handles the parsing
   * of the log at the level of [8-byte timestamp][mavlink message], where
   * this parser handles the [mavlink message] portion
   *
   * ==== For live streams of UDP/TCP Data:
   * Once this class is complete, it *should* just show up as a parser option...?
   */

  std::string prefix = "tlog.global_position_int.lat";
  double timestamp = 0;
  double numeric_value = 0;
  auto plot_data = &(getSeries(prefix));
  plot_data->pushBack( {timestamp, numeric_value} );

  return true;
}
