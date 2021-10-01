#ifndef MAVLINK_PARSER_H
#define MAVLINK_PARSER_H

#include "PlotJuggler/messageparser_base.h"

using namespace PJ;


class MAVLink_Parser: public MessageParser
{
public:

  MAVLink_Parser(const std::string& topic_name, PlotDataMapRef& data, bool use_msg_stamp):
    MessageParser(topic_name, data) {}

protected:

  bool parseMessage(const MessageRef msg, double &timestamp) override;
};

class MAVLink_ParserCreator: public MessageParserCreator
{
public:
  MAVLink_ParserCreator() {}

  MessageParserPtr createInstance(const std::string& topic_name, PlotDataMapRef& data) override {
    return std::make_shared<MAVLink_Parser>(topic_name, data);
  }

  const char* name() const override { return "MAVLink"; }
};

#endif // MAVLINK_PARSER_H
