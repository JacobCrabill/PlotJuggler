#include "dataload_tlog.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QWidget>
#include <QSettings>
#include <QProgressDialog>
#include <QMainWindow>
#include "PlotJuggler/datastreamer_base.h"
#include "selectlistdialog.h"

uint64_t readUint64BigEndian(uint8_t* data)
{
  uint64_t out = ((uint64_t)data[0] << 56) + ((uint64_t)data[1] << 48) + ((uint64_t)data[2] << 40) +
                 ((uint64_t)data[3] << 32) + ((uint64_t)data[4] << 24) + ((uint64_t)data[5] << 16) +
                 ((uint64_t)data[6] << 8) + data[7];

  return out;
}

DataLoadTLog::DataLoadTLog() : _main_win(nullptr)
{
  for (QWidget* widget : qApp->topLevelWidgets())
  {
    if (widget->inherits("QMainWindow"))
    {
      _main_win = widget;
      break;
    }
  }
}

const std::vector<const char*>& DataLoadTLog::compatibleFileExtensions() const
{
  static std::vector<const char*> extensions = { "tlog" };
  return extensions;
}

bool DataLoadTLog::readDataFromFile(FileLoadInfo* fileload_info, PlotDataMapRef& plot_data)
{
  const auto& filename = fileload_info->filename;

  QFile file(filename);

  if (!file.open(QIODevice::ReadOnly))
  {
    throw std::runtime_error("TLog: Failed to open file");
  }

  // Create the MAVLink parser object
  MAVLink_ParserCreator creator;
  // std::shared_ptr<MessageParserCreator> parser_creator;
  // QString protocol = "MAVLink";
  // if (!availableParsers()->contains(protocol))
  // {
  //   throw std::runtime_error("TLog: MAVLink parser not found");
  //   return false;
  // }
  // parser_creator = availableParsers()->at( "MAVLink" );
  // _parser = parser_creator->createInstance({}, plot_data);
  _parser = creator.createInstance({}, plot_data);

  // Read the file into memory
  QByteArray file_array = file.readAll();
  uint8_t* data = reinterpret_cast<uint8_t*>(file_array.data());
  const size_t nbytes = file_array.size();

  // Useful constants
  const uint8_t TIME_LEN = 8;
  const uint8_t CRC_LEN = 2;
  const uint8_t HEADER_LEN_V1 = 6;
  const uint8_t HEADER_LEN_V2 = 10;
  const uint8_t OVERHEAD_V1 = HEADER_LEN_V1 + CRC_LEN;
  const uint8_t OVERHEAD_V2 = HEADER_LEN_V2 + CRC_LEN;

  const uint8_t MAVLINK_MAGIC_V1 = 0xfe;
  const uint8_t MAVLINK_MAGIC_V2 = 0xfd;

  // ... loop through file; do:
  // [8 bytes: timestamp][mavlink message]
  // get timestamp, msg
  // _parser->parseMessage(msg, timestamp);

  size_t ofs = 0;
  double first_timestamp = -1;
  while (ofs + HEADER_LEN_V1 + TIME_LEN < nbytes)
  {
    // Step 1: Read the timestamp; convert microseconds to seconds
    double timestamp = 1e-6 * readUint64BigEndian(&data[ofs]);

    const size_t mofs = ofs + TIME_LEN;

    // Step 2: Begin parsing the bytes assumed to be a MAVLink packet
    uint8_t stx = data[mofs];
    uint8_t len = data[mofs + 1];
    uint16_t mlen = len;
    if (stx == MAVLINK_MAGIC_V1)
    {
      mlen = OVERHEAD_V1 + len;
    }
    else if (stx == MAVLINK_MAGIC_V2)
    {
      mlen = OVERHEAD_V2 + len;
    }
    else
    {
      // This is not the byte you're looking for (bad packet?) - move along
      ofs++;
      continue;
    }

    if (mofs + mlen > nbytes)
    {
      // malformed / partial packet at end of log - quit
      break;
    }

    // Loosely sanity-check the timestamp value (seconds since Unix epoch)
    if (first_timestamp < 0)
    {
      first_timestamp = timestamp;
    }
    else if ((timestamp - first_timestamp) > 1e6)
    {
      // Timestamps should never be this large - Bad packet; try to resync
      ofs++;
      continue;
    }

    // Try to parse the message
    MessageRef msg(&data[mofs], mlen);
    if (_parser->parseMessage(msg, timestamp))
    {
      // successfully parsed a message
      ofs += TIME_LEN + mlen;
    }
    else
    {
      // Bad message - try to resync
      ofs++;
      continue;
    }
  }

  return true;
}

DataLoadTLog::~DataLoadTLog()
{
}

bool DataLoadTLog::xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const
{
  return true;
}

bool DataLoadTLog::xmlLoadState(const QDomElement&)
{
  return true;
}
