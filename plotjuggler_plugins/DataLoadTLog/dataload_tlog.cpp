#include "dataload_tlog.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QWidget>
#include <QSettings>
#include <QProgressDialog>
#include <QMainWindow>
#include "selectlistdialog.h"
#include "mavlink_parser.h"

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
  QByteArray file_array = file.readAll();
  TLogParser::DataStream datastream( file_array.data(), file_array.size() );

  TLogParser parser(datastream);

  std::shared_ptr<MessageParserCreator> parser_creator;
  parser_creator = availableParsers()->at( "MAVLink" );
  _parser = parser_creator->createInstance({}, dataMap());

  // ... loop through file; do:
  // [8 bytes: timestamp][mavlink message]
  // get timestamp, msg
  // _parser.parseMessage(msg, timestamp);

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
