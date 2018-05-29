#include "cli_controller.h"
#include "cli_options.h"
#include "pipeline_factory.h"
#include "pipeline_timer.h"
#include "pipeline_writer.h"

#ifdef ENABLE_GUI
#include "qt_controller.h"
#endif

#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <iostream>
#include <memory>

using namespace MouseTrack;

/// Set global log level according to given string, valid values:
/// none, trace, debug, info, warning, error, fatal
/// Unknown values default to info
void setLogLevel(const std::string &level) {
  namespace log = boost::log::trivial;
  if (level == "none") {
    boost::log::core::get()->set_filter(log::severity > log::fatal);
    return;
  }
  auto logFilter = log::info;
  if (level == "trace") {
    logFilter = log::trace;
  } else if (level == "debug") {
    logFilter = log::debug;
  } else if (level == "info") {
    logFilter = log::info;
  } else if (level == "warning") {
    logFilter = log::warning;
  } else if (level == "error") {
    logFilter = log::error;
  } else if (level == "fatal") {
    logFilter = log::fatal;
  } else {
    // empty: default for unknown entries
  }
  boost::log::core::get()->set_filter(log::severity >= logFilter);
}

op::variables_map parseCli(int argc, char *argv[],
                           const op::options_description &option_desc) {

  op::positional_options_description pos_desc;
  // allow to shorten "-src <path>" to "<path>"
  pos_desc.add("src", -1);
  op::command_line_parser parser{argc, argv};
  parser.options(option_desc).positional(pos_desc).allow_unregistered();
  op::parsed_options parsed_options = parser.run();
  op::variables_map cli_options;
  op::store(parsed_options, cli_options);
  return cli_options;
}

int main(int argc, char *argv[]) {
  op::options_description option_desc = cli_options();
  op::variables_map cli_options;

  try {
    cli_options = parseCli(argc, argv, option_desc);
  } catch (const std::string &e) {
    std::cerr << "Exception caught while reading cli arguments: " << e
              << std::endl;
  } catch (const char *e) {
    std::cerr << "Exception caught while reading cli arguments: " << e
              << std::endl;
  } catch (const int e) {
    std::cerr << "Exception caught while reading cli arguments: " << e
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Exception caught while reading cli arguments: " << e.what()
              << std::endl;
  }

  if (cli_options.count("help")) {
    std::cout << option_desc;
    return 0;
  }

  setLogLevel(cli_options["log"].as<std::string>());

  const bool gui_mode = cli_options.count("cli") == 0;

  std::unique_ptr<Controller> controller;

  try {
    if (gui_mode) {
#ifdef ENABLE_GUI
      controller = std::unique_ptr<QtController>(new QtController());
#else
      std::cerr << "Binary was not compiled with a GUI option. See --help for "
                   "other options.";
      return -1;
#endif
    } else {
      if (cli_options.count("src") == 0) {
        std::cout
            << "Source path must not be empty for command line interface. "
               "See --help for further options."
            << std::endl;
        return -1;
      }
      controller = std::unique_ptr<CliController>(new CliController());
    }
  } catch (const std::string &e) {
    std::cerr << "Exception caught while creating controller: " << e
              << std::endl;
    return 16;
  } catch (char *e) {
    std::cerr << "Exception caught while creating controller: " << e
              << std::endl;
    return 16;
  } catch (int e) {
    std::cerr << "Exception caught while creating controller: " << e
              << std::endl;
    return 16;
  } catch (const std::exception &e) {
    std::cerr << "Exception caught while creating controller: " << e.what()
              << std::endl;
    return 16;
  }
  PipelineFactory pipelineFactory;
  PipelineTimer timer;
  std::unique_ptr<PipelineWriter> writer;

  try {
    controller->pipeline() =
        std::move(pipelineFactory.fromCliOptions(cli_options));
    if (cli_options.count("pipeline-timer-log")) {
      std::string p = cli_options["pipeline-timer-log"].as<std::string>();
      timer.logPath(p);
    }
    if (cli_options.count("pipeline-timer")) {
      controller->pipeline().addObserver(&timer);
    }

    if (cli_options.count("out-dir")) {
      writer = std::make_unique<PipelineWriter>(
          cli_options["out-dir"].as<std::string>());
      controller->pipeline().addObserver(writer.get());
      // TODO: we should make this configurable at some points
      writer->writeRawFrameWindow = false;
      // by default, remove label 5, which in our case is background
      // TODO: This should be configurable
      writer->labelsToIgnore().insert(5);
    }
  } catch (const std::string &e) {
    std::cerr << "Exception caught while creating pipeline: " << e << std::endl;
    return 15;
  } catch (char *e) {
    std::cerr << "Exception caught while creating pipeline: " << e << std::endl;
    return 15;
  } catch (int e) {
    std::cerr << "Exception caught while creating pipeline: " << e << std::endl;
    return 15;
  } catch (const std::exception &e) {
    std::cerr << "Exception caught while creating pipeline: " << e.what()
              << std::endl;
    return 15;
  }

  try {
    int errorCode = controller->main(argc, argv, cli_options);
    std::cout << std::flush;
    return errorCode;
  } catch (const std::string &e) {
    std::cerr << "Exception caught from controller: " << e << std::endl;
  } catch (char *e) {
    std::cerr << "Exception caught from controller: " << e << std::endl;
  } catch (int e) {
    std::cerr << "Exception caught from controller: " << e << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Exception caught from controller: " << e.what() << std::endl;
  }
  return 17;
}
