/// \file
/// Maintainer: Felice Serena
///
///

#include "cli_controller.h"
#include "cli_options.h"

#include <boost/log/trivial.hpp>
#include <csignal>
#include <iostream>

static MouseTrack::CliController *activeMouseTrackCliController = nullptr;

/// entry point for signals from the system,
/// passes it on to cli controller
void signalHandler(int signum) {
  activeMouseTrackCliController->handleSignal(signum);
  std::cout << std::flush;
}

namespace MouseTrack {

int CliController::main(int, char **, op::variables_map &) {
  activeMouseTrackCliController = this;
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  // start pipeline
  pipeline().start();

  // wait for pipeline to terminate
  pipeline().join();

  return 0;
}

void CliController::handleSignal(int signal) {
  BOOST_LOG_TRIVIAL(info) << "Interrupt signal (" << signal << ") received.\n";
  if (signal == SIGINT || signal == SIGTERM) {
    if (_isTerminating) {
      // second signal, we terminate hard
      BOOST_LOG_TRIVIAL(info) << "Received second interrupt, hard abort.";
      std::terminate();
    }
    BOOST_LOG_TRIVIAL(info) << "Terminating pipeline ...";
    _isTerminating = true;
    pipeline().stop();
  }
}
} // namespace MouseTrack
