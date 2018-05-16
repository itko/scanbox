/// \file
/// Maintainer: Luzian Hug
///
///

#include "qt_controller.h"

#include "gui_observer.h"
#include "main_window.h"
#include <QApplication>
#include <QPushButton>
#include <boost/log/trivial.hpp>

namespace MouseTrack {

int QtController::main(int argc, char *argv[], op::variables_map &cli_options) {

  // set up application
  QApplication app(argc, argv);
  // show a button in a window
  MainWindow window;

  std::unique_ptr<GUIObserver> observer(new GUIObserver(&window));
  pipeline().addObserver(observer.get());

  window.show();
  // start pipeline
  pipeline().start();

  // start run loop
  int returnCode = app.exec();

  pipeline().join();

  return returnCode;
}

} // namespace MouseTrack
