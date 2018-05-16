/// \file
/// Maintainer: Luzian Hug
///
///

#include "main_window.h"
#include "gui_observer.h"

#include <QString>

#include <boost/log/trivial.hpp>

namespace MouseTrack {

MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {

  // setFixedSize(2*640,480);

  _frame_number_label = std::make_unique<QLabel>(this);

  _ref_img_view = std::make_unique<GUIRefImgView>(this);
  setFrameNumber(0);
}

MainWindow::~MainWindow(){}; // This is needed because of a Qt bug

void MainWindow::setFrameNumber(FrameNumber f) {
  BOOST_LOG_TRIVIAL(debug) << "Updating Frame Number: " << f;
  _frame_number = f;
  _frame_number_label->setText("Frame Number: " + QString::number(f));
}

void MainWindow::setFrameWindow(
    std::shared_ptr<const FrameWindow> frame_window) {
  _frame_window = frame_window;
  _ref_img_view->draw(frame_window);
}

} // namespace MouseTrack
