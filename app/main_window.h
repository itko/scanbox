/// \file
/// Maintainer: Luzian Hug
///
///

#pragma once
#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include <memory>

#include "generic/frame_window.h"
#include "generic/types.h"
#include "gui_ref_img_view.h"

namespace MouseTrack {

class MainWindow : public QWidget {
  Q_OBJECT
public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow(); // This is needed because of a Qt bug

  void setFrameNumber(FrameNumber f);
  void setFrameWindow(std::shared_ptr<const FrameWindow> frame_window);

private:
  FrameNumber _frame_number;
  std::unique_ptr<QLabel> _frame_number_label;
  std::shared_ptr<const FrameWindow> _frame_window;
  std::unique_ptr<GUIRefImgView> _ref_img_view;
};

} // namespace MouseTrack
