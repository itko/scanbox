/// \file
/// Maintainer: Luzian Hug
///
///

#pragma once
#include <QLabel>
#include <QWidget>

#include <memory>

#include "generic/frame_window.h"

namespace MouseTrack {

class GUIRefImgView : public QWidget {
  Q_OBJECT
public:
  explicit GUIRefImgView(QWidget *parent);
  void draw(std::shared_ptr<const FrameWindow> f);

private:
  QLabel *_image;
};

} // namespace MouseTrack
