/// \file
/// Maintainer: Luzian Hug
///
///

#include "gui_ref_img_view.h"

#include <QColor>
#include <QImage>
#include <QLabel>

#include <boost/log/trivial.hpp>

namespace MouseTrack {

GUIRefImgView::GUIRefImgView(QWidget *parent) : QWidget(parent) {

  setFixedSize(752, 480);
  _image = std::make_unique<QLabel>(this);
}

void GUIRefImgView::draw(std::shared_ptr<const FrameWindow> f) {
  if (f == nullptr) {
    BOOST_LOG_TRIVIAL(warning) << "Passed empty Frame Window to GUI";
    return;
  }
  const Picture &pic = f->frames()[0].referencePicture;
  std::unique_ptr<QImage> image(
      new QImage(pic.cols(), pic.rows(), QImage::Format_Grayscale8));

  BOOST_LOG_TRIVIAL(debug) << "image width to display: " << image->width();

  for (size_t x = 0; x < pic.cols(); x++) {
    for (size_t y = 0; y < pic.rows(); y++) {
      int i = 255 * pic(y, x);
      image->setPixel(pic.cols() - x - 1, y, qRgb(i, i, i));
    }
  }
  _image->setFixedSize(image->width(), image->height());
  _image->setPixmap(QPixmap::fromImage(*image));
}

} // namespace MouseTrack
