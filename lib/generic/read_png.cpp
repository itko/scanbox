/// \file
/// Maintainer: Felice Serena
///
///

#include "read_png.h"

#include <boost/log/trivial.hpp>
#include <png.h>

namespace MouseTrack {

Picture read_png_normalized(const std::string &path) {
  auto mat = read_png(path);
  Picture result = mat.cast<double>();
  result /= 255.0;
  return result;
}

// Modified version from: http://zarb.org/~gc/html/libpng.html
Eigen::MatrixXi read_png(const std::string &path) {
  FILE *fp = fopen(path.c_str(), "rb");
  if (!fp) {
    throw "File could not be opened.";
  }
  png_byte header[8];
  size_t transfered = fread(header, 1, 8, fp);
  if (transfered != 8) {
    throw "read_png: couldn't read header from png file.";
  }
  if (0 != png_sig_cmp(header, 0, 8)) {
    throw "File is not a PNG.";
  }
  png_structp png_ptr =
      png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

  if (!png_ptr) {
    throw "failed to creat struct";
  }

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    throw "failed to create info struct";
  }

  if (setjmp(png_jmpbuf(png_ptr))) {
    throw "error during init io";
  }
  png_init_io(png_ptr, fp);
  png_set_sig_bytes(png_ptr, 8);

  // read all chunks up to but not including the data
  png_read_info(png_ptr, info_ptr);

  const int width = png_get_image_width(png_ptr, info_ptr);
  const int height = png_get_image_height(png_ptr, info_ptr);
  const png_byte color_type = png_get_color_type(png_ptr, info_ptr);
  const png_byte channels = png_get_channels(png_ptr, info_ptr);

  png_read_update_info(png_ptr, info_ptr);

  /* read file */
  if (setjmp(png_jmpbuf(png_ptr))) {
    throw "Error during reading image";
  }

  switch (color_type) {
  case PNG_COLOR_TYPE_GRAY:
    // OK
    break;
  case PNG_COLOR_TYPE_GRAY_ALPHA:
    // OK
    break;
  case PNG_COLOR_TYPE_PALETTE:
    fclose(fp);
    throw "Palette pngs are not supported";
  case PNG_COLOR_TYPE_RGB:
    BOOST_LOG_TRIVIAL(info)
        << "Color PNG not supported, only reading R channel";
    break;
  case PNG_COLOR_TYPE_RGBA:
    BOOST_LOG_TRIVIAL(info)
        << "Color PNG not supported, only reading R channel";
    break;
  }

  // read image
  Eigen::MatrixXi result(height, width);

  png_byte *row = (png_byte *)malloc(png_get_rowbytes(png_ptr, info_ptr));
  for (int y = 0; y < height; y += 1) {
    png_read_row(png_ptr, row, NULL);
    for (int x = 0; x < width; x += 1) {
      png_byte *pixel = &(row[x * channels]);
      result(y, x) = pixel[0];
    }
  }

  free(row);

  fclose(fp);

  return result;
}

} // namespace MouseTrack
