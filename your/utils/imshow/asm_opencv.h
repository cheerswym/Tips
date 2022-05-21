#ifndef ONBOARD_UTILS_IMSHOW_ASM_OPENCV_H_
#define ONBOARD_UTILS_IMSHOW_ASM_OPENCV_H_

/**
   Functions to convert between OpenCV's cv::Mat and Qt's QImage and QPixmap.

   Andy Maloney
   https://asmaloney.com/2013/11/code/converting-between-cvmat-and-qimage-or-qpixmap
**/

/*
   Endianness
   ---

   Although not totally clear from the docs, some of QImage's formats we use
   here are endian-dependent. For example:

      Little Endian
         QImage::Format_ARGB32 the bytes are ordered:    B G R A
         QImage::Format_RGB32 the bytes are ordered:     B G R (255)
         QImage::Format_RGB888 the bytes are ordered:    R G B

      Big Endian
         QImage::Format_ARGB32 the bytes are ordered:    A R G B
         QImage::Format_RGB32 the bytes are ordered:     (255) R G B
         QImage::Format_RGB888 the bytes are ordered:    R G B

   Notice that Format_RGB888 is the same regardless of endianness. Since OpenCV
   expects (B G R) we need to swap the channels for this format.

   This is why some conversions here swap red and blue and others do not.

   This code assumes little endian. It would be possible to add conversions for
   big endian machines though. If you are using such a machine, please feel free
   to submit a pull request on the GitHub page.
*/
// #if Q_BYTE_ORDER == Q_BIG_ENDIAN
// #error Some of QImage's formats are endian-dependant. \
// This file assumes little endian. See comment at top of header.
// #endif

#include <QtCore/QDebug>
#include <QtCore/QtGlobal>
#include <QtGui/QImage>
#include <QtGui/QPixmap>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"

namespace ASM {
// NOTE: This does not cover all cases - it should be easy to add new ones as
// required.
QImage cvMatToQImage(const cv::Mat &inMat);

QPixmap cvMatToQPixmap(const cv::Mat &inMat);

// If inImage exists for the lifetime of the resulting cv::Mat, pass false to
// inCloneImageData to share inImage's data with the cv::Mat directly
//    NOTE: Format_RGB888 is an exception since we need to use a local QImage
//    and thus must clone the data regardless NOTE: This does not cover all
//    cases - it should be easy to add new ones as required.
cv::Mat QImageToCvMat(const QImage &inImage, bool inCloneImageData = true);

// If inPixmap exists for the lifetime of the resulting cv::Mat, pass false to
// inCloneImageData to share inPixmap's data with the cv::Mat directly
//    NOTE: Format_RGB888 is an exception since we need to use a local QImage
//    and thus must clone the data regardless
inline cv::Mat QPixmapToCvMat(const QPixmap &inPixmap,
                              bool inCloneImageData = true);
}  // namespace ASM

#endif  // ONBOARD_UTILS_IMSHOW_ASM_OPENCV_H_
