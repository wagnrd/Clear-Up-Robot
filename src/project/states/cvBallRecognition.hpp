#pragma once

#include <Aria.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <thread>

/*
 * Recognizes a coloured (here green) ball using OpenCV.
 * It converts the camera image into HSV and the threshes it in an black and white image.
 * Then it tries to detect circles with the hough transformation and then draws a circle into the output image
 * where the circle has been detected in the threshed image.
 * 
 * AUTHOR: Denis Wagner
 */

struct CVBall
{
  double centerX, centerY, radius;

  CVBall(double centerX = 0, double centerY = 0, double radius = 0)
      : centerX(centerX), centerY(centerY), radius(radius) {}
};

class CVBallRecognition
{
public:
  void initOpenCV();
  CVBall getBall();

  friend void updateOpenCV(CVBallRecognition *cvBallRecognition);

private:
  CVBall currentBall;
  std::thread *openCVThread;
  // green
  int iLowH = 45;
  int iHighH = 75;

  int iLowS = 80;
  int iHighS = 200;

  int iLowV = 90;
  int iHighV = 255;

  // http://opencv-cpp.blogspot.com/2016/10/object-detection-and-tracking-color-separation.html
};