#include "cvBallRecognition.hpp"
#include <Aria.h>

void updateOpenCV(CVBallRecognition *cvBallRecognition)
{
    // https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html
    cv::VideoCapture robotCam(1);

    if (!robotCam.isOpened()) // if not success, exit program
    {
        // std::cout << "Cannot open the web cam" << std::endl;
        ArLog::log(ArLog::Terse, "CV: Cannot open the robot cam.");
        return;
    }

    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    // green ball
    int iLowH = 45;
    int iHighH = 75;

    int iLowS = 80;
    int iHighS = 200;

    int iLowV = 90;
    int iHighV = 255;

    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    //Create vector of vec3f objects to store the houghCircle result
    std::vector<cv::Vec3f> circles;

    while (true)
    {
        cv::Mat imgOriginal;

        bool bSuccess = robotCam.read(imgOriginal); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
            //std::cout << "Cannot read a frame from video stream" << std::endl;
            ArLog::log(ArLog::Terse, "Cannot read frame from video stream.");
            break;
        }

        cv::Mat imgHSV;

        cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        //Blur to reduce false circles to be detected
        cv::GaussianBlur(imgHSV, imgHSV, cv::Size(9, 9), 5, 5);

        cv::Mat imgThresholded;

        cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV),
                    imgThresholded); //Threshold the image

        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        //morphological closing (fill small holes in the foreground)
        dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        //Detecting Circles
        cv::HoughCircles(imgThresholded, circles, CV_HOUGH_GRADIENT, 3, imgThresholded.rows, 50, 50);

        for (auto circle : circles)
        {
            cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
            //draw center of the circle
            cv::circle(imgOriginal, center, 3, cv::Scalar(0, 255, 0), -1);
            //draw outer circle
            cv::circle(imgOriginal, center, circle[2], cv::Scalar(0, 0, 255), 2);
        }

        imshow("Thresholded Image", imgThresholded); //show the thresholded image
        imshow("Original", imgOriginal);             //show the original image

        if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            break;
        }
    }
}

void CVBallRecognition::initOpenCV()
{
    openCVThread = new std::thread(updateOpenCV, this);
}

CVBall BgetBall()
{
    return CVBall();
}