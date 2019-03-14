#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  Mat img = imread("map_new.pgm", CV_8U);
  namedWindow("original map", WINDOW_NORMAL);
  imshow("original map", img);
  Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));
  Mat out;
  erode(img, out, element);
  namedWindow("output", WINDOW_NORMAL);
  imshow("output", out);
  imwrite("map_eroded.pgm", out);
  waitKey(0);
  return 0;
}