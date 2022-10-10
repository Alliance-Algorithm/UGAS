#include <opencv.hpp>
using namespace std;
using namespace cv;

int main() {
	Mat img = imread("resources/test.png");
	imshow("Project test img", img);
	waitKey(0);
	return 0;
}