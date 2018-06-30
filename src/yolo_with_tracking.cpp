#include <string>
#include <vector>
#include <fstream>

#define OPENCV
#define GPU

// import functions from YOLO DLL
#include "yolo_v2_class.hpp"

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

void draw_boxes(Mat mat_img, vector<bbox_t> result_vec, std::vector<std::string> obj_names) 
{
	for (auto &i : result_vec)
	{
		Scalar color = obj_id_to_color(i.obj_id);
		rectangle(mat_img, Rect(i.x, i.y, i.w, i.h), color, 3);
		if (obj_names.size() > i.obj_id)
		{
			putText(mat_img, obj_names[i.obj_id], Point2f(i.x, i.y - 10), FONT_HERSHEY_COMPLEX_SMALL, 1, color);
		}
	}

	imshow("window name", mat_img);
}

vector<string> obj_names_from_file(string const filename)
{
	ifstream f(filename);
	vector<string> file_lines;
	if (!f.is_open())
	{
		return file_lines;
	}

	string line;
	while (getline(f, line))
	{
		file_lines.push_back(line);
	}

	f.close();
	return file_lines;
}

int main()
{
	string names_file = "data/coco.names";
	string weights_file = "yolov3.weights";
	string cfg_file = "yolov3.cfg";

	Detector detector(cfg_file, weights_file);
	VideoCapture cap(0);
	auto obj_names = obj_names_from_file(names_file);

	// check if we succeeded
	if (!cap.isOpened())
	{
		return -1;
	}

	while (true)
	{
		Mat frame;
		cap >> frame;
		auto result_vec = detector.detect(frame);
		draw_boxes(frame, result_vec, obj_names);

		if (waitKey(30) >= 0)
		{
			break;
		}
	}

	return 0;
}