#include <string>
#include <vector>
#include <fstream>

#define OPENCV
#define GPU

// import functions from YOLO DLL
#include "yolo_v2_class.hpp"

#include "tracker.h"

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

int k;

void draw_boxes(Mat mat_img, vector<bbox_t> result_vec, std::vector<std::string> obj_names) 
{
	for (auto &i : result_vec)
	{
		Scalar color = obj_id_to_color(i.obj_id);
		rectangle(mat_img, Rect(i.x, i.y, i.w, i.h), color, 3);
		if (obj_names.size() > i.obj_id)
		{
			string obj_name = obj_names[i.obj_id];
			if (i.track_id > 0)
			{
				obj_name += " - " + to_string(i.track_id);
			}

			putText(mat_img, obj_name, Point2i(i.x, i.y - 10), FONT_HERSHEY_COMPLEX_SMALL, 1, color);
		}
	}

	cv::imshow("res/window name", mat_img);
	// cv::imwrite("res/res" + to_string(k) + ".jpg", mat_img);
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
	detector.nms = float(0.2);
	Tracker tracker;

	// default web-cam
	VideoCapture cap(0);
	// check if we succeeded
	if (!cap.isOpened())
	{
		return -1;
	}

	k = 0;
	auto obj_names = obj_names_from_file(names_file);
	Mat frame;
	vector<bbox_t> result_vec, tracking_vec, detection_vec;
	while (true)
	{
		try
		{
			cap >> frame;
			k++;
			// cv::imwrite("res/cur_frame" + to_string(k) + ".jpg", frame);
			if (!frame.empty())
			{
				detection_vec = detector.detect(frame);
				detection_vec = detector.tracking_id(detection_vec);
				result_vec = detection_vec;
				tracking_vec = tracker.get_result();

				for (auto &i: tracking_vec)
				{
					bool is_detected = false;
					for (auto &j: result_vec)
					{
						if ((i.track_id == j.track_id) && (i.obj_id == j.obj_id))
						{
							is_detected = true;
						}
					}
					
					if (!is_detected)
					{
						result_vec.push_back(i);
					}
				}

				draw_boxes(frame, result_vec, obj_names);
				tracker.extrapolate(detection_vec, frame.size());
			}
			else
			{
				break;
			}

			if (waitKey(30) >= 0)
			{
				break;
			}
		}

		catch (exception &e)
		{
			cerr << "exception " << e.what() << endl;
			getchar();
		}
		catch (...)
		{
			cerr << "unknown exception" << endl;
			getchar();
		}
	}

	return 0;
}