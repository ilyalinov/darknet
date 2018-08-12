#pragma once
#include <vector>
#include <deque>
#include <opencv2/opencv.hpp>

#include "../../../src/yolo_v2_class.hpp"

class Tracker
{
	struct tracked_bbox_t
	{
		double shiftX;
		double shiftY;
		int last_detection_frames_ago;
		bool current_detection;
		bbox_t bbox;

		tracked_bbox_t() : shiftX(0), shiftY(0), last_detection_frames_ago(track_frames_history), current_detection(false) {}
	};

	static const int det_frames_history = 30;
	static const int track_frames_history = 3;
	std::deque<std::vector<bbox_t>> detection_history;
	std::vector<tracked_bbox_t> tracking_history;
public:
	Tracker();

	// exctrapolation
	void extrapolate(std::vector<bbox_t> detection_vec, cv::Size size);

	// returns vector with all tracked objects
	std::vector<bbox_t> get_result();

private:
	// log new detection result
	void add_new_result(std::vector<bbox_t> detection_vec);

	tracked_bbox_t extrapolate_bbox(bbox_t b);

	std::pair<double, double> calculate_shift(bbox_t b1, bbox_t b2);

	void set_cur_detection_flag_to_false();
};