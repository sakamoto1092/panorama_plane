#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include<opencv2/nonfree/nonfree.hpp>
#include<opencv2/nonfree/features2d.hpp>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <boost/program_options.hpp>

#define PANO_W 2560
#define PANO_H 1440

using namespace std;
using namespace cv;
using boost::program_options::options_description;
using boost::program_options::value;
using boost::program_options::variables_map;
using boost::program_options::store;
using boost::program_options::parse_command_line;
using boost::program_options::notify;


// Tilt :
int SetTiltRotationMatrix(Mat *tiltMatrix, double tilt_deg) {
	double tilt_angle;

	tilt_angle = tilt_deg / 180.0 * M_PI;
	(*tiltMatrix).at<double> (1, 1) = cos(tilt_angle);
	(*tiltMatrix).at<double> (1, 2) = -sin(tilt_angle);
	(*tiltMatrix).at<double> (2, 1) = sin(tilt_angle);
	(*tiltMatrix).at<double> (2, 2) = cos(tilt_angle);

	//	cvmSet(tiltMatrix, 1, 1, cos(tilt_angle));
	//	cvmSet(tiltMatrix, 1, 2, -sin(tilt_angle));
	//	cvmSet(tiltMatrix, 2, 1, sin(tilt_angle));
	//	cvmSet(tiltMatrix, 2, 2, cos(tilt_angle));
	return 0;
}

int SetPanRotationMatrix(Mat *panMatrix, double pan_deg) {
	double pan_angle;
	pan_angle = pan_deg / 180.0 * M_PI;

	(*panMatrix).at<double> (2, 2) = cos(pan_angle);
	(*panMatrix).at<double> (2, 0) = -sin(pan_angle);
	(*panMatrix).at<double> (0, 2) = sin(pan_angle);
	(*panMatrix).at<double> (0, 0) = cos(pan_angle);
	//	cvmSet(panMatrix, 2, 2, cos(pan_angle));
	//	cvmSet(panMatrix, 2, 0, -sin(pan_angle));
	//	cvmSet(panMatrix, 0, 2, sin(pan_angle));
	//	cvmSet(panMatrix, 0, 0, cos(pan_angle));
	return 0;
}

// Roll :
int SetRollRotationMatrix(Mat *rollMatrix, double roll_deg) {
	double roll_angle;

	roll_angle = roll_deg / 180.0 * M_PI;
	(*rollMatrix).at<double> (0, 0) = cos(roll_angle);
	(*rollMatrix).at<double> (0, 1) = -sin(roll_angle);
	(*rollMatrix).at<double> (1, 0) = sin(roll_angle);
	(*rollMatrix).at<double> (1, 1) = cos(roll_angle);
	//	cvmSet(rollMatrix, 0, 0, cos(roll_angle));
	//	cvmSet(rollMatrix, 0, 1, -sin(roll_angle));
	//	cvmSet(rollMatrix, 1, 0, sin(roll_angle));
	//	cvmSet(rollMatrix, 1, 1, cos(roll_angle));
	return 0;
}

// Pitch :
int SetPitchRotationMatrix(Mat *pitchMatrix, double pitch_deg) {
	double pitch_angle;

	pitch_angle = pitch_deg / 180.0 * M_PI;
	(*pitchMatrix).at<double> (1, 1) = cos(pitch_angle);
	(*pitchMatrix).at<double> (1, 2) = -sin(pitch_angle);
	(*pitchMatrix).at<double> (2, 1) = sin(pitch_angle);
	(*pitchMatrix).at<double> (2, 2) = cos(pitch_angle);
	//	cvmSet(pitchMatrix, 1, 1, cos(pitch_angle));
	//	cvmSet(pitchMatrix, 1, 2, -sin(pitch_angle));
	//	cvmSet(pitchMatrix, 2, 1, sin(pitch_angle));
	//	cvmSet(pitchMatrix, 2, 2, cos(pitch_angle));
	return 0;
}

// Yaw
int SetYawRotationMatrix(Mat *yawMatrix, double yaw_deg) {
	double yaw_angle;

	yaw_angle = yaw_deg / 180.0 * M_PI;
	(*yawMatrix).at<double> (2, 2) = cos(yaw_angle);
	(*yawMatrix).at<double> (2, 0) = -sin(yaw_angle);
	(*yawMatrix).at<double> (0, 2) = sin(yaw_angle);
	(*yawMatrix).at<double> (0, 0) = cos(yaw_angle);
	//	cvmSet(yawMatrix, 2, 2, cos(yaw_angle));
	//	cvmSet(yawMatrix, 2, 0, -sin(yaw_angle));
	//	cvmSet(yawMatrix, 0, 2, sin(yaw_angle));
	//	cvmSet(yawMatrix, 0, 0, cos(yaw_angle));
	return 0;
}

void setHomographyReset(Mat* homography) {
	cvZero(homography);
	(*homography).at<double> (0, 0) = 1;
	(*homography).at<double> (1, 1) = 1;
	(*homography).at<double> (2, 2) = 1;
	//cvmSet(homography, 0, 0, 1);
	//cvmSet(homography, 1, 1, 1);
	//cvmSet(homography, 2, 2, 1);
}

double compareSURFDescriptors(const float* d1, const float* d2, double best,
		int length) {
	double total_cost = 0;
	assert(length % 4 == 0);
	for (int i = 0; i < length; i += 4) {
		double t0 = d1[i] - d2[i];
		double t1 = d1[i + 1] - d2[i + 1];
		double t2 = d1[i + 2] - d2[i + 2];
		double t3 = d1[i + 3] - d2[i + 3];
		total_cost += t0 * t0 + t1 * t1 + t2 * t2 + t3 * t3;
		if (total_cost > best)
			break;
	}
	return total_cost;
}


void get_histimage(Mat srcimage, Mat *hist_image, Mat count) {

	MatND hist; // ヒストグラム
	Scalar mean, dev; // 平均と分散の格納先
	float hrange[] = { 0, 256 }; // ヒストグラムの輝度値レンジ
	const float* range[] = { hrange }; // チャネルごとのヒストグラムの輝度値レンジ（グレースケールなので要素数は１）
	int binNum = 126; // ヒストグラムの量子化の値
	int histSize[] = { binNum }; // チャネルごとのヒストグラムの量子化の値
	int channels[] = { 0 }; // ヒストグラムを求めるチャネル指定
	int dims = 1; // 求めるヒストグラムの数


	float max_dev = FLT_MIN, min_dev = FLT_MAX; // エッジ画像におけるヒストグラムの分散のmin max
	float max_mean = FLT_MIN, min_mean = FLT_MAX; // エッジ画像におけるヒストグラムの平均のmin max
	float sum_mean = 0.0;
	//	Mat count(10, 10, CV_32F, cv::Scalar(0)); // エッジの数を格納するカウンタ

	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			double max_value;
			int bin_w;

			calcHist(&Mat(srcimage, cv::Rect(j * 128, i * 72, 128, 72)), 1,
					channels, Mat(), hist, dims, histSize, range, true, false);

			meanStdDev(hist, mean, dev);
			count.at<float> (i, j) = dev[0];

			if (dev[0] < min_dev)
				min_dev = dev[0];
			if (dev[0] > max_dev)
				max_dev = dev[0];

			if (mean[0] < min_mean)
				min_mean = mean[0];
			if (mean[0] > max_mean)
				max_mean = mean[0];

			sum_mean += mean[0];
			//			std::cout << "count : " << mean << std::endl;


			minMaxLoc(hist, NULL, &max_value, NULL, NULL);

			hist *= hist_image[i * 10 + j].rows / max_value;
			bin_w = cvRound((double) 260 / 256);

			for (int k = 0; k < 256; k++)
				rectangle(hist_image[i * 10 + j], Point(k * bin_w, hist_image[i
						* 10 + j].rows), cvPoint((k + 1) * bin_w, hist_image[i
						* 10 + j].rows - cvRound(hist.data[k])),
						cvScalarAll(0), -1, 8, 0);
		}
	}

}
/*
 *  透視投影変換後の画像をパノラマ平面にマスクを用いて
 *  上書きせずに未投影の領域のみに投影する関数
 *
 * @Param  src パノラマ画像に投影したい画像
 * @Param  dst パノラマ画像
 * @Param mask 投影済みの領域を表したマスク画像
 * @Param  roi 投影したい画像の領域を表した画像
 *
 *  （＊maskは処理後に更新されて返される）
 */
void make_pano(Mat src, Mat dst, Mat mask, Mat roi) {

	//サイズの一致を確認
	if (src.cols == dst.cols && src.rows == dst.rows) {
		int h = dst.rows;
		int w = dst.cols;
		for (int i = 0; i < w; i++) {
			for (int j = 0; j < h; j++) {
				if (mask.at<unsigned char> (j, i) == 0) {

					dst.at<Vec3b> (j, i) = src.at<Vec3b> (j, i);
					if (roi.at<unsigned char> (j, i) == 255)
						mask.at<unsigned char> (j, i) = roi.at<unsigned char> (
								j, i);
				}
			}
		}
	}
}

int main(int argc, char** argv) {

	VideoWriter VideoWriter; // パノラマ動画
	VideoCapture cap; // ビデオファイル

	// 動画から取得した各種画像の格納先
	Mat image; // 投影先の画像
	Mat object; // 変換元の画像
	Mat gray_image; // 変換元の微分画像
	Mat center_img; // センターサークル画像

	// ホモグラフィ行列による変換後の画像格納先
	Mat transform_image; // 画像単体での変換結果
	Mat transform_image2; // パノラマ平面への変換結果

	// マスク関係画像（既存のパノラマの未描画領域のみに投影するため）
	Mat mask; // パノラマ画像のマスク
	Mat pano_black; // パノラマ画像と同じサイズの黒画像
	Mat white_img; // フレームと同じサイズの白画像

	int skip; // 合成開始フレーム番号
	long end; // 合成終了フレーム番号
	long frame_num; // 現在のフレーム位置
	int blur; // ブレのしきい値
	long FRAME_MAX; // 動画の最大フレーム数
	int FRAME_T; // フレーム飛ばし間隔

	// 各種フラグ
	bool f_comp = false; // 線形補完
	bool f_center = false; // センターサークル中心
	bool f_video = false; // ビデオ書き出し

	float fps = 30; // 書き出しビデオのfps
	string n_video; // 書き出しビデオファイル名
	string cam_data; // 映像センサファイル名
	string n_center; // センターサークル画像名

	// 映像 センサファイル名取得
	char imagefileName[256];
	char timefileName[256];
	char sensorfileName[256];
	FILE *SensorFP;

	// 全ホモグラフィ行列をアウトプット
	std::ofstream out_Hmat("mat_homography.txt", std::ios::out);

	// 各種アルゴリズムによる特徴点検出および特徴量記述
	string algorithm_type;
	Ptr<Feature2D> feature;
	// SIFT
	//SIFT feature;

	// SURF
	//SURF feature(5, 3, 4, true);

	// ORB
	//ORB featur;

	// 特徴点のマッチャー（ユークリッド距離とハミング距離で使用する関数を変える）
	FlannBasedMatcher matcher;
	//BFMatcher matcher(cv::NORM_HAMMING, true);

	// 対応点の対の格納先
	std::vector<cv::DMatch> matches; // matcherにより求めたおおまかな対を格納

	// 特徴点の集合と特徴量
	std::vector<KeyPoint> objectKeypoints, imageKeypoints;
	Mat objectDescriptors, imageDescriptors;

	vector<Point2f> pt1, pt2; // 画像対における特徴点座の集合

	// より良いペアの格納先
	double min_dist = DBL_MAX;
	std::vector<cv::DMatch> good_matches;
	std::vector<KeyPoint> good_objectKeypoints;
	std::vector<KeyPoint> good_imageKeypoints;

	Mat homography = Mat(3, 3, CV_64FC1); // 画像対におけるホモグラフィ
	Mat h_base = cv::Mat::eye(3, 3, CV_64FC1); // パノラマ平面へのホモグラフィ
	int n, w, h;

	// パノラマ平面の構成
	int roll = 0;
	int pitch = 0;
	int yaw = 0;
	Mat A1Matrix = cv::Mat::eye(3, 3, CV_64FC1);
	Mat A2Matrix = cv::Mat::eye(3, 3, CV_64FC1);

	Mat rollMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	Mat pitchMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	Mat yawMatrix = cv::Mat::eye(3, 3, CV_64FC1);

	//　パノラマ平面への射影行列の作成
	A1Matrix.at<double> (0, 2) = -640;
	A1Matrix.at<double> (1, 2) = -360;
	A1Matrix.at<double> (2, 2) = 1080;


	A2Matrix.at<double> (0, 0) = 600;
	A2Matrix.at<double> (1, 1) = 600;
	A2Matrix.at<double> (0, 2) = PANO_W / 2;
	A2Matrix.at<double> (1, 2) = PANO_H / 2;

	try {
		// コマンドラインオプションの定義
		options_description opt("Usage");
		opt.add_options()("cam", value<std::string> ()->default_value(
				"cam_data.txt"), "動画名やセンサファイル名が記述されたファイルの指定")("center", value<
				std::string> (), "センター画像の指定")("start,s",
				value<int> ()->default_value(0), "スタートフレームの指定")("end,e", value<
				int> (), "終了フレームの指定")("comp", value<bool> ()->default_value(
				false), "補完の設定")("inter,i", value<int> ()->default_value(9),
				"取得フレームの間隔")("blur,b", value<int> ()->default_value(0),
				"ブラーによるフレームの破棄の閾値")("video,v", value<std::string> (),
				"書きだす動画ファイル名の指定")("fps,f", value<int> ()->default_value(30),
				"書きだす動画のフレームレートの指定")("algo,a", value<string> ()->default_value(
				"SURF"), "特徴点抽出等のアルゴリズムの指定")("help,h", "ヘルプの出力");

		// オプションのマップを作成
		variables_map vm;
		store(parse_command_line(argc, argv, opt), vm);
		notify(vm);

		// 必須オプションの確認
		if (vm.count("help")) {
			cout << "  [option]... \n" << opt << endl;
			return -1;
		}

		// 各種オプションの値を取得
		cam_data = vm["cam"].as<string> (); // 映像 センサファイル名

		if (vm.count("start")) // スタートフレーム番号
			skip = vm["start"].as<int> ();
		else
			skip = 0;

		if (vm.count("end")) // 終了フレーム番号
			end = vm["end"].as<int> ();

		if (vm.count("center")) { // センターサークル画像名
			n_center = vm["center"].as<string> ();
			f_center = true;
		}

		if (vm.count("video")) { // 書き出し動画ファイル名
			n_video = vm["video"].as<string> ();
			f_video = true;
		}

		if (vm.count("algo"))
			algorithm_type = vm["algo"].as<string> ();

		f_comp = vm["comp"].as<bool> (); // 補完の有効無効
		FRAME_T = vm["inter"].as<int> (); // フレーム取得間隔
		blur = vm["blur"].as<int> (); // 手ブレ閾値
		fps = vm["fps"].as<int> (); // 書き出し動画のfps

	} catch (exception& e) {
		cerr << "error: " << e.what() << "\n";
		return -1;
	} catch (...) {
		cerr << "Exception of unknown type!\n";
		return -1;
	}

	// 映像　センサファイル名を取得
	if ((SensorFP = fopen(cam_data.c_str(), "r")) == NULL) {
		cerr << "cant open cam_data.txt" << endl;
		return -1;
	}
	fscanf(SensorFP, "%s", imagefileName);
	fscanf(SensorFP, "%s", timefileName);
	fscanf(SensorFP, "%s", sensorfileName);
	fclose(SensorFP);

	if (f_center)
		center_img = imread(n_center);

	if (f_center && center_img.empty()) {
		cerr << "Cant open center_img " << n_center << endl;
		f_center = false;
		return -1;
	}
	// 動画ファイルをオープン
	if (!(cap.open(string(imagefileName)))) {
		fprintf(stderr, "Avi Not Found!!\n");
		return -1;
	}

	// 総フレーム数の取得
	FRAME_MAX = cap.get(CV_CAP_PROP_FRAME_COUNT);
	if (FRAME_MAX < end)
		end = FRAME_MAX;

	std::cout << "Video Property : total flame = " << FRAME_MAX << endl;
	cout << "fps = " << fps << endl;

	// 合成開始フレームまでスキップ
	if (skip > 0) {
		cap.set(CV_CAP_PROP_POS_FRAMES, skip + 1);
		frame_num = skip + 1;
	} else {
		frame_num = 0;
	}
	feature = Feature2D::create(algorithm_type);
	if (algorithm_type.compare("SURF") == 1) {
		feature->set("extended", 1);
		feature->set("hessianThreshold", 50);
		feature->set("nOctaveLayers", 4);
		feature->set("nOctaves", 3);
		feature->set("upright", 0);
	}

	//	double tt = (double) cvGetTickCount();

	// 各種回転をパノラマ平面に適用
	SetRollRotationMatrix(&rollMatrix, (double) roll);
	SetPitchRotationMatrix(&pitchMatrix, (double) pitch);
	SetYawRotationMatrix(&yawMatrix, (double) yaw);

	// 最終的なパノラマ平面へのホモグラフィ行列を計算
	h_base = A2Matrix * A1Matrix * yawMatrix * pitchMatrix * rollMatrix;

	cout << h_base << endl;

	/*logging*/
	ofstream log("composition_log.txt");
	vector<string> v_log_str;
	string log_str;

	feature->getParams(v_log_str);

	log << "<avi_file_name>" << endl << imagefileName << endl;
	log << "<A1>" << endl << A1Matrix << endl;
	log << "<A2>" << endl << A2Matrix << endl;
	log << "<roll pitch yaw>" << endl;
	log << roll << " " << pitch << " " << yaw << endl;
	log << "<FRAME_ T> " << endl << FRAME_T << endl;
	log << "<Comp>" << endl;
	log << f_comp << endl;
	log << "<use center>" << endl;
	log << f_center << endl;
	log << "<deblur>" << endl << blur << endl;
	log << "<Algorithm> " << endl << algorithm_type << endl;
	log << "<Algorithm Param>" << endl;
	for (int ii = 0; ii < v_log_str.size(); ii++)
		log << v_log_str[ii] << " " << feature->getDouble(v_log_str[ii])
				<< endl;

	// 最初のフレームを取得（センターサークル画像に差し替え）
	if (f_center)
		image = center_img.clone();
	else
		cap >> image;

	cvtColor(image, gray_image, CV_RGB2GRAY);

	// マスクイメージ関係を生成
	//  パノラマ平面へ射影する際のマスク処理
	w = image.cols;
	h = image.rows;
	mask = Mat(PANO_H, PANO_W, CV_8U, cv::Scalar(0));
	pano_black = Mat(PANO_H, PANO_W, CV_8U, cv::Scalar(0));
	white_img = Mat(image.rows, image.cols, CV_8U, cv::Scalar(255));
	transform_image2 = cv::Mat::zeros(Size(PANO_W, PANO_H), CV_8UC3);

	// 特徴点の検出と特徴量の記述
	feature->operator ()(gray_image, Mat(), imageKeypoints, imageDescriptors);

	warpPerspective(image, transform_image, h_base, Size(PANO_W, PANO_H)); // 先頭フレームをパノラマ平面へ投影

	// パノラマ動画ファイルを作成
	if (f_video)
		VideoWriter.open(n_video, CV_FOURCC('X', 'V', 'I', 'D'), (int) fps,
				Size(PANO_W * 0.75, PANO_H * 0.75), 1);

	warpPerspective(white_img, pano_black, h_base, Size(PANO_W, PANO_H),
			CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);

	make_pano(transform_image, transform_image2, mask, pano_black);
	Mat mask2;
	erode(mask, mask2, cv::Mat(), cv::Point(-1, -1), 10);
	//		imshow("mask1",mask);
	///		waitKey(0);
	//		imshow("mask2",mask2);
	//		waitKey(0);

	feature->operator ()(transform_image2, mask2, imageKeypoints,
			imageDescriptors);

	// フレームを飛ばす
	if (!f_center)
		for (int i = 0; i < FRAME_T; i++) {
			cap >> object;
			frame_num++;
		}

	// 手ブレ検出用各種変数 //
	int img_num = 0;
	stringstream ss; // 書き出しファイル名
	cv::Mat tmp_img; //
	cv::Mat sobel_img; // エッジ画像格納先


	vector<Mat> hist_image;

	int blur_skip;

	if (f_center)
		blur_skip = 0;
	else
		blur_skip = FRAME_T;

	Mat count(10, 10, CV_32F, cv::Scalar(0));
	while (frame_num + FRAME_T + 1 < end) {
		cap >> object;
		frame_num++;
		if (frame_num < 1600 && frame_num > 1500)
			while (frame_num < end) {
				printf("\nframe=%d\n", frame_num);

				cv::Laplacian(object, tmp_img, CV_32F, 1, 1);
				//Canny(cvarrToMat(objectc), sobel_img,50,100);
				cv::convertScaleAbs(tmp_img, sobel_img, 1, 0);

				// 縦横１０分割したエッジ画像の各ヒストグラムの領域確保
				for (int i = 0; i < 100; i++)
					hist_image.push_back(Mat(200, 260, CV_8U, cv::Scalar(255)));

				// ヒストグラム画像を作成
				get_histimage(sobel_img, hist_image.data(), count);

				/*
				 // 各ヒストグラムを順次表示
				 cvNamedWindow("Histogram", CV_WINDOW_AUTOSIZE);
				 for (int i = 0; i < 100; i++) {
				 imshow("Histogram", hist_image[i]);
				 waitKey(0);
				 }
				 */
				cout << "dev map : " << count << endl;
				imshow("check", sobel_img);
				int key = waitKey(0);
				if (key == 0x100020) {

					ss << "img/img_" << frame_num << ".jpg";
					std::cout << ss.str();
					imwrite(ss.str(), image);
					ss.clear();
					ss.str("");

					ss << "img/sobel_img_" << frame_num << ".jpg";
					std::cout << ss.str();
					imwrite(ss.str(), sobel_img);
					ss.clear();
					ss.str("");
					img_num++;

					std::cout << "skip frame : " << frame_num << std::endl;
					hist_image.clear();
					cap >> object;
					frame_num++;
				} else {
					break;
				}
			}

		cvtColor(object, gray_image, CV_RGB2GRAY);
		feature->operator ()(gray_image, Mat(), objectKeypoints,
				objectDescriptors);
		//		detector.detect(object, objectKeypoints);
		//		extractor.compute(object, objectKeypoints, objectDescriptors);

		cv::Laplacian(object, tmp_img, CV_32F, 1, 1);
		cv::convertScaleAbs(tmp_img, sobel_img, 1, 0);


		//matcher.match(objectDescriptors, imageDescriptors, matches);
		std::vector<std::vector<cv::DMatch> > matches12, matches21;
		int knn = 1;
		matcher.knnMatch(imageDescriptors, objectDescriptors, matches21, knn);
		matcher.knnMatch(objectDescriptors, imageDescriptors, matches12, knn);
		matches.clear();
		// KNN探索で，1->2と2->1が一致するものだけがマッチしたとみなされる
		for (size_t m = 0; m < matches12.size(); m++) {
			bool findCrossCheck = false;
			for (size_t fk = 0; fk < matches12[m].size(); fk++) {
				cv::DMatch forward = matches12[m][fk];
				for (size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++) {
					cv::DMatch backward = matches21[forward.trainIdx][bk];
					if (backward.trainIdx == forward.queryIdx) {
						matches.push_back(forward);
						findCrossCheck = true;
						break;
					}
				}
				if (findCrossCheck)
					break;
			}
		}
		cout << "matches : " << matches.size() << endl;

		double min_dist = DBL_MAX;
		for (int i = 0; i < (int) matches.size(); i++) {
			double dist = matches[i].distance;
			if (dist < min_dist)
				min_dist = dist;
		}

		cout << "min dist :" << min_dist << endl;

		good_objectKeypoints.clear();
		good_objectKeypoints.clear();
		good_matches.clear();
		pt1.clear();
		pt2.clear();
		for (int i = 0; i < (int) matches.size(); i++) {
			if (round(objectKeypoints[matches[i].queryIdx].class_id) == round(
					imageKeypoints[matches[i].trainIdx].class_id)) {

				if (matches[i].distance < min_dist * 3) {
					//		  &&	(fabs(objectKeypoints[matches[i].queryIdx].pt.y - imageKeypoints[matches[i].trainIdx].pt.y)
					//		/ fabs(objectKeypoints[matches[i].queryIdx].pt.x - 	imageKeypoints[matches[i].trainIdx].pt.x)) < 0.1) {

					good_matches.push_back(matches[i]);
					pt1.push_back(objectKeypoints[matches[i].queryIdx].pt);
					pt2.push_back(imageKeypoints[matches[i].trainIdx].pt);
					good_objectKeypoints.push_back(
							objectKeypoints[matches[i].queryIdx]);
					good_imageKeypoints.push_back(
							imageKeypoints[matches[i].trainIdx]);
				}
			}
		}

		cout << "selected good_matches : " << good_matches.size() << endl;

		// マッチング結果をリサイズして表示
		Mat result, r_result;
		cv::drawMatches(object, objectKeypoints, transform_image2,
				imageKeypoints, good_matches, result);
		resize(result, r_result, Size(), 0.5, 0.5, INTER_LANCZOS4);
		namedWindow("matches", CV_WINDOW_AUTOSIZE);
		imshow("matches", r_result);
		waitKey(30);

		// パノラマ平面の特徴点などは image
		// 取得したフレームの特徴点などは object
		// で固定なのでコピー作業はいらない
		//		imageKeypoints = objectKeypoints;
		//		objectDescriptors.copyTo(imageDescriptors);
		//		image = object.clone();

		if (f_comp && blur_skip != 0) {
			cout << "start comp" << endl;
			vector<Point2f> dist;
			vector<Point2f> est_pt1 = pt1, est_pt2;
			Mat est_h_base = h_base.clone();
			float inv_skip = 1.0 / (float) (blur_skip + 1);
			frame_num -= blur_skip;
			cap.set(CV_CAP_PROP_POS_FRAMES, frame_num - blur_skip);
			cout << "pt1 " << pt1[0] << endl;
			cout << "pt2 " << pt2[0] << endl;
			for (int k = 0; k < blur_skip; k++) {
				est_pt2.clear();

				for (int l = 0; l <pt1.size(); l++)
					est_pt2.push_back(est_pt1[l] + (pt2[l] - pt1[l]) * inv_skip);
				cout << "est_pt1 " << est_pt1[0] << endl;
				cout << "est_pt2 " << est_pt2[0] << endl;
				//waitKey(0);
				// 補完した点でホモグラフィ−行列を計算
				n = pt1.size() / 2;
				printf("n = %d\n", n);
				homography = findHomography(Mat(est_pt1), Mat(est_pt2),
						CV_RANSAC, 5.0);

				// パノラマ平面へのホモグラフィーを計算
				est_h_base = est_h_base * homography;
				//h_base = h_base * homography;

				// 飛ばしたフレームを取得しパノラマ平面へ投影


				cap >> object;
				warpPerspective(object, transform_image, est_h_base,
						object.size());
				Mat h2 = est_h_base;

				warpPerspective(white_img, pano_black, h2, white_img.size(),
						CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);

				// 特徴点をコピー
				est_pt1 = est_pt2;

				make_pano(transform_image, transform_image2, mask, pano_black);

				ss << "frame = " << frame_num;
				putText(transform_image, ss.str(), Point(100, 100),
						CV_FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255), 1,
						8);
				ss.clear();
				ss.str("");
				VideoWriter.write(transform_image);
				imshow("Object Correspond", transform_image2);
				frame_num++;
				waitKey(30);
				//cvWaitKey(0);

			}
			// 補完の際に上書きしているのでフレームを再取得
			cap >> object;

		}
		n = pt1.size();
		printf("n = %d\n", n);
		printf("num_of_obj = %d\n", good_objectKeypoints.size());
		printf("num_of_img = %d\n", good_imageKeypoints.size());
		if (n >= 4) {
			homography = findHomography(Mat(pt1), Mat(pt2), CV_RANSAC, 5.0);
		} else {
			setHomographyReset(&homography);
			printf("frame_num = %d\n", frame_num);
		}

		cv::Mat tmp = homography.clone();
		out_Hmat << tmp << endl;

		// パノラマ平面に対するホモグラフィーを直接求めているためh_baseにかけたりしなくて良い
		//h_base = h_base * homography;

		warpPerspective(object, transform_image, homography, Size(PANO_W,
				PANO_H));

		Mat h2 = homography;

		warpPerspective(white_img, pano_black, h2, Size(PANO_W, PANO_H),
				CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);

		make_pano(transform_image, transform_image2, mask, pano_black);

		ss << "frame = " << frame_num;
		putText(transform_image, ss.str(), Point(100, 100),
				CV_FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255), 1, 8);
		ss.clear();
		ss.str("");

		resize(transform_image, r_result, Size(), 0.75, 0.75, INTER_LINEAR);
		VideoWriter.write(r_result);
		imshow("Object Correspond", transform_image2);
		waitKey(30);
		erode(mask, mask2, cv::Mat(), cv::Point(-1, -1), 10);

		resize(transform_image, r_result, Size(), 0.75, 0.75, INTER_LINEAR);
		VideoWriter.write(r_result);
		imshow("Object Correspond", transform_image2);
		waitKey(30);
		erode(mask, mask2, cv::Mat(), cv::Point(-1, -1), 10);


		feature->operator ()(transform_image2, mask2, imageKeypoints,
				imageDescriptors);
		blur_skip = FRAME_T;
		for (int i = 0; i < FRAME_T; i++) {

			cap >> object;
			frame_num++;
		}
	}

	namedWindow("Object Correspond", CV_WINDOW_AUTOSIZE);
	imshow("Object Correspond", transform_image2);
	cvWaitKey(0);
	imwrite("transform4.jpg", transform_image2);

	return 0;
}
