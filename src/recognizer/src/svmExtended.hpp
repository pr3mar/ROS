#ifndef SVM_EXTEND
#define SVM_EXTEND

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/ml/ml.hpp>


class SvmExtended : public CvSVM {

public:
	SvmExtended();
	SvmExtended(const cv::Mat& trainData, const cv::Mat& responses, const cv::Mat& varIdx=cv::Mat(), const cv::Mat& sampleIdx=cv::Mat(), CvSVMParams params=CvSVMParams());
	CvSVMDecisionFunc* wrapper_decision_func();
};

#endif