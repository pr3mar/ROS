#include "svmExtended.hpp"

//using namespace cv;

SvmExtended::SvmExtended()
: CvSVM()
{
	// do nothing
}

SvmExtended::SvmExtended(const cv::Mat& trainData, const cv::Mat& responses, const cv::Mat& varIdx, const cv::Mat& sampleIdx, CvSVMParams params)
: CvSVM(trainData, responses, varIdx, sampleIdx, params)
{
	// do nothing
}

CvSVMDecisionFunc* SvmExtended::wrapper_decision_func() {
	return decision_func;
}

