/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap/core/stereo/StereoSGBM.h>
#include <rtabmap/utilite/ULogger.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

namespace rtabmap {

StereoSGBM::StereoSGBM(const ParametersMap & parameters) :
		StereoDense(parameters),
		blockSize_(Parameters::defaultStereoSGBMBlockSize()),
		minDisparity_(Parameters::defaultStereoSGBMMinDisparity()),
		numDisparities_(Parameters::defaultStereoSGBMNumDisparities()),
		preFilterCap_(Parameters::defaultStereoSGBMPreFilterCap()),
		uniquenessRatio_(Parameters::defaultStereoSGBMUniquenessRatio()),
		speckleWindowSize_(Parameters::defaultStereoSGBMSpeckleWindowSize()),
		speckleRange_(Parameters::defaultStereoSGBMSpeckleRange()),
		P1_(Parameters::defaultStereoSGBMP1()),
		P2_(Parameters::defaultStereoSGBMP2()),
		disp12MaxDiff_(Parameters::defaultStereoSGBMDisp12MaxDiff()),
		mode_(Parameters::defaultStereoSGBMMode())
{
	this->parseParameters(parameters);
}

void StereoSGBM::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kStereoSGBMBlockSize(), blockSize_);
	Parameters::parse(parameters, Parameters::kStereoSGBMMinDisparity(), minDisparity_);
	Parameters::parse(parameters, Parameters::kStereoSGBMNumDisparities(), numDisparities_);
	Parameters::parse(parameters, Parameters::kStereoSGBMPreFilterCap(), preFilterCap_);
	Parameters::parse(parameters, Parameters::kStereoSGBMUniquenessRatio(), uniquenessRatio_);
	Parameters::parse(parameters, Parameters::kStereoSGBMSpeckleWindowSize(), speckleWindowSize_);
	Parameters::parse(parameters, Parameters::kStereoSGBMSpeckleRange(), speckleRange_);
	Parameters::parse(parameters, Parameters::kStereoSGBMP1(), P1_);
	Parameters::parse(parameters, Parameters::kStereoSGBMP2(), P2_);
	Parameters::parse(parameters, Parameters::kStereoSGBMDisp12MaxDiff(), disp12MaxDiff_);
	Parameters::parse(parameters, Parameters::kStereoSGBMMode(), mode_);
}

cv::Mat StereoSGBM::computeDisparity(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage) const
{
	UASSERT(!leftImage.empty() && !rightImage.empty());
	UASSERT(leftImage.cols == rightImage.cols && leftImage.rows == rightImage.rows);
	UASSERT(leftImage.type() == CV_8UC1 || leftImage.type() == CV_8UC3);
	UASSERT(rightImage.type() == CV_8UC1 || rightImage.type() == CV_8UC3);

	cv::Mat leftMono;
	if(leftImage.channels() == 3)
	{
		cv::cvtColor(leftImage, leftMono, CV_BGR2GRAY);
	}
	else
	{
		leftMono = leftImage;
	}

	cv::Mat rightMono;
	if(rightImage.channels() == 3)
	{
		cv::cvtColor(rightImage, rightMono, CV_BGR2GRAY);
	}
	else
	{
		rightMono = rightImage;
	}

	cv::Mat disparity;
#if CV_MAJOR_VERSION < 3
	cv::StereoSGBM stereo(
			minDisparity_,
			numDisparities_,
			blockSize_,
            P1_,
			P2_,
			disp12MaxDiff_,
			preFilterCap_,
			uniquenessRatio_,
			speckleWindowSize_,
			speckleRange_,
            mode_==1);
	stereo(leftMono, rightMono, disparity);
#else
	cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
			minDisparity_,
			numDisparities_,
			blockSize_,
            P1_,
			P2_,
			disp12MaxDiff_,
			preFilterCap_,
			uniquenessRatio_,
			speckleWindowSize_,
			speckleRange_,
            mode_);
	stereo->compute(leftMono, rightMono, disparity);
#endif

	if(minDisparity_>0)
	{
		cv::Mat dst;
		cv::threshold(disparity, dst, minDisparity_*16, 0, cv::THRESH_TOZERO);
		disparity = dst;
	}

	return disparity;
}

} /* namespace rtabmap */
