#include "featuremanager.h"

extern ofstream debugFile;

void FeatureManager::getFeatureDescriptor(const vector<Camera> &cameras) {
	const int camNum = (int) cameras.size();

	// keypoints container
	vector<vector<KeyPoint> > keypoints(camNum);
	// descriptor container
	vector<Mat> descriptors(camNum);
	// fundamental matrices container M(i,j) = Fij, where xi'*Fij*xj = 0;
	vector<vector<Mat_<double> > > Fs;

	// get cross cameras fundamental matrices
	getFundamentalMatrices(cameras, &Fs);

	// get SIFT feature keypoints and descriptors
	#pragma omp parallel for
	for (int i = 0; i < cameras.size(); ++i) {
		SIFT sift;
		sift(cameras[i].getPyramidImage(0), cameras[i].getPyramidImage(0), keypoints[i], descriptors[i]);
	}

	// knn feature descriptor matching
	const int k = 3;
	vector<vector<DMatch> > knnMatches;
	Ptr<DescriptorMatcher> matcherPtr = DescriptorMatcher::create("BruteForce");
	matcherPtr->knnMatch(descriptors[0], descriptors[1], knnMatches, k);

	for (int i = 0; i < knnMatches.size(); ++i) {
		const vector<DMatch> &matches = knnMatches[i];
		
		double minDist = DBL_MAX;
		int minK;
		const DMatch *minMatch;

		for (int j = 0; j < k; ++j) {
			const DMatch &match = matches[j];
			
			// epipolar line checking
			int pIdx1 = match.queryIdx; // img 0
			int pIDx2 = match.trainIdx; // img 1
			const Mat_<double> &F = Fs[0][1];

			const Point2f pt1 = keypoints[0][pIdx1].pt;
			const Point2f pt2 = keypoints[1][pIDx2].pt;

			Mat_<double> p1(3, 1);
			Mat_<double> p2(3, 1);
			p1.at<double>(0, 0) = pt1.x;
			p1.at<double>(1, 0) = pt1.y;
			p1.at<double>(2, 0) = 1.0;
			p2.at<double>(0, 0) = pt2.x;
			p2.at<double>(1, 0) = pt2.y;
			p2.at<double>(2, 0) = 1.0;

			Mat_<double> epiLine = p1.t()*F; 

			Mat_<double> distM = epiLine * p2;
			double dist = abs(distM.at<double>(0, 0)) / sqrt(epiLine.at<double>(0, 0)*epiLine.at<double>(0, 0) + epiLine.at<double>(0, 1)*epiLine.at<double>(0, 1));

			if (dist < minDist) {
				minDist = dist;
				minMatch = &match;
				minK = j;
			} 
		} // end of knn

		if (minMatch == NULL || minDist > 1 || minK != 0) continue;

		const DMatch &match = *minMatch;
			
		// epipolar line checking
		int pIdx1 = match.queryIdx; // img 0
		int pIDx2 = match.trainIdx; // img 1
		const Mat_<double> &F = Fs[0][1];

		const Point2f pt1 = keypoints[0][pIdx1].pt;
		const Point2f pt2 = keypoints[1][pIDx2].pt;

		Mat img1 = cameras[0].getRgbImage().clone();
		Mat img2 = cameras[1].getRgbImage().clone();
		circle(img1, pt1, 3, Scalar(0,0,255), 3);
		circle(img2, pt2, 3, Scalar(0,0,255), 3);
		printf("epipolar line distance: %f \t knn: %d\n", minDist, minK);
		imshow("img1", img1);
		imshow("img2", img2);
		waitKey();

	} // end of matches
}

Mat_<double> FeatureManager::getFundamental(const Camera &camFrom, const Camera &camTo) {
	// F = eTx * pT * pF^-1;
	// eT = pT * C;
	const Mat &pF   = camFrom.getP();
	const Mat &pT   = camTo.getP();
	const Vec3d &cF = camFrom.getCenter();
	double cFData [] = {cF[0], cF[1], cF[2], 1.0};
	const Mat cFM(4, 1, CV_64FC1, cFData);
	const Mat eT = pT * cFM;
	// essential matrix
	double exTData [] = {0, -eT.at<double>(2, 0), eT.at<double>(1, 0),
				         eT.at<double>(2, 0), 0, -eT.at<double>(0, 0),
					    -eT.at<double>(1, 0), eT.at<double>(0, 0), 0};
	const Mat exT(3, 3, CV_64FC1, exTData);
	const Mat pFinv = pF.inv(DECOMP_SVD);
	return exT*pT*pFinv;
}

// get fundamental matrices M(i,j) = Fij, where xi'*Fij*xj = 0;
void FeatureManager::getFundamentalMatrices(const vector<Camera> &cameras, vector<vector<Mat_<double> > > *Fs) {
	// number of cameras
	const int num = (int) cameras.size();

	// fundamental matrices container
	vector<vector<Mat_<double> > > &M = *Fs;
	M.resize(num, vector<Mat_<double> >(num));

	for (int i = 0; i < num; i++) {
		for (int j = i; j < num; j++) {
			if (i == j) {
				M[i][j] = Mat::eye(3, 3, CV_64FC1);
			} else {
				const Camera &camFrom = cameras[j];
				const Camera &camTo = cameras[i];
				M[i][j] = getFundamental(camFrom, camTo);
				M[j][i] = M[i][j].t();
			}
		}
	}
}