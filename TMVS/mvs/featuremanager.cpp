#include "featuremanager.h"

extern ofstream debugFile;

struct NViewMatch {
	int queryDescIdx;
	int trainDescIdx;
	int queryCamIdx;
	int trainCamIdx;
};

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

	// nearest feature descriptor matching
	vector<vector<NViewMatch> > correctMatches(camNum);
	for (int i = 0; i < camNum; ++i) {
		vector<DMatch> matches;
		BFMatcher matcher(NORM_L2, true);
		// match other views
		for (int j = 0; j < camNum; ++j) {
			if (i == j) continue; // skip self matching
			matches.clear();
			matcher.clear();
			matcher.match(descriptors[i], descriptors[j], matches);

			// fundamental matrix
			const Mat_<double> &F = Fs[i][j];

			// epipolar line filtering
			for (int k = 0; k < (int) matches.size(); ++k) {
				const DMatch &match = matches[k];

				// feature index
				int qidx = match.queryIdx;
				int tidx = match.trainIdx;
				// feature point
				const Point2f &qpt = keypoints[i][qidx].pt;
				const Point2f &tpt = keypoints[j][tidx].pt;

				Mat_<double> qM(3, 1);
				Mat_<double> tM(3, 1);
				qM.at<double>(0, 0) = qpt.x;
				qM.at<double>(1, 0) = qpt.y;
				qM.at<double>(2, 0) = 1.0;
				tM.at<double>(0, 0) = tpt.x;
				tM.at<double>(1, 0) = tpt.y;
				tM.at<double>(2, 0) = 1.0;

				Mat_<double> epiLine = qM.t()*F;
				Mat_<double> distM = epiLine * tM;
				double dist = abs(distM.at<double>(0, 0)) / sqrt(epiLine.at<double>(0, 0)*epiLine.at<double>(0, 0) + epiLine.at<double>(0, 1)*epiLine.at<double>(0, 1));

				if (dist <= 1.0) {
					NViewMatch nvm;
					nvm.queryCamIdx = i;
					nvm.trainCamIdx = j;
					nvm.queryDescIdx = match.queryIdx;
					nvm.trainDescIdx = match.trainIdx;
					correctMatches[i].push_back(nvm);
				}
			}
		}
	}

	for (int i = 0; i < (int) correctMatches.size(); ++i) {
		
	}

	/*
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

	*/
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