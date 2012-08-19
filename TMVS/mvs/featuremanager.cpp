#include "featuremanager.h"

extern ofstream debugFile;

void FeatureManager::getFeatureDescriptor(const vector<Camera> &cameras, const double maxDist) {
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
	vector<vector<vector<DMatch> > > matchTable(camNum, vector<vector<DMatch> >(camNum));
	for (int i = 0; i < camNum; ++i) {
		vector<DMatch> matches;
		BFMatcher matcher(NORM_L2);

		// match other views
		for (int j = 0; j < camNum; ++j) {
			if (i == j) continue; // skip self matching
			matches.clear();
			matcher.clear();
			matcher.match(descriptors[i], descriptors[j], matches);

			// fundamental matrix
			const Mat_<double> &F = Fs[i][j];

			epipolarLineFiltering(keypoints[i], keypoints[j], F, maxDist, &matches);

			// push into match table
			for (vector<DMatch>::const_iterator it = matches.begin(); it != matches.end(); it++) {
				const DMatch &match = *it;
				matchTable[i][j].push_back(match);
			}
		}
	}

	filteroutNonMatchViews(&matchTable);
}

void FeatureManager::epipolarLineFiltering(const vector<KeyPoint> &queryKeypoints, const vector<KeyPoint> &trainKeypoints, const Mat_<double> &F, const double maxDist, vector<DMatch> *matchesPtr) {
	vector<DMatch> &matches = *matchesPtr;

	// epipolar line filtering
	for (vector<DMatch>::const_iterator it = matches.begin(); it != matches.end();) {
		const DMatch &match = *it;

		// feature index
		int qidx = match.queryIdx;
		int tidx = match.trainIdx;
		// feature point
		const Point2f &qpt = queryKeypoints[qidx].pt;
		const Point2f &tpt = trainKeypoints[tidx].pt;

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

		// distance to epipolar line
		double dist = abs(distM.at<double>(0, 0)) / sqrt(epiLine.at<double>(0, 0)*epiLine.at<double>(0, 0) + epiLine.at<double>(0, 1)*epiLine.at<double>(0, 1));

		// filter out errornous match
		if (dist > maxDist) {
			it = matches.erase(it);
			continue;
		}

		++it;
	}
}

void FeatureManager::filteroutNonMatchViews(vector<vector<vector<DMatch> > > *matchTablePtr) {
	vector<vector<vector<DMatch> > > &matchTable = *matchTablePtr;
	for (int i = 0; i < (int) matchTable.size(); ++i) {
		// find maximum number match views
		double maxMatch = 0;
		for (int j = 0; j < (int) matchTable[i].size(); ++j) {
			if (matchTable[i][j].size() > maxMatch) {
				maxMatch = matchTable[i][j].size();
			}
		}

		// filter out non-match views
		for (int j = 0; j < (int) matchTable[i].size(); ++j) {
			if (matchTable[i][j].size() < maxMatch/4) {
				matchTable[i][j].clear();
			}
		}
	}
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