#include "featuremanager.h"

extern ofstream debugFile;

void FeatureManager::setSeedPatches(const vector<Camera> &cameras, const double maxDist, MVS *mvs) {
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
		Mat_<uchar> img;
		cvtColor(cameras[i].getRgbImage(), img, CV_RGB2GRAY);

		SIFT sift;
		sift(img, img, keypoints[i], descriptors[i]);
	}

	// nearest feature descriptor matching (matchTable[queryCamIDX][trainCamIDX])
	vector<vector<vector<DMatch> > > matchTable(camNum, vector<vector<DMatch> >(camNum));
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

			epipolarLineFiltering(keypoints[i], keypoints[j], F, maxDist, &matches);

			epipolarLineFilteringWithSupport(keypoints, Fs, maxDist, &matches, i, j, camNum);

			// push into match table
			for (vector<DMatch>::const_iterator it = matches.begin(); it != matches.end(); it++) {
				const DMatch &match = *it;
				matchTable[i][j].push_back(match);
			}
		}
	}

	filteroutNonMatches(&matchTable);

	// union matches
	vector<vector<NVMatch> > nvmatches;
	for (int i = 0; i < camNum; ++i) {
		for (int j = 0; j < camNum; ++j) {
			// skip self matching
			if ( i == j ) continue;
			// skip empty matching
			if ( matchTable[i][j].empty() ) continue;

			while ( !matchTable[i][j].empty() ) {
				const DMatch &match = matchTable[i][j].back();
				vector<NVMatch>* unionSeed = setNVMatch(i, j, match, nvmatches);
				
				if (unionSeed == NULL) { // push new union feature
					vector<NVMatch> seed(2);
					seed[0].camIdx = i;
					seed[0].featureIdx = match.queryIdx;
					seed[1].camIdx = j;
					seed[1].featureIdx = match.trainIdx;
					nvmatches.push_back(seed);
				}

				matchTable[i][j].pop_back();
			}
		}
	}

	// create seed patches
	for (vector<vector<NVMatch> >::const_iterator it = nvmatches.begin(); it != nvmatches.end(); ++it) {
		const vector<NVMatch> &match = *it;
		// skip few visible camera feature
		if (match.size() < mvs->minCamNum) continue;
		vector<int> camIdx;
		vector<Vec2d> imgPoint;
		for (vector<NVMatch>::const_iterator it = match.begin(); it != match.end(); ++it) {
			camIdx.push_back(it->camIdx);
			const Point2f &pt = keypoints[it->camIdx][it->featureIdx].pt;
			imgPoint.push_back(Vec2d(pt.x, pt.y));
		}
		Patch pth(Vec3d(0, 0, 0), Vec3b(128, 128, 128), camIdx, imgPoint);
		pth.reCentering();
		mvs->patches.insert(pair<int, Patch>(pth.getId(), pth));
	}

	return;
	// show n-view matches
	for (vector<vector<NVMatch> >::const_iterator it = nvmatches.begin(); it != nvmatches.end(); ++it) {
		const vector<NVMatch> &nvmatch =*it;
		for (vector<NVMatch>::const_iterator it2 = nvmatch.begin(); it2 != nvmatch.end(); ++it2) {
			Mat_<Vec3b> img = cameras[it2->camIdx].getRgbImage().clone();
			const Point2f &pt = keypoints[it2->camIdx][it2->featureIdx].pt;
			circle(img, pt, 3, Scalar(255, 255, 255), 3, CV_AA);
			circle(img, pt, 2, Scalar(0, 0, 0), 3, CV_AA);
			char title [30];
			sprintf(title, "image%d", it2->camIdx);
			imshow(title, img);
		}
		waitKey(0);
		destroyAllWindows();
	}
}

vector<NVMatch>* FeatureManager::setNVMatch(const int queryCamIdx, const int trainCamIdx, const DMatch &match, vector<vector<NVMatch> > &nvmatches) {
	vector<vector<NVMatch> >::iterator it;
	vector<NVMatch>::iterator it2, it3;

	// find union feature
	for (it = nvmatches.begin(); it != nvmatches.end(); ++it) {
		// union feature
		vector<NVMatch> &nvmatch = *it;
		// find link node
		for (it2 = nvmatch.begin(); it2 != nvmatch.end(); ++it2) {
			const NVMatch &mth = *it2;
			if (queryCamIdx == mth.camIdx && match.queryIdx == mth.featureIdx) {
				for (it3 = nvmatch.begin(); it3 != nvmatch.end(); ++it3) {
					if (it3->featureIdx == match.trainIdx) break;
				}
				if (it3 == nvmatch.end()) {
					NVMatch newMatch;
					newMatch.camIdx = trainCamIdx;
					newMatch.featureIdx = match.trainIdx;
					nvmatch.push_back(newMatch);
				}
				return &nvmatch;
			}
			if (trainCamIdx == mth.camIdx && match.trainIdx == mth.featureIdx) {
				for (it3 = nvmatch.begin(); it3 != nvmatch.end(); ++it3) {
					if (it3->featureIdx == match.queryIdx) break;
				}
				if (it3 == nvmatch.end()) {
					NVMatch newMatch;
					newMatch.camIdx = queryCamIdx;
					newMatch.featureIdx = match.queryIdx;
					nvmatch.push_back(newMatch);
				}
				return &nvmatch;
			} 
		} // end of image features
	} // end of union features
	return NULL;
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

// added by Chaody, 2012.Sep.15
// 只針對特定特徵點，再到其他view去找support
void FeatureManager::epipolarLineFilteringWithSupport
	(const vector<vector<KeyPoint> > &keypoints, const vector<vector<Mat_<double> > > &Fs, const double maxDist, vector<DMatch> *matchesPtr, const int iV1, const int iV2, const int camNum) 
{
	vector<DMatch> &matches = *matchesPtr;
	int k;

	// epipolar line filtering with support
	for (vector<DMatch>::const_iterator it = matches.begin(); it != matches.end();) {
		const DMatch &match = *it;
		int iSupCnt = 0;

		for (k=0; k<camNum; k++)
		{
			if (k == iV1) continue; // skip matching the first view in match table
			if (k == iV2) continue; // skip matching the second view in match table
					
			printf("iV1:%d iV2:%d k:%d camNum:%d\n", iV1, iV2, k, camNum);

			// feature index
			int qidx = match.queryIdx;
			int tidx = match.trainIdx;
			// feature point
			const Point2f &qptV1 = keypoints[iV1][qidx].pt;
			const Point2f &qptV2 = keypoints[iV2][tidx].pt;
			const int keyNum = (int) keypoints[k].size();

			for (int kIdx = 0; kIdx < keyNum; kIdx++) {
				double dist_V1=65535.0, dist_V2=65535.0;
				const Point2f &tpt = keypoints[k][kIdx].pt;

				Mat_<double> qM(3, 1);
				Mat_<double> tM(3, 1);
				qM.at<double>(0, 0) = qptV1.x;
				qM.at<double>(1, 0) = qptV1.y;
				qM.at<double>(2, 0) = 1.0;
				tM.at<double>(0, 0) = tpt.x;
				tM.at<double>(1, 0) = tpt.y;
				tM.at<double>(2, 0) = 1.0;

				Mat_<double> epiLine = qM.t()*Fs[iV1][k];
				Mat_<double> distM = epiLine * tM;

				// distance to epipolar line
				dist_V1 = abs(distM.at<double>(0, 0)) / sqrt(epiLine.at<double>(0, 0)*epiLine.at<double>(0, 0) + epiLine.at<double>(0, 1)*epiLine.at<double>(0, 1));
				
				// comfirm another iV2 
				if (dist_V1 < maxDist) {
					qM.at<double>(0, 0) = qptV2.x;
					qM.at<double>(1, 0) = qptV2.y;
					qM.at<double>(2, 0) = 1.0;

					Mat_<double> epiLine = qM.t()*Fs[iV2][k];
					Mat_<double> distM = epiLine * tM;

					// distance to epipolar line
					dist_V2 = abs(distM.at<double>(0, 0)) / sqrt(epiLine.at<double>(0, 0)*epiLine.at<double>(0, 0) + epiLine.at<double>(0, 1)*epiLine.at<double>(0, 1));						
				}

				// only points pass two view constraint, support count+1
				if (dist_V2 < maxDist) {
					iSupCnt++;
					printf("iV1:%d iV2:%d k:%d dist:%f iSUpCnt:%d\n", iV1, iV2, k, dist_V2, iSupCnt);
				}
			}
		}
				
		// filter out errornous match
		if (iSupCnt < 2) {
			it = matches.erase(it);
			continue;
		}
		
		++it;
	}
}

void FeatureManager::filteroutNonMatches(vector<vector<vector<DMatch> > > *matchTablePtr) {

	vector<vector<vector<DMatch> > > &matchTable = *matchTablePtr;

	// filter out non-cross matching and delete cross-matching redundance
	for (int i = 0; i < (int) matchTable.size(); ++i) {
		for (int j = 0; j < (int) matchTable[i].size(); ++j) {
			vector<DMatch>::iterator it;
			vector<DMatch>::iterator it2;
			for (it = matchTable[i][j].begin(); it != matchTable[i][j].end();) {
				DMatch &match = *it;
				bool crossMatch = false;

				for (it2 = matchTable[j][i].begin(); it2 != matchTable[j][i].end(); ++it2) {
					DMatch &match2 = *it2;
					// cross matching
					if (match.trainIdx == match2.queryIdx && match.queryIdx == match2.trainIdx) {
						crossMatch = true;
						// remove redundance
						matchTable[j][i].erase(it2);
						break;
					}
				}
				// delete non-cross matching
				if (!crossMatch) {
					it = matchTable[i][j].erase(it);
					continue ;
				}
				++it;
			}
		}
	}

	// filter out few feature views
	for (int i = 0; i < (int) matchTable.size(); ++i) {
		// find maximum number match views
		int maxMatch = 0;
		for (int j = 0; j < (int) matchTable[i].size(); ++j) {
			if (matchTable[i][j].size() > maxMatch) {
				maxMatch = matchTable[i][j].size();
			}
		}

		// filter out non-match views
		for (int j = 0; j < (int) matchTable[i].size(); ++j) {
			if (matchTable[i][j].size() < maxMatch/4.0) {
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