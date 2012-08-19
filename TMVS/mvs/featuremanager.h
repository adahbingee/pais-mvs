#ifndef __PAIS_FEATURE_MANAGER__
#define __PAIS_FEATURE_MANAGER__

#include <vector>
#include <opencv2\nonfree\nonfree.hpp>
#include "camera.h"

// using namespace PAIS;

namespace PAIS {
	struct NVMatch {
		int camIdx;
		int featureIdx;
	};

	class FeatureManager {
	public:
		static void setSeedPatches(const vector<Camera> &cameras, const double maxDist, MVS *mvs);
	private:
		static vector<NVMatch>* setNVMatch(const int queryCamIdx, const int trainCamIdx, const DMatch &match, vector<vector<NVMatch> > &nvmatches);
		static void epipolarLineFiltering(const vector<KeyPoint> &queryKeypoints, const vector<KeyPoint> &trainKeypoints, const Mat_<double> &F,  const double maxDist, vector<DMatch> *matchesPtr);
		static void filteroutNonMatches(vector<vector<vector<DMatch> > > *matchTable);
		// get fundamental matrix xT'*F*xF = 0
		static Mat_<double> getFundamental(const Camera &camFrom, const Camera &camTo);
		// get fundamental matrices M(i,j) = Fij, where xi'*Fij*xj = 0;
		static void getFundamentalMatrices(const vector<Camera> &cameras, vector<vector<Mat_<double> > > *Fs);
	};
};

#endif