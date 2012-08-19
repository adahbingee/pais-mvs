#ifndef __PAIS_FEATURE_MANAGER__
#define __PAIS_FEATURE_MANAGER__

#include <vector>
#include <opencv2\nonfree\nonfree.hpp>
#include "camera.h"

// using namespace PAIS;

namespace PAIS {
	class FeatureManager {
	public:
		static void getFeatureDescriptor(const vector<Camera> &cameras, const double maxDist);
	private:
		static void epipolarLineFiltering(const vector<KeyPoint> &queryKeypoints, const vector<KeyPoint> &trainKeypoints, const Mat_<double> &F,  const double maxDist, vector<DMatch> *matchesPtr);
		static void filteroutNonMatchViews(vector<vector<vector<DMatch> > > *matchTable);
		// get fundamental matrix xT'*F*xF = 0
		static Mat_<double> getFundamental(const Camera &camFrom, const Camera &camTo);
		// get fundamental matrices M(i,j) = Fij, where xi'*Fij*xj = 0;
		static void getFundamentalMatrices(const vector<Camera> &cameras, vector<vector<Mat_<double> > > *Fs);
	};
};

#endif