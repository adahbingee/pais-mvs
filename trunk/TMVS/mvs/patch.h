#ifndef __PAIS_PATCH__H__
#define __PAIS_PATCH__H__

#include "../pso/psosolver.h"
#include "abstractpatch.h"
#include "mvs.h"

using namespace PAIS;
using namespace cv;

namespace PAIS {
	class MVS;

	class Patch : public AbstractPatch {
	private:
		static const int TYPE_SEED   = 0x0;
		static const int TYPE_EXPAND = 0x1;
		bool drop;
		int type;

		void setCorrelationTable(const vector<Mat_<double>> &H);
		// get homography texture 1D vector
		void getHomographyPatch(const Vec2d &pt, const Mat_<uchar> &img, const Mat_<double> &H, Mat_<double> &hp);
		// expand visible camera using normal correlation
		void expandVisibleCamera();
		// do pso optimization 
		void psoOptimization();

	protected:
		void setEstimatedNormal();
		void setReferenceCameraIndex();
		void setDepthAndRay();
		void setDepthRange();
		void setLOD();
		void setPriority();
		void setImagePoint();
	
	public:
		static bool isNeighbor(const Patch &pth1, const Patch &pth2);
		
		// seed patch constructor
		Patch(const Vec3d &center, const Vec3b &color, const vector<int> &camIdx, const vector<Vec2d> &imgPoint, const int id = -1);
		// expansion patch constructor
		Patch(const Vec3d &center, const Patch &parent, const int id = -1);
		// mvs loader constructor
		Patch(const Vec3d &center, const Vec2d &normalS, const vector<int> &camIdx, const double fitness, const double correlation, const int id = -1);

		void reCentering();
		void refine();
		void removeInvisibleCamera();
		void setQuantization(const Vec3d &center, const Vec3d &normal);

		// get homographies
		void getHomographies(const Vec3d &center, const Vec3d &normal, vector<Mat_<double>> &H) const;
		// get homography region ratio
		double getHomographyRegionRatio(const Vec2d &pt, const Mat_<double> &H) const;
		// show homography window in visible cameras
		void showRefinedResult() const;
		// show SAD error image
		void showError() const;
		// is dropped
		bool isDropped() const { return drop; }
		~Patch(void);
	};

	double getFitness(const Particle &p, void *obj);
};

#endif