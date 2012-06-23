#ifndef __PAIS_MVS_VIEWER_H__
#define __PAIS_MVS_VIEWER_H__

#define NAME_PATCH         "patch"
#define NAME_NORMAL        "normal"
#define NAME_PICKED_PATCH  "picked patch"
#define NAME_PICKED_NORMAL "picked normal"
#define NAME_VISIBLE_CAMERA_CENTER "visible camera center"
#define NAME_VISIBLE_CAMERA_NORMAL "visible camera normal"

// include PCL
#include <boost/thread/thread.hpp>
#include <pcl\common\common_headers.h>
#include <pcl\visualization\pcl_visualizer.h>

#include "../mvs/mvs.h"

using namespace pcl;
using namespace pcl::visualization;

namespace PAIS {
	// A wrapper class of PCL Visualizer
	class MvsViewer {
	private:
		// mvs const instance
		const MVS *mvs;
		PCLVisualizer pclViewer;

		PointCloud<pcl::PointXYZRGB>::Ptr centers;
		PointCloud<pcl::Normal>::Ptr normals;

		// viewer flags (default:false)
		// false/true
		bool normalEnable;    // hide normal       / show normal 
		bool backgroundColor; // black background  / white background 
		bool colorEnable;     // hide vertex color / show vertex color
		bool axes;

		void init();
		void addCameras();
		void addPatches();
	public:

		int pointSize;

		// constructor
		MvsViewer(const MVS &mvs, bool initPatch = true, bool show = false);
		// descructor
		~MvsViewer(void);

		void addPatch(const Patch &pth);

		// getter
		PCLVisualizer& getPclViewer()             { return pclViewer; }
		const PCLVisualizer& getPclViewer() const { return pclViewer; }

		// viewer params
		void toggleBackground();
		void toggleNormal();
		void toggleColor();
		void toggleAxes();
		const Patch* getPickedPatch(const int idx) const;
		void printPatchInformation(const Patch &pth) const;
		void showPickedPoint(const Patch &pth);
		void showVisibleCamera(const Patch &pth);

		// start rendering
		void open();
	};
}; 

void keyBoardEvent(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
void pointPickEvent(const pcl::visualization::PointPickingEvent &event, void* viewer_void);

#endif