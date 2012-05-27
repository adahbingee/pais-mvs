#include "mvsviewer.h"

void keyBoardEvent(const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
	if (!event.keyUp()) return;

	MvsViewer &viewer = *((MvsViewer *) viewer_void);
	
	switch (event.getKeyCode()) {
	case 'B':
	case 'b':
		viewer.toggleBackground();
		break;
	}

}

void pointPickEvent(const pcl::visualization::PointPickingEvent &event, void* viewer_void) {
	
}

MvsViewer::MvsViewer(const MVS &mvs, bool show) {
	// set instance holder
	this->mvs = &mvs;

	// initialize viewer
	init();

	// add MVS cameras
	addCameras();
	// add MVS patches
	addPatches("MVS Patches");

	if (show) open();
}

MvsViewer::~MvsViewer(void) {
	
}

void MvsViewer::init() {
	// set viewer
	pclViewer.setBackgroundColor(0, 0, 0);
	pclViewer.initCameraParameters();
	pclViewer.addCoordinateSystem (0.1);

	// set event listener
	pclViewer.registerKeyboardCallback(keyBoardEvent, (void*) this);
	pclViewer.registerPointPickingCallback(pointPickEvent, (void*) this);

	// set viewer flags
	normalEnable    = false;
	backgroundColor = false;
	colorEnable     = false;
}

void MvsViewer::addCameras() {
	const vector<Camera> &cameras = mvs->getCameras();
	const int camNum = (int) cameras.size();

	// camera position container
    PointCloud<PointXYZ>::Ptr centers(new PointCloud<PointXYZ>);
    // camera normal container
    PointCloud<pcl::Normal>::Ptr normals(new PointCloud<pcl::Normal>);

	// fill containers
	for (int i = 0; i < camNum; i++) { 
		const Camera &cam = cameras[i];
		const Vec3d  &c   = cam.getCenter();
		const Vec3d  &n   = cam.getOpticalNormal();

		PointXYZ    pt(c[0], c[1], c[2]);
        pcl::Normal nt(n[0], n[1], n[2]);

		centers->push_back(pt);
		normals->push_back(nt);
	}

	// remove old points
	pclViewer.removePointCloud("cameras");
	pclViewer.removePointCloud("cameraNormals");

	// add new points
	pclViewer.addPointCloud(centers, "cameras");
	pclViewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(centers, normals, 1, 0.1, "cameraNormals");

	// set camera visualize properities
	pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cameras");
    pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cameras");
    pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "cameraNormals");
    pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cameraNormals");
}

void MvsViewer::addPatches(const char *name) {
	const map<int, Patch> &patches = mvs->getPatches();
	map<int, Patch>::const_iterator it;

	PointCloud<PointXYZRGB>::Ptr centers(new PointCloud<PointXYZRGB>);
	PointCloud<pcl::Normal>::Ptr normals(new PointCloud<pcl::Normal>);

	for (it = patches.begin(); it != patches.end(); ++it) {
		// patch center
		const Vec3d &p = it->second.getCenter();
		// patch normal
		const Vec3d &n = it->second.getNormal();
		// patch color
		const Vec3b &c = it->second.getColor();
		
		
		PointXYZRGB pt;
		pt.x = p[0];
		pt.y = p[1];
		pt.z = p[2];
		pt.r = c[0];
		pt.g = c[1];
		pt.b = c[2];
		pcl::Normal nt(n[0], n[1], n[2]);

		centers->push_back(pt);
		normals->push_back(nt);
	}

	// remove old points
    pclViewer.removePointCloud(name);
	pclViewer.removePointCloud(string(name).append("normals"));

	// add new points
    pclViewer.addPointCloud(centers, name);
	pclViewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(centers, normals, 1, 0.1, string(name).append("normals"));
	
	// set normal color
	pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, string(name).append("normals"));
    pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, string(name).append("normals"));
}

void MvsViewer::open() {
	while ( !pclViewer.wasStopped() ) {
		pclViewer.spin();
	}
}

void MvsViewer::toggleBackground() {
	if (backgroundColor) {
		pclViewer.setBackgroundColor(0, 0, 0);
		backgroundColor = false;
	} else {
		pclViewer.setBackgroundColor(1, 1, 1);
		backgroundColor = true;
	}

	pclViewer.spinOnce(1, true);
}