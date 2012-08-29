#include "mvsviewer.h"

void keyBoardEvent(const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
	if (!event.keyUp()) return;

	MvsViewer &viewer = *((MvsViewer *) viewer_void);

	// printf("key %s %d\n", event.getKeySym(), event.getKeyCode());

	if (event.getKeyCode() == 0) {
		if (event.getKeySym().compare("Prior") == 0) {
			viewer.reduceNormalLevel();
		} else if (event.getKeySym().compare("Next") == 0) {
			viewer.addNormalLevel();
		}
		return;
	}

	switch (event.getKeyCode()) {
	case 'H':
	case 'h':
		printf("\tA, a toggle point color\n");
		printf("\tB, b toggle background color\n");
		printf("\tN, n toggle normal display\n");
		printf("\tD, d toggle axes display\n");
		printf("\tshift+W, shift+w toggle cameras display\n");
		printf("\tshift+S, shift+s optimize by patch id\n");
		printf("\tpageup, pagedown add/reduce normal density\n");
		break;
	case 'A':
	case 'a':
		viewer.toggleColor();
		break;
	case 'B':
	case 'b':
		viewer.toggleBackground();
		break;
	case 'N':
	case 'n':
		viewer.toggleNormal();
		break;
	case '+':
		viewer.addPointSize();
		break;
	case '-':
		viewer.reducePointSize();
		break;
	case 'D':
	case 'd':
		viewer.toggleAxes();
		break;
	case 'W':
	case 'w':
		viewer.toggleCameras();
		break;
	case 's':
	case 'S':
		if ( event.isShiftPressed() ) {
			int pthId = -1;
			printf("input patch id to be optimized: ");
			scanf("%d", &pthId);
			Patch *pth = const_cast<Patch*> (viewer.getPickedPatch(pthId));
			if (pth == NULL) break;
			pth->refine();
			cvDestroyAllWindows();
			viewer.printPatchInformation(*pth);
			viewer.showPickedPoint(*pth);
			viewer.showVisibleCamera(*pth);
			waitKey(1);
		}
		break;
	}
}

void pointPickEvent(const pcl::visualization::PointPickingEvent &event, void* viewer_void) {
	MvsViewer &viewer = *((MvsViewer *) viewer_void);
	viewer.selectedPatchId = event.getPointIndex();
	// get patch
	const Patch *pth = viewer.getPickedPatch(viewer.selectedPatchId);
	if (pth == NULL) return;

	cvDestroyAllWindows();
	viewer.printPatchInformation(*pth);
	viewer.showPickedPoint(*pth);
	viewer.showVisibleCamera(*pth);
	waitKey(1);
}

MvsViewer::MvsViewer(const MVS &mvs, bool initPatch, bool show, bool animate) {
	// set instance holder
	this->mvs = &mvs;

	this->animate = animate;

	// initialize viewer
	init();

	// add MVS cameras
	addCameras();

	// add patches
	if (initPatch) {
		if ( !animate ) {
			// add MVS patches show final result
			addPatches();
		} else {
			// add MVS patches show growing result
			addPatchesAnimate();
		}
	}

	if (show) open();
}

MvsViewer::~MvsViewer(void) {
	
}

void MvsViewer::init() {
	pointSize    = 1;
	normalLevel  = 1;
	normalLength = getNormalLength();

	// set container
	centers = PointCloud<PointXYZRGB>::Ptr (new PointCloud<PointXYZRGB>);
	normals = PointCloud<Normal>::Ptr (new PointCloud<Normal>);

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
	colorEnable     = true;
	axes            = true;
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
	pclViewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(centers, normals, 1, normalLength, "cameraNormals");

	// set camera visualize properities
	pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cameras");
    pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cameras");
    pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "cameraNormals");
    pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cameraNormals");
}

void MvsViewer::addPatch(const Patch &pth) {
	// patch center
	const Vec3d &p = pth.getCenter();
	// patch normal
	const Vec3d &n = pth.getNormal();
	// patch color
	const Vec3b &c = pth.getColor();

	PointXYZRGB pt;
	pt.x = p[0];
	pt.y = p[1];
	pt.z = p[2];
	pt.r = c[2];
	pt.g = c[1];
	pt.b = c[0];
	pcl::Normal nt(n[0], n[1], n[2]);

	centers->push_back(pt);
	normals->push_back(nt);

	if ( !pclViewer.updatePointCloud(centers, NAME_PATCH) ) {
		pclViewer.addPointCloud(centers, NAME_PATCH);
	}

	pclViewer.removePointCloud(NAME_NORMAL);
	pclViewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(centers, normals, 1, normalLength, NAME_NORMAL);

	// set normal color
	pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, NAME_NORMAL);
	pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, NAME_NORMAL);

	showPickedPoint(pth);
	showVisibleCamera(pth);

	pclViewer.spinOnce(1);
}

void MvsViewer::addPatches() {
	const map<int, Patch> &patches = mvs->getPatches();
	map<int, Patch>::const_iterator it;

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
		pt.r = c[2];
		pt.g = c[1];
		pt.b = c[0];
		Normal nt(n[0], n[1], n[2]);

		centers->push_back(pt);
		normals->push_back(nt);
	}

	// remove old points
	pclViewer.removePointCloud(NAME_PATCH);
	pclViewer.removePointCloud(NAME_NORMAL);

	// add new points
    pclViewer.addPointCloud(centers, NAME_PATCH);
	pclViewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(centers, normals, normalLevel, normalLength, NAME_NORMAL);
	
	// set normal color
	pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, NAME_NORMAL);
    pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, NAME_NORMAL);

	pclViewer.resetCamera();
}

void MvsViewer::addPatchesAnimate() {
	const map<int, Patch> &patches = mvs->getPatches();
	const int pthNum = (int) mvs->getPatches().size();
	map<int, Patch>::const_iterator it;
	for (it = patches.begin(); it != patches.end(); ++it) {
		addPatch(it->second);
	}
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

void MvsViewer::toggleNormal() {
	if (normalEnable) {
		pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, NAME_NORMAL);
		normalEnable = false;
	} else {
		pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, NAME_NORMAL);
		normalEnable = true;
	}

	pclViewer.spinOnce(1);
}

void MvsViewer::toggleColor() {
	if (colorEnable) {
		pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0, 0, NAME_PATCH);
		colorEnable = false;
	} else {
		addPatches();
		colorEnable = true;
	}

	pclViewer.spinOnce(1);
}

void MvsViewer::toggleAxes() {
	if (axes) {
		pclViewer.removeCoordinateSystem();
		axes = false;
	} else {
		pclViewer.addCoordinateSystem (0.1);
		axes = true;
	}
	pclViewer.spinOnce(1);
}

void MvsViewer::toggleCameras() {

}

void MvsViewer::addPointSize() {
	pointSize = min(pointSize+1, 10);
	pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, (double) pointSize, NAME_PATCH);
}

void MvsViewer::reducePointSize() {
	pointSize = max(pointSize-1, 1);
	pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, (double) pointSize, NAME_PATCH);
}

void MvsViewer::addNormalLevel() {
	normalLevel = min(normalLevel+10, 100);

	// update normal
	pclViewer.removePointCloud(NAME_NORMAL);
	pclViewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(centers, normals, normalLevel, normalLength, NAME_NORMAL);
	pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, NAME_NORMAL);
    pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, NAME_NORMAL);

	pclViewer.spinOnce(1);
}

void MvsViewer::reduceNormalLevel() {
	normalLevel = max(normalLevel-10, 1);

	// update normal
	pclViewer.removePointCloud(NAME_NORMAL);
	pclViewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(centers, normals, normalLevel, normalLength, NAME_NORMAL);
	pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, NAME_NORMAL);
    pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, NAME_NORMAL);

	pclViewer.spinOnce(1);
}

const Patch* MvsViewer::getPickedPatch(const int idx) const {
	// skip out of index boundary
	if (idx > mvs->getPatches().size() || idx < 0) {
		return NULL;
	}

	map<int, Patch>::const_iterator it;
	it = mvs->getPatches().begin();
	for (int i = 0; i < idx; ++it, ++i);

	return &it->second;
}

void MvsViewer::showPickedPoint(const Patch &pth) {
	// patch center
	const Vec3d &p = pth.getCenter();
	// patch normal
	const Vec3d &n = pth.getNormal();

	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<pcl::Normal>::Ptr normal(new PointCloud<pcl::Normal>);

	PointXYZ pt;
	pt.x = p[0];
	pt.y = p[1];
	pt.z = p[2];
	pcl::Normal nt(n[0], n[1], n[2]);

	cloud->push_back(pt);
	normal->push_back(nt);

	// remove old points
	pclViewer.removePointCloud(NAME_PICKED_PATCH);
	pclViewer.removePointCloud(NAME_PICKED_NORMAL);

	// add new points
    pclViewer.addPointCloud(cloud, NAME_PICKED_PATCH);
	// add point normals
	pclViewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normal, 1, normalLength, NAME_PICKED_NORMAL);
	// set color
    pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, NAME_PICKED_PATCH);
	// set point size
	pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, NAME_PICKED_PATCH);
}

void MvsViewer::showVisibleCamera(const Patch &pth) {
	// visible camera
	PointCloud<PointXYZRGB>::Ptr cameraCenter(new PointCloud<PointXYZRGB>);
	PointCloud<pcl::Normal>::Ptr cameraNormal(new PointCloud<pcl::Normal>);
	for (int i = 0; i < pth.getCameraNumber(); ++i) {
		const int camIdx = pth.getCameraIndices()[i];
		const Camera &cam = mvs->getCamera(camIdx);

		PointXYZRGB pt;
		pt.x = cam.getCenter()[0];
		pt.y = cam.getCenter()[1];
		pt.z = cam.getCenter()[2];
		// set camera point color 
		if ( camIdx == pth.getReferenceCameraIndex() ) {
			pt.r = 255;
			pt.g = 255;
			pt.b = 255;
		} else {
			pt.r = 0;
			pt.g = 255;
			pt.b = 0;
		}
		
		pcl::Normal nt(cam.getOpticalNormal()[0], cam.getOpticalNormal()[1], cam.getOpticalNormal()[2]);
		cameraCenter->push_back(pt);
		cameraNormal->push_back(nt);
	}

	pclViewer.removePointCloud(NAME_VISIBLE_CAMERA_CENTER);
	pclViewer.removePointCloud(NAME_VISIBLE_CAMERA_NORMAL);
	// add new points
    pclViewer.addPointCloud(cameraCenter, NAME_VISIBLE_CAMERA_CENTER);
	// add point normals
	pclViewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cameraCenter, cameraNormal, 1, 0.1, NAME_VISIBLE_CAMERA_NORMAL);
	// set point size
	pclViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, NAME_VISIBLE_CAMERA_CENTER);
}

void MvsViewer::printPatchInformation(const Patch &pth) const {
	const Vec3d &center       = pth.getCenter();
	const Vec3d &normal       = pth.getNormal();
	const Vec2d &normalS      = pth.getSphericalNormal();
	const vector<int> &camIdx = pth.getCameraIndices();
	const int camNum          = pth.getCameraNumber();

	printf("\n");
	printf("ID: %d\n", pth.getId());
	printf("center: %f %f %f\n", center[0], center[1], center[2]);
	printf("normal: %f %f %f\n", normal[0], normal[1], normal[2]);
	printf("spherical normal: %f %f\n", normalS[0], normalS[1]);
	printf("distance to origin: %f\n", -normal.ddot(center));
	printf("avg correlation: %f\n", pth.getCorrelation());
	printf("Level of detail: %d\n", pth.getLOD());
	printf("fitness: %f\n", pth.getFitness());
	printf("priority: %f\n", pth.getPriority());
	printf("visible camera number: %d\n", pth.getCameraNumber());
	printf("depth: %f\n", pth.getDepth());
	printf("depth range: %f ~ %f\n", pth.getDepthRange()[0], pth.getDepthRange()[1]);

	double corr;
	for (int i = 0; i < camNum; ++i) {
		const Camera &cam = mvs->getCamera(camIdx[i]);
		corr = normal.ddot(-cam.getOpticalNormal());
		printf("visible angle correlation %d: %f\n", camIdx[i], corr);
	}

	pth.showRefinedResult();
	pth.showError();
}

double MvsViewer::getNormalLength() const {
	const vector<Camera> &cameras = mvs->getCameras();
	const int camNum = (int) cameras.size();

	double sum = 0;
	for (int i = 0; i < camNum; i++) { 
		const Camera &cam1 = cameras[i];
		for (int j = 0; j < camNum; j++) {
			if (i==j) continue;
			const Camera &cam2 = cameras[j];
			sum += norm(cam1.getCenter()-cam2.getCenter());
		}
	}
	sum /= (camNum*camNum - camNum);

	return sum / 10.0;
}