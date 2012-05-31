#include "stdafx.h"

#include <opencv2\opencv.hpp>

#include "mvs\mvs.h"
#include "view\mvsviewer.h"

using namespace cv;
using namespace PAIS;

int _tmain(int argc, _TCHAR* argv[])
{
	clock_t start_t, end_t;
	
	MVS mvs(5, 15, 36);
	mvs.loadNVM("../../../TMVS_data/face/face.nvm");

	start_t = clock();
	mvs.refineSeedPatches();
	end_t = clock();

	printf("time1\t%f\n", (double)(end_t - start_t) / CLOCKS_PER_SEC);

	MvsViewer viewer(mvs, true);

	return 0;
}