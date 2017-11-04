# pais-mvs
Multi-view stereo image-based 3D reconstruction

PAIS Multi-view Stereo

![image](https://github.com/adahbingee/pais-mvs/blob/master/doc.JPG)


library:

OpenCV 2.4.2

Point Cloud Library 1.6.0

# Publications
* "A novel 3D dense reconstruction with high accuracy and completeness". Zen Chen, Wen-Chao Chen, And Ping-Yi Sung. IEEE International Conference on Multimedia and Expo Workshops ICMEW 2013

* "Stochastic Optimization Based 3D Dense Reconstruction from Multiple Views with High Accuracy and Completeness". Wen-Chao Chen, Zen Chen, And Ping-Yi Sung. Journal of Information Science And Engineering 31, 131-146, 2015

* "Stocastic Optimization Based 3D Reconstruction with High Accuracy and Completeness". Wen-Chao Chen, Ping-Yi Sung And Zen Chen . The 25th IPPR Conference on Computer Vision, Graphics, and Image Processing, CVGIP 2012 Excellent Paper Award

[Link](http://www.phototalks.idv.tw/academic/?page_id=1494)

[Pre-build binary](https://github.com/adahbingee/pais-mvs/releases)

# Index
* Introduction
* Input file format
 * nvm file format
 * nvm2 file format
 * mvs file format
* Output file format
 * mvs file format
 * ply file format
 * psr file format
* Config parameters
* Command

# Introduction
1. Input / Output file format
2. Config parameters
3. Command line instruction

# Input file format
## nvm file format
NVM file is come from VisualSFM data structure. Here is the example of the data structure. The detail can be found in FileLoader.

You can skip the calibration step and just use the Structure from Motion result output from [VisualSFM](http://ccwu.me/vsfm/).

```
NVM_V3 

5
pawn0013.jpg    614.095397949 0.705410371683 0.160690743319 0.671401589359 0.160605237544 -0.556085150075 0.0481223921551 -0.00781510757143 -0.199289312888 0 
pawn0010.jpg    616.175537109 0.90353903514 0.221746421078 0.3576944596 0.0806247263945 -0.880841878288 0.0327703491031 -0.684201024844 -0.209314043486 0 
pawn0011.jpg    612.03302002 0.85241383667 0.2037593266 0.469072019941 0.108830220502 -0.71971232163 0.0433857776889 -0.492035476323 -0.207263977174 0 
pawn0012.jpg    611.360473633 0.786507583571 0.183363764635 0.573952646995 0.135504187104 -0.608685012281 0.0487066227347 -0.263440114899 -0.203210786458 0 
pawn0014.jpg    617.585876465 0.611485687162 0.135944898976 0.757586998462 0.183482834469 -0.572254659063 0.0434025057556 0.255716172724 -0.198563271584 0 
```

The first row shows the NVM file version.

Third line `5` indicate there are 5 cameras in the following list.

Each list entry:

 * image file name, can be PNG, JPG, BMP, or any OpenCV supported image format. Note that the file path can be relative or absolute path. 
 * focal length
 * 4 elements vector (k wx wy wz) indicate the quaternion rotation. Transformation form quaternion to 3x3 rotation matrix can be found in code. [Camera](https://code.google.com/p/pais-mvs/source/browse/trunk/TMVS/mvs/camera.cpp#6).
 * 3 elements vector (x y z) indicate camera center in world coordinate.
 * radial distortion parameter. The usage of parameter can be found in [camera projection](https://code.google.com/p/pais-mvs/source/browse/trunk/TMVS/mvs/camera.cpp#138).
 * 0 for end of line

## nvm2 file format
In the NVM file format, the camera principle point is assumed at the image center, and can't to be edit.

NVM2 file specify the principle point(px, py) after focal length. Others the same.

## mvs file format
MVS file is saved in a binary way for internal use, for example: MVS viewer, patch filtering.

# Output file format

## mvs file format
MVS file is saved in a binary way for internal use, for example: MVS viewer, patch filtering.

## ply file format
ply is the point cloud model. Can be open using MeshLab.

## psr file format
psr file is ready for the [Poisson surface reconstruction](http://www.cs.jhu.edu/~misha/Code/PoissonRecon/).

# Config parameters
config.txt should be placed in the same directory of main program.

```
### patch optimization configuration ###
# patch radius (patch size = 2*radius+1)
# !!!!check distWeighting if patchRadius have been changed!!!!
# default 15
patchRadius			15

# reduce normal search range for expansion patch (reduce to +- pi/reduceNormalRange)
# 1 stands for search 360 degree, 2 for 180 degree
# smaller for larger cell size
# default 2 (reduce to 180 degree)
reduceNormalRange	2

# enable or disable adaptive distance weighting (0: disable, 1: enable)
# default 1
adaptiveDistanceEnable		1
# distance weighting to patch center
# default(patch radius/3) 5
distWeighting		5

# enable or disable adaptive difference weighting (0: disable, 1: enable)
# default 1
adaptiveDifferenceEnable	1
# difference weighting
# default(128*128) 16384
diffWeighting		16384

# enable or disable adaptive gradient weighting (0: disable, 1: enable)
# default 0
adaptiveGradientEnable		0
# gradient maginitude weighting
# default 10.0
gradientWeighting	10.0

# visible camera angle correlation
# smaller for sparse camera deploy
# default 0.87 radian for 30 degree
visibleCorrelation	0.7

# depth search range scalar
# smaller for smoother object, larger for complex/sharp object
# smaller for small cell size, larger for big cell size
# larger depthRangeScalar cloud avoid local trap!
# beta in paper
# default(cellsize/2) 2
depthRangeScalar	8

### PSO optimization configuration ###
# particle number
# default 30
particleNum			15
# maximum iteration number
# default 60
maxIteration		30

### expansion configuration ###
# image cell size, control the density of patches
# default 4
cellSize			2
# maximum patch number in single cell
# default 3
maxCellPatchNum		3
# patch expansion strategy (Best: 0, Worst: 1, Breath: 2, Depth: 3)
# default 0
expansionStrategy	0

### level of detail configuration ###
# texture variation
# default 36
textureVariation	36
# minimum level of detail
# default 0
minLOD				0
# maximum level of detail
# default 15
maxLOD				15
# level of detail ratio
# default 0.8
lodRatio			0.8

### filtering configuration ###
# minimum visible camera number
# default 3
minCamNum			3
# minimum texture correlation
# default 0.9
minCorrelation		0.9
# minimum patch homography ellipse axes ratio
# default 0.15
minRegionRatio		0.15
# maximum fitness (critical effect the completeness)
# default 10.0
maxFitness			10.0
# neighbor patch search radius scalar (from TVCG09 PCMVS)
# the neighbor radius = pow(volume, 1.0/3.0) * neighborRadiusScalar
# default 0.0025*(cellsize*2)
neighborRadiusScalar		0.01
```

# command
There are 4 commands for our program.
`TMVS.exe -r`
`TMVS.exe -f`
`TMVS.exe -v`
`TMVS.exe -a`

## Reconstruction `TMVS.exe -r`
Run multi-view stereo reconstruction. 

The program will use the N-view matching points as the seed points from VisualSFM. 

When there are no seed points, it will run N-view matching algorithm by SIFT feature and perform N-view matching with epipolar constrain. 

Note that the config.txt must be initialized.

```
TMVS.exe -r input.nvm
```

There are 5 output files from reconstruction command.

`init.mvs` the input seed points from VisualSFM.

`seed.mvs` seed points after optimization.

`exp.mvs` expansion point cloud.

`exp.ply` expansion point cloud.

`exp.psr` expansion point cloud.

## Patch post-processing filtering `TMVS.exe -f`
Run post-processing filtering after reconstruction. Note the filtering process only accept `mvs` file.
```
TMVS.exe -f exp.mvs
```

## MVS Viewer `TMVS.exe -v`
Rendering result in PCL framework.

press `h` to see help function.

```
TMVS.exe -v exp.mvs
```

## MVS Viewer `TMVS.exe -a`
Rendering result in PCL framework with expansion animation.

press `h` to see help function.

```
TMVS.exe -a exp.mvs
```
