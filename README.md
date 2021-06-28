# Point cloud plane segmentation/removal

This is a tool for iterative large plane segmentation/removal in (indoor) point clouds based on PCL SAC segmentation. It segments planes perpendicular to the respective axis/axes setting and then dumps these planes as well as the plane model outliers as .ply or .pcd point clouds with a log file.

### Visualization output example
![output example](https://github.com/romankraemer/cloud-plane-seg/blob/main/output_viewer_example.png "output example")

### Prerequisites
 - Build environment [tested with GCC 7 and CMake 3.10]
 - PointCloudLibrary [tested with PCL 1.8(.1)]
 - Boost filesystem and Boost program_options

### Build
```bash
$ cd path/to/build/dir
$ cmake path/to/src/dir
$ make
```

### Example usage
```bash
$ cd path/to/build/dir
$ ./cloud-plane-seg --input path/to/your/point/cloud.pcd --axis zyx --angle-xy 2 --angle-xz 10 --angle-yz 10 --distance-xy 0.03 --distance-xz 0.03 --distance-yz 0.03 --multi-xy 0.25 --multi-xz 0.25 --multi-yz 0.25 --nplanes all --max-i 10
```

The output files will be dumped into /output_files in the root directory of this tool.

### Inputs and options
Use input argument *\-\-help* to show a list of input arguments

```bash
$ ./cloud-plane-seg --help
```

|input argument   | explanation   |
| ------------ | ------------ |
|\-\-help | List of input arguments. Default values will be used if there is no input for an argument.|
|\-\-input arg | Set the path to the input file (.ply or .pcd). Use quotation marks (\-\-input "/path to the file") if there are spaces in the file path.|
|\-\-axis arg | Set axis for SAC perpendicular plane model: xy-plane (\-\-axis z), xz-plane (\-\-axis y), yz-plane (\-\-axis x), a combination (\-\-axis yx) or all (\-\-axis zyx) [default: zyx].|
|\-\-output arg | Set the output file format (\-\-output ply or \-\-output pcd) for the filtered cloud [default: ply].|
|\-\-nplanes arg | Try to find all planes (all) or only one plane (one) [default: all].|
|\-\-max-i arg | Set the max. iterations per axis as integer [default: not set].|
|\-\-angle-xy arg | Set the EpsAngle for xy-plane [default: 2.0°].|
|\-\-angle-xz arg | Set the EpsAngle for xz-plane [default: 10.0°].|
|\-\-angle-yz arg | Set the EpsAngle for yz-plane [default: 10.0°].|
|\-\-distance-xy arg | Set the distance threshold value for xy-plane [default: 0.03].|
|\-\-distance-xz arg | Set the distance threshold value for xz-plane [default: 0.04].|
|\-\-distance-yz arg | Set the distance threshold value for yz-plane [default: 0.04].|
|\-\-multi-xy arg | Set multiplier value for SAC loop sample termination condition. New plane model must contain at least (multiplier)*(points of previous plane model) [default: 0.25].|
|\-\-multi-xz arg | Set multiplier value for SAC loop sample termination condition. New plane model must contain at least (multiplier)*(points of previous plane model) [default: 0.25].|
|\-\-multi-yz arg | Set multiplier value for SAC loop sample termination condition. New plane model must contain at least (multiplier)*(points of previous plane model) [default: 0.25].|

### Some additional explanation
This tool uses *pcl::SACMODEL_PERPENDICULAR_PLANE model* and begins sampling with the z-axis (floor plane, ceiling plane). Usually here are the most points which can be segmented/removed. This makes it easier to find planes perpendicular to the y-axis and x-axis afterwards.

#### Eps angle:
Increase angle value if walls/floors/ceilings are not perfectly perpendicular to the corresponding axis.

#### Distance threshold:
Increase distance threshold value for uneven walls/floors/ceiling. From experience distance thresholds for walls need to be higher than for the floor as walls are often not as plane as they seem. The same units as those of the point cloud are used. For example, if the scaling of the point cloud is in meters, --distance-xy 0.03 means a threshold of ±3cm.

#### Multiplier: multi-xy, multi-xz, multi-yz
Since this tool is based on RANSAC, a plane model will always be found as long as enough points remain. The multiplier introduces a termination condition:

*if (current_inliers < previous_inliers &ast; multiplier){
SAC stop sampling;}*

Basic idea: if the amount of points in a found plane model gets rapidly smaller, one can assume that most/all of the meaningful/actual planes were found.

Values for multiplier 0.0 ... 1.0.

**lower value** &#8594;  more plane models will be found, but potentially leading to "overshooting" and finding plane models for points which do not represent a plane of interest.

**higher value** &#8594; termination condition met faster, but potentially failing to find plane models for smaller planes of interest.

#### Set max. iterations
To avoid the problems mentioned above you can choose a low value for the multiplier(s) and set ***\-\-max-i*** to a fixed number (e.g. 10) to limit the sampling of plane models per axis to this number.
