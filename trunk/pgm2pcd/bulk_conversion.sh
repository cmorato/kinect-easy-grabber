#!/bin/bash

PGM2PCDBINARY="/home/pachi/Dropbox/motion_segmentation/kinect/KinectEasyGrabber/pgm2pcd/Release/pgm2pcd"

echo ""
echo "Usage:"
echo "  " $0" <depth_images_path> <rgb_images_path> <map_coords_path> <pcd_output_path>"
echo ""

if [ $# -lt 4 ]
then
	echo "ERROR: 4 input parameters required"
	exit 1
fi


DEPTHIMAGES=`ls "${1}"`
RGBIMAGES=`ls "${2}"`
MAPCOORDS=`ls "${3}"`
PCDOUTPUT="${4}"

ARRAYDEPTHIMAGES=()
ARRAYRGBIMAGES=()
ARRAYMAPCOORDS=()

mkdir ${PCDOUTPUT} 2> /dev/null

index=0
for d in ${DEPTHIMAGES}
do
	ARRAYDEPTHIMAGES[$index]=$d
	((index++))
done

index=0
for d in ${RGBIMAGES}
do
	ARRAYRGBIMAGES[$index]=$d
	((index++))
done

index=0
for d in ${MAPCOORDS}
do
	ARRAYMAPCOORDS[$index]=$d
	((index++))
done

#----------------------------------------
#----------------------------------------
#----------------------------------------

count=$index
echo $count " images read"
echo ""

index=0
while [ $index -lt $count ]
do
	pcd_output_file=${ARRAYDEPTHIMAGES[$index]:0:(-3)}
	pcd_output_file=${pcd_output_file/depth/pcd}"pcd"
	eval ${PGM2PCDBINARY} \"${1}/${ARRAYDEPTHIMAGES[$index]}\" \"${2}/${ARRAYRGBIMAGES[$index]}\" \"${3}/${ARRAYMAPCOORDS[$index]}\" \"${4}/${pcd_output_file}\"
	((index++))
done


