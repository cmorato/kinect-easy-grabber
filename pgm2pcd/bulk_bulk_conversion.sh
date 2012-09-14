#!/bin/bash

DIRS="/pachi/Dropbox/motion_segmentation/kinect/data/3_bodies
	  /home/pachi/Dropbox/motion_segmentation/kinect/data/pachi_hands
	  /home/pachi/Dropbox/motion_segmentation/kinect/data/pachi_termos
	  /home/pachi/Dropbox/motion_segmentation/kinect/data/pachi_walk_hd"

#/home/pachi/Dropbox/motion_segmentation/kinect/data/1_body
#/home/pachi/Dropbox/motion_segmentation/kinect/data/1_body_fast

for d in ${DIRS}
do
	echo "---------------------------------------------------------------------"
	echo "---------------------------------------------------------------------"
	echo "---------------------------------------------------------------------"
	#echo ${d}
	./bulk_conversion.sh ${d}/depth ${d}/rgb ${d}/map ${d}/pcd
	
done

