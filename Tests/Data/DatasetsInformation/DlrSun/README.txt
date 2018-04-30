# README VERSION 1.0
# 2015-05-06-12:52
# DLR e.V.
# Institute of Robotics and Mechatronics
# Department of Perception and Cognition
# 
# Written by Martin Lingenauber

Welcome to the CROOS-CV dataset
================================

This dataset is intended to support and benchmark Computer Vision (CV) development for Close Range On-Orbit Servicing (CROOS). It is an representative image dataset for CROOS operations with distances of 2 m between servicer and client satellite that was recorded under illumination conditions similar to a Low Earth Orbit.

A training set with 180 trajectories and a test set with 810 trajectories are provided. Both were recorded with an industrial robot and with three different sun incidence angles and multiple shutter times. A trajectory consists of stereo image pairs along with the ground truth pose of the cameras. Additionally, a 3D model of the client and all calibration data is provided with the dataset.

The training set contains images recorded with shutter times specifically chosen to match the illumination situation in our recording setup as good as possible. It is intended to be used for algorithm development, to observe challenges for computer vision algorithms and for parameter optimization as shown in the application example in OUR PAPER. In contrast, the test set is intended to be used for performance analysis only and shall not be used to tune parameters. It contains more different illumination conditions and some random brightness changes in the images.

Using the CROOS-Vis Dataset?
 =============================

Please let us know if you are using the CROOS-CV dataset by sending an email to martin.lingenauber@dlr.de (as we are curious in which ways it can be used and we would like to keep you updated on any changes).

Please cite the following paper if you use the CROOS-CV dataset in your research:

@INPROCEEDINGS{lingenauber2015dataset,
  title = {{A} dataset to support and benchmark computer vision development
    for close range on-orbit servicing},
  author = {{L}ingenauber, {M}artin and {K}riegel, {S}imon and {K}a{\ss}ecker,
    {M}ichael and {P}anin, {G}iorgio},
  booktitle = {{ASTRA} 2015 - 13th Symposium on {A}dvanced {S}pace {T}echnologies in {R}obotics
    and {A}utomation},
  year = {2015}
}

License
=======

All data in this dataset are copyright by us and published under the Creative Commons Attribution-ShareAlike 4.0 License (Creative Commons Attribution-ShareAlike 4.0 International License). This means that you must credit the authors in the manner specified above and you may distribute the resulting work only under the same license.

The license text is given in the file LICENSE_CC-BY-SA-4.0.txt in the root folder of the dataset.

Content Overview
=================
The dataset contains the following folders:

== training_set and test_set ==

Both folders contain the folders sun0, sun1, sun2 for the different sun positions. Each sun folder contains folders with different shutter times called focus20_shutter<shutter time>_noreflector. Then these folder contain the 30 folders LIF<LIF number>_<LIF trajectory> for the 30 different paths towards the target LIF brackets.

A trajectory is defined as the set of corresponding images that were recorded while following a path, i.e. moving the robot from a start point on a specific path towards a certain target (in this case the LIFs) on a satellite, with a certain illumination (or sun position) and with a defined shutter time. The trajectories are generated so that the left camera is always viewing the center of the LIF. Each trajectory contains the uncalibrated, greyscale stereo image pairs pic<acquisition number>.<camera>.png along with the corresponding ground truth poses of the robot's Tool Center Point pic<acquisition number>.tcp.coords and of each camera pic<acquisition number>.<camera>.coords. Here the acquisition number refers to the image number obtained along the trajectory (000 for furthest away) and camera refers to left (0) and right (1). Furthermore, the pose of the light source is provided for each sun position in the corresponding sun folder with the files pose_sun<sun position number>_focus20_shutter<shutter time>_noreflector. Please always check if there is a specific sun pose file for a certain shutter time as this indicates that the sun position was slightly changed and remeasured between recordings for two different shutter times.

== calibration ==

This folder contains the camera calibration file, which includes the intrinsics and extrinsics of each camera. The calibration was done with the DLR CalDe CalLab software, for details about the he calibration tool and about the calibration file format please visit DLR CalDe and CalLab (http://www.dlr.de/rmc/rm/desktopdefault.aspx/tabid-3925/)

== 3d_model ==

The 3D model of the complete mockup and of each targeted Launcher Interface Attachment (LIF), all correctly positioned in the robot's reference frame, are available as low and as high resolution meshes in Wavefront OBJ and PLY file format.

== videos ==

Additionally, videos for each camera, compiled from the single images with 10 fps, are available for a more convenient access to the data.

