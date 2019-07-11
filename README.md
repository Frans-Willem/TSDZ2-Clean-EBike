# TSDZ2 Clean EBike
This is a fork of the [TSDZ2-Smart-Ebike](https://github.com/OpenSource-EBike-firmware/TSDZ2-Smart-EBike) project, with slightly differing goals from the original project.
We diverged from the 0.19.0 release

## Goals
### Readability & maintainability
This goes for code as well as commit messages. Code will be read much more often than it is written, so optimize for that case.

In some cases it may be needed to sacrifice readability for performance, but in those cases use a scientific method to figure out if the less-readable version is actually more performant. 
If both the readable and performant alternatives result in the same code being generated by the compiler, always pick the more readable or more maintainable alternative.

### Stability
Aim to keep the master branch stable. When experimenting with new features, keep them on a separate branch until it's been proven stable.
People will be using these bikes in traffic, let's respect that.

### Usability
Prefer usability over a mess of features and settings. If a feature or setting would only be used by 5% of users, prefer a seperate fork, or put stuff behind ifdefs.
This firmware should be suitable for most city e-bikes, speed pedelecs/e-bikes, and in some cases for electric mountainbikes.

## Releases and development builds
This fork does not yet have any official releases. However, an automated build system has been set up with Travis-CI.org, uploading it's results to bintray.

Please note that the automated builds may not be stable, and flashing them to your bike is at your own risk.
Nevertheless, we will take great care to ensure that at least the master branch should be relatively safe to try out.

The main (master) branch's build result and download link are below:\
[![Build Status](https://travis-ci.org/Frans-Willem/TSDZ2-Clean-EBike.svg?branch=master)](https://travis-ci.org/Frans-Willem/TSDZ2-Clean-EBike)
[ ![Download](https://api.bintray.com/packages/frans-willem/TSDZ2-Clean-EBike/master/images/download.svg) ](https://bintray.com/frans-willem/TSDZ2-Clean-EBike/master/_latestVersion#files)


Other build results and logs can be found at [Travis-CI.org](https://travis-ci.org/Frans-Willem/TSDZ2-Clean-EBike).
Built binaries for older versions or other branches can be found at [Bintray.com](https://bintray.com/frans-willem/TSDZ2-Clean-EBike/).

## Compilation
The project is compiled using SDCC 3.9.0. While it may work with older or newer versions, SDCC is prone to breaking


## Original readme
This ebike motor controller firmware project is to be used with the Tongsheng TSDZ2 mid drive motor.

It has the following benefits compared to the stock firmware:
* The motor runs more efficient therefore it becomes more powerful and consumes less energy.
* The ebike will feel more responsive and agile.
* Using other supported displays and pheriperals will provide more functionality and features. Because this project is in heavy development more features will be added.

This project is being developed and maintained for free by a community of users. Some of them are developers who work professionally developing this type of technology for very well known companies.

For more details, see the project page: https://github.com/OpenSource-EBike-firmware/TSDZ2_wiki/wiki 

**IMPORTANT NOTES**
* Installing this firmware will void your warranty of the TSDZ2 mid drive and KT-LCD3.
* We are not responsible for any personal injuries or accidents caused by use of this firmware.
* There is no guarantee of safety using this firmware, please use it at your own risk.
* We advise you to consult the laws of your country and tailor the motor configuration accordingly.
* Please be aware of your surroundings and maintain a safe riding style.
