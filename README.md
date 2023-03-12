# SFND 3D Object Tracking

Flow Diagram of Project

<img src="images/course_code_structure.png" width="779" height="414" />

# Project Rubric:

## FP.0 Final Report
- Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

## FP.1 Match 3D Objects
- Implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences.

https://github.com/Photonf22/3D_Object_Tracking/blob/d1720884aa9c6bf60ce6e09532616ca6dc968695/src/camFusion_Student.cpp#L317

## FP.2 Compute Lidar-based TTC
- Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

https://github.com/Photonf22/3D_Object_Tracking/blob/d1720884aa9c6bf60ce6e09532616ca6dc968695/src/camFusion_Student.cpp#L279

## FP.3 Associate Keypoint Correspondences with Bounding Boxes
- Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box.

https://github.com/Photonf22/3D_Object_Tracking/blob/d1720884aa9c6bf60ce6e09532616ca6dc968695/src/camFusion_Student.cpp#L188

## FP.4 Compute Camera-based TTC
- Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.

https://github.com/Photonf22/3D_Object_Tracking/blob/d1720884aa9c6bf60ce6e09532616ca6dc968695/src/camFusion_Student.cpp#L216

## FP.5 : Performance Evaluation 1

![image](https://user-images.githubusercontent.com/105236455/224571888-11a896b0-9b31-402c-8735-5974c2e0b22b.png)

According to the above examples, It can be assumed that the factor that caused the lidar to be somewhat off compared to the Camera TTC is due to the outliers points. Even though we used the mean distance of all points between the current and previous frame. This was not enought to get rid of these outliers which affected our results. One way to solve this would be to lower the threshold value of what is acceptable when calculating TTC Lidar. Thus points that fall above the mean euclidian distance times the threshold will be ignored. Currently the threshold is set to 1.3 but one could lower it to 1.1 to get rid of more points that affect the lidar TTC calculation.

###  Harris-Brisk

#### Image 10  (Previous) and Image 11 (Current and Previous) as well as Image 12 (Current) Top-View
![image](https://user-images.githubusercontent.com/105236455/224577921-ceb7360f-de1c-47d7-9c29-486f8ce5efcd.png)

### Fast-SIFT

#### Image 6  (Previous) and Image 7 (Current) Top-View
![image](https://user-images.githubusercontent.com/105236455/224577970-8df0d07b-a1cd-4394-95b7-e3f2e1295edd.png)

#### Image 9 (Previous) and Image 10 (Current) Top-View
![image](https://user-images.githubusercontent.com/105236455/224578023-2215982a-4149-40d2-bc9d-2368228a527c.png)

### Akaze-ORB
#### Image 8 (Previous) and Image 9 (Current) Top-View

![image](https://user-images.githubusercontent.com/105236455/224578219-26e0f934-4886-4f0f-918a-dc21777e50b0.png)

#### Image 11 (Previous) and Image 12 (Current) Top-View

![image](https://user-images.githubusercontent.com/105236455/224578202-7946303c-96a9-4d8c-bb64-4e2f28dcce9d.png)

## FP.6 Performance Evaluation 2

#### Top Choices according to speed and accuracy

![image](https://user-images.githubusercontent.com/105236455/224578309-7878a705-e177-4cbc-9b7c-14f282ab0cec.png)

#### Image Index along with their respective TTC Lidar and Camera.

![image](https://user-images.githubusercontent.com/105236455/224578324-f6681ecc-28ac-410f-a1ad-9edec1fddfc4.png)

Graph of Chosen Detectors-Descriptors which shows the differences between TTC Lidar and TTC Camera estimation

![image](https://user-images.githubusercontent.com/105236455/224578376-839ed9e5-fcc2-4f83-b214-22bb66db6315.png)

Chosen Detector-Descriptor and TTC Lidar observation:

Fast-Brisk:
- Looking at the Fast-Brisk combination one can easily see that Image 9 the Lidar TTC  estimation is off due to the amount of outliers in the image. 

Fast-ORB:
-  Looking at the Fast-ORB combination one can easily see that Image 10 the Lidar TTC  estimation is off due to the amount of outliers in the image. 

Akaze-ORB:
- Looking at the Akaze-ORB combination one can easily see that Image 9 the Lidar TTC  estimation is off due to the amount of outliers in the image. 

Fast-Brief:
-  Looking at the Fast-Brief combination one can easily see that Image 10 the Lidar TTC  estimation is off due to the amount of outliers in the image. 

I chose the following 3 top detector/descriptor combinations because they are the ones that perform the fastest. Therefore my answer to this question is biased on the seed of the Combination of detectors/descriptors. The faster and more accurately we can obtain an estimated TTC then the better since in real life applications speed is important and reliability when talking about life and death scenarious. 

Below is a table which shows some examples where TTC Camera estimation is way off.

- In harris-brisk Image 10 Excel was unable to calculate or show the estimated TTC Camera since the value would have been infinity. 

![image](https://user-images.githubusercontent.com/105236455/224578511-8d9e3cec-97e2-4e51-ae18-da6b05a0806c.png)

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
  * Install Git LFS before cloning this Repo.
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

