
#include <iostream>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <iomanip>
#include <vector>
#include <map>
#include <sys/types.h>
#include <sys/stat.h>
#include <unordered_set>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "dataStructures.h"
#include "camFusion.hpp"

using namespace std;

void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<float>>> dataset){
    // Make a CSV file with one or more columns of integer values
    // Each column of data is represented by the pair <column name, column data>
    //   as std::pair<std::string, std::vector<int>>
    // The dataset is represented as a vector of these columns
    // Note that all columns should be the same size
    
    // Create an output filestream object
    std::ofstream myFile(filename);
    
    // Send column names to the stream
    for(int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j).first;
        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";
    
    // Send data to the stream
    for(int i = 0; i < dataset.at(0).second.size(); ++i)
    {
        for(int j = 0; j < dataset.size(); ++j)
        {
            myFile << dataset.at(j).second.at(i);
            if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
        }
        myFile << "\n";
    }
    
    // Close the file
    myFile.close();
}

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait, int imgIndex, const std::string path_dir)
{
    // create topview image
       cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));
       
 for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
                // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
            // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }
    // create topview image

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string image_name= "Top View Lidar Pts #" + std::to_string((unsigned long)imgIndex)+".jpg";
    cv::imwrite(path_dir+'/'+image_name,topviewImg);
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
     std::pair<int,int> boxes;
     float robust_mean = 0.0;
    //To eliminate those, I recommend that you compute a robust mean of all the euclidean distances between keypoint matches and then remove those that are too far away from the mean
    
    for(auto kptMatch = kptMatches.begin() ; kptMatch != kptMatches.end(); kptMatch++)
    {
        // find robust mean of all euclidean distances between all matches
        robust_mean+= cv::norm(kptsCurr[kptMatch->trainIdx].pt - kptsPrev[kptMatch->queryIdx].pt);                
    }
    robust_mean /= kptMatches.size();

    // remove points that are "too far away(not exactly above the mean but a bit too higher than this)"
    for(auto kptMatch = kptMatches.begin() ; kptMatch != kptMatches.end(); kptMatch++)
    {
        // find robust mean of all euclidean distances between all matches         
        float current_mean = cv::norm(kptsCurr[kptMatch->trainIdx].pt - kptsPrev[kptMatch->queryIdx].pt);
        // mean*1.3 so anything above this is too far away from the mean and should not be included.
        if((boundingBox.roi.contains(kptsCurr[kptMatch->trainIdx].pt) == true) && (current_mean <= (robust_mean*1.3)))
        {
            boundingBox.kptMatches.push_back(*kptMatch);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
     // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

            double minDist = 50.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }
    std::sort(begin(distRatios), end(distRatios));

    // compute median dist. ratio to remove outlier influence
    double median;

    if (distRatios.size() % 2 == 0) 
    {
        int index1= (distRatios.size()/2)-1;
        int index2= (distRatios.size()/2);
        median = (distRatios[index2] + distRatios[index1]) /2;
    } else {
        median = distRatios[(distRatios.size()/2) - 0.5];
    }

    const double dT{ 1 / frameRate };

    TTC = -dT / (1 - median);
}

bool compare(LidarPoint lidar1, LidarPoint lidar2)
{
    return lidar1.x < lidar2.x; 
}
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;
    // calculating average of lidar points
    // if lidar point is above average then it must be an outlier so do not count to calculation
    std::sort(begin(lidarPointsPrev), end(lidarPointsPrev), compare);
    std::sort(begin(lidarPointsCurr), end(lidarPointsCurr),compare);
    if (lidarPointsPrev.size() % 2 == 0) 
    {
        int index1= (lidarPointsPrev.size()/2)-1;
        int index2= (lidarPointsPrev.size()/2);
        minXPrev = (lidarPointsPrev[index2].x + lidarPointsPrev[index1].x) /2;
    } else {
        minXPrev = lidarPointsPrev[(lidarPointsPrev.size()/2) - 0.5].x;
    }

    if (lidarPointsCurr.size() % 2 == 0) 
    {
        int index3= (lidarPointsCurr.size()/2)-1;
        int index4= (lidarPointsCurr.size()/2);
        minXCurr = (lidarPointsCurr[index4].x + lidarPointsCurr[index3].x) /2;
    } else {
        minXCurr = lidarPointsCurr[(lidarPointsCurr.size()/2) - 0.5].x;
    }

    double dT = 1 / frameRate;
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}

 //// STUDENT ASSIGNMENT
            //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
//            map<int, int> bbBestMatches;
//            matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end()-2), *(dataBuffer.end()-1));

// associate bounding boxes between current and previous frame using keypoint matches

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    std::pair<int,int> boxes;
    std::map<std::pair<int,int>, int> bbBestMatchesCurr;
    for(int k= 0 ; k < matches.size();k++)
    {
        for(auto i=  currFrame.boundingBoxes.begin(); i != currFrame.boundingBoxes.end();i++)
        {
            for(auto j=  prevFrame.boundingBoxes.begin(); j != prevFrame.boundingBoxes.end();j++)
            {
                if((i->roi.contains(currFrame.keypoints[matches[k].trainIdx].pt) == true) && (j->roi.contains(prevFrame.keypoints[matches[k].queryIdx].pt) == true))
                {

                    bbBestMatchesCurr[{j->boxID,i->boxID}]++;
                }
            }
        }
    }
    // has more than one match these two box matches so add them to the best matched bounding boxes
    int maximum_matches= 0;
    for(auto iter = bbBestMatchesCurr.begin();iter != bbBestMatchesCurr.end();iter++)
    {
        if(iter->second > maximum_matches)
        {
            bbBestMatches.insert({iter->first.first,iter->first.second});
        }
    }
}
