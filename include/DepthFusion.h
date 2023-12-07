/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DEPTHFUSION_H
#define DEPTHFUSION_H

#include "System.h"
#include <filesystem>
#include <condition_variable>
// #include <nvblox/core/types.h>
// #include <nvblox/mapper/mapper.h>

namespace ORB_SLAM2
{

class DepthFusion
{
public:
    DepthFusion();

    DepthFusion(double depthFactor, std::string savePath);

    void InsertKeyFrame(KeyFrame* pKF, cv::Mat &imRGB, cv::Mat &imD);

    void SaveKeyFrame(std::string savePath);

    void TSDFFusion(std::string savePath);
    
    void Shutdown();

protected:
    // Mapper mTSDFMap;

    std::vector<KeyFrame*> mvpKeyFrames;
    std::vector<cv::Mat> mvColorImgs;
    std::vector<cv::Mat> mvDepthImgs;
        
    std::mutex mMutexShutdown;
    bool mbShutdown;

    std::mutex mMutexKeyFrame;
    std::mutex mMutexKeyFrameUpdate;
    condition_variable mKeyFrameUpdate;

    double depthFactor;

    std::string savePath;

};

}

#endif // DEPTHFUSION_H