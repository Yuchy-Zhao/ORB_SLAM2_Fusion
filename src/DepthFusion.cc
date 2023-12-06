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


#include "DepthFusion.h"
#include <open3d/Open3D.h>


namespace ORB_SLAM2
{

DepthFusion::DepthFusion(double voxelSize, std::string savePath)
{
    this->voxelSize = voxelSize;
    this->savePath = savePath;
    // this->mTSDFMap = Mapper(voxel_size_s, MemoryType::kDevice);
}

void DepthFusion::Shutdown()
{
    unique_lock<mutex> lock(mMutexShutdown);
    mbShutdown = true;
    this->SaveKeyFrame(this->savePath);
}

void DepthFusion::InsertKeyFrame(KeyFrame* pKF, cv::Mat &imRGB, cv::Mat &imD)
{
    unique_lock<mutex> lock(mMutexKeyFrame);
    mvpKeyFrames.push_back(pKF);
    mvColorImgs.push_back(imRGB.clone());
    mvDepthImgs.push_back(imD.clone());
}

void DepthFusion::SaveKeyFrame(std::string savePath)
{
    size_t N = mvpKeyFrames.size();
    ofstream f;
    std::stringstream filename;
    filename << savePath << "traj.txt";
    f.open(filename.str());
    f << fixed;
    open3d::pipelines::integration::ScalableTSDFVolume volume(
        4.0 / 512.0, 0.04, open3d::pipelines::integration::TSDFVolumeColorType(1)
    );
    for (size_t i=0; i<N ; i++)
    {
        std::stringstream colorPath;
        colorPath << savePath << "results/frame" << std::setw(6) << std::setfill('0') << i << ".jpg";
        std::stringstream depthPath;
        depthPath << savePath << "results/depth" << std::setw(6) << std::setfill('0') << i << ".png";
        cv::imwrite(colorPath.str(), mvColorImgs[i]);
        cv::imwrite(depthPath.str(), mvDepthImgs[i]);
        KeyFrame* pKF = mvpKeyFrames[i];
        cv::Mat T;
        cv::invert(pKF->GetPose(), T);
        f << setprecision(9) << T.at<float>(0,0) << " " << T.at<float>(0,1)  << " " << T.at<float>(0,2)  << " " << T.at<float>(0,3)  << " "
                << T.at<float>(1,0) << " " << T.at<float>(1,1)  << " " << T.at<float>(1,2)  << " " << T.at<float>(1,3)  << " "
                << T.at<float>(2,0) << " " << T.at<float>(2,1)  << " " << T.at<float>(2,2)  << " " << T.at<float>(2,3)  << " "
                << T.at<float>(3,0) << " " << T.at<float>(3,1)  << " " << T.at<float>(3,2)  << " " << T.at<float>(3,3)  << endl;
        open3d::geometry::Image color;
        open3d::geometry::Image depth;
        open3d::io::ReadImage(colorPath.str(), color);
        open3d::io::ReadImage(depthPath.str(), depth);
        open3d::geometry::RGBDImage rgbd;
        rgbd.CreateFromColorAndDepth(color, depth, 5000.0, 6.0, false);
        Eigen::Matrix4f fT;
        fT << T.at<float>(0,0), T.at<float>(0,1), T.at<float>(0,2), T.at<float>(0,3),
              T.at<float>(1,0), T.at<float>(1,1), T.at<float>(1,2), T.at<float>(1,3),
              T.at<float>(2,0), T.at<float>(2,1), T.at<float>(2,2), T.at<float>(2,3),
              T.at<float>(3,0), T.at<float>(3,1), T.at<float>(3,2), T.at<float>(3,3);
        Eigen::MatrixXd dT = fT.cast<double>(); 
        volume.Integrate(rgbd, open3d::camera::PinholeCameraIntrinsic(640, 480, 525.0, 525.0, 319.5, 319.5), dT);
        std::cout << "Integerate" << i << "-th image into the volume." << std::endl;
    }
    
}

void DepthFusion::TSDFFusion(KeyFrame* pKF, cv::Mat &imRGB, cv::Mat &imD)
{


}

} //namespace ORB_SLAM
