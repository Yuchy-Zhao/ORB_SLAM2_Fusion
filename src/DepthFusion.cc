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

DepthFusion::DepthFusion(double depthFactor, std::string savePath)
{
    this->depthFactor = depthFactor;
    this->savePath = savePath;
    // this->mTSDFMap = Mapper(voxel_size_s, MemoryType::kDevice);
}

void DepthFusion::Shutdown()
{
    unique_lock<mutex> lock(mMutexShutdown);
    mbShutdown = true;
    this->SaveKeyFrame(this->savePath);
    this->TSDFFusion(this->savePath);
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
    for (size_t i=0; i<N ; i++)
    {
        std::stringstream colorPath;
        colorPath << savePath << "results/frame" << std::setw(6) << std::setfill('0') << i << ".png";
        std::stringstream depthPath;
        depthPath << savePath << "results/depth" << std::setw(6) << std::setfill('0') << i << ".png";
        cv::imwrite(colorPath.str(), mvColorImgs[i]);
        cv::Mat depth;
        if (depthFactor == 1.0)
        {
            mvDepthImgs[i] *= 6553.5;
            mvDepthImgs[i].convertTo(mvDepthImgs[i], CV_16U);
        }
        cv::imwrite(depthPath.str(), mvDepthImgs[i]);
        KeyFrame* pKF = mvpKeyFrames[i];
        cv::Mat T;
        cv::invert(pKF->GetPose(), T);
        f << setprecision(9) << T.at<float>(0,0) << " " << T.at<float>(0,1)  << " " << T.at<float>(0,2)  << " " << T.at<float>(0,3)  << " "
                << T.at<float>(1,0) << " " << T.at<float>(1,1)  << " " << T.at<float>(1,2)  << " " << T.at<float>(1,3)  << " "
                << T.at<float>(2,0) << " " << T.at<float>(2,1)  << " " << T.at<float>(2,2)  << " " << T.at<float>(2,3)  << " "
                << T.at<float>(3,0) << " " << T.at<float>(3,1)  << " " << T.at<float>(3,2)  << " " << T.at<float>(3,3)  << endl;
        std::cout << "Save " << i << "-th image and depth." << std::endl;
    }
    f.close();   
}

void DepthFusion::TSDFFusion(std::string savePath)
{
    size_t N = mvpKeyFrames.size();
    open3d::pipelines::integration::ScalableTSDFVolume volume(
        0.01, 0.04, open3d::pipelines::integration::TSDFVolumeColorType(1)
    );
    for (size_t i=0; i<N ; i++)
    {
        std::stringstream colorPath;
        colorPath << savePath << "results/frame" << std::setw(6) << std::setfill('0') << i << ".png";
        std::stringstream depthPath;
        depthPath << savePath << "results/depth" << std::setw(6) << std::setfill('0') << i << ".png";
        open3d::geometry::Image color;
        open3d::geometry::Image depth;
        open3d::io::ReadImage(colorPath.str(), color);
        open3d::io::ReadImage(depthPath.str(), depth);
        double depthscale = depthFactor;
        if (depthFactor == 1.0)
            depthscale = 6553.5;
        auto RGBD = open3d::geometry::RGBDImage::CreateFromColorAndDepth(color, depth, depthscale, 6.0, false);
        KeyFrame* pKF = mvpKeyFrames[i];
        cv::Mat cT = pKF->GetPose();
        Eigen::Matrix4f fT;
        fT << cT.at<float>(0,0), cT.at<float>(0,1), cT.at<float>(0,2), cT.at<float>(0,3),
              cT.at<float>(1,0), cT.at<float>(1,1), cT.at<float>(1,2), cT.at<float>(1,3),
              cT.at<float>(2,0), cT.at<float>(2,1), cT.at<float>(2,2), cT.at<float>(2,3),
              cT.at<float>(3,0), cT.at<float>(3,1), cT.at<float>(3,2), cT.at<float>(3,3);
        Eigen::MatrixXd dT = fT.cast<double>(); 
        volume.Integrate(*RGBD, open3d::camera::PinholeCameraIntrinsic(RGBD->color_.width_, RGBD->color_.height_, pKF->fx, pKF->fy, pKF->cx, pKF->cy), dT);
        std::cout << "Integerate " << i << "-th image into the volume." << std::endl;
    }
    auto mesh = volume.ExtractTriangleMesh();
    std::stringstream meshname;
    meshname << savePath << "mesh.ply";
    open3d::io::WriteTriangleMeshToPLY(meshname.str(), *mesh, true, true, false, true, false, false);
    std::cout << "Extract mesh and save." << std::endl;
}

} //namespace ORB_SLAM
