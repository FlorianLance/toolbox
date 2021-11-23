
/*******************************************************************************
** Toolbox-base                                                               **
** MIT License                                                                **
** Copyright (c) [2018] [Florian Lance]                                       **
**                                                                            **
** Permission is hereby granted, free of charge, to any person obtaining a    **
** copy of this software and associated documentation files (the "Software"), **
** to deal in the Software without restriction, including without limitation  **
** the rights to use, copy, modify, merge, publish, distribute, sublicense,   **
** and/or sell copies of the Software, and to permit persons to whom the      **
** Software is furnished to do so, subject to the following conditions:       **
**                                                                            **
** The above copyright notice and this permission notice shall be included in **
** all copies or substantial portions of the Software.                        **
**                                                                            **
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR **
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   **
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    **
** THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER **
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    **
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        **
** DEALINGS IN THE SOFTWARE.                                                  **
**                                                                            **
********************************************************************************/

#pragma once

// local
#include "kinect4_data_types.hpp"

namespace tool::camera::K4{

    static void save_compressed_frames_to_file(){

        //    void ScanerManagerWindow::save_video_file(){

        //        auto path = QFileDialog::getSaveFileName(nullptr, "Select kinect video file", QString(), "Kinect video (*.kvid)");
        //        if(path.length() == 0){
        //            return;
        //        }

        //        std::ofstream file;
        //        file.open(path.toStdString(), std::ios_base::binary);
        //        if(!file.is_open()){
        //            QtLogger::error("Cannot save video to " + path);
        //            return;
        //        }

        //        // retrieve intrinsics
        //        std::unordered_map<size_t, std_v1<float>> intrinsics;
        //        for(auto &data : m_savedData){
        //            auto idC = std::get<0>(data);
        //            if(intrinsics.count(idC) == 0){
        //                intrinsics[idC] = std::get<1>(data)->intrinsics;
        //            }
        //        }

        //        // write total nb of clouds
        //        files::write(file, m_savedData.size());

        //        // write nb of cameras
        //        files::write(file, m_grabbersManager.size());

        //        for(size_t ii = 0; ii < m_grabbersManager.size(); ++ii){

        //            // write camera transformation
        //            files::write(file, m_grabbersManager[ii]->get_model_matrix());

        //            // write cameras intrinsics
        //            if(intrinsics.count(ii) != 0){
        //                auto &camIntrinsics = intrinsics[ii];
        //                // 0 focalLengthX
        //                // 1 focalLengthY
        //                // 2 principalPointX
        //                // 3 principalPointY
        //                files::write_array(file, camIntrinsics.data(), 4);
        //            }else{ // defaults
        //                std_v1<float> defaultI = {1.f,1.f,1.f,1.f};
        //                files::write_array(file, defaultI.data(), 4);
        //            }
        //        }

        //        // write clouds data
        //        for(auto &data : m_savedData){

        //            // get id
        //            auto id = std::get<0>(data);

        //            // get frame
        //            auto &frameD = std::get<1>(data);
        //            files::write(file, frameD->timeStampGetFrame); // timestamp
        //            files::write(file, id); // id camera
        //            files::write(file, frameD->frameId); // frame id

        //            // write sizes
        //        files::write(file, frameD->compressedDepthSize);
        //            files::write(file, frameD->jpegColorSize);

        //            // write compressed data
        //            file.write(reinterpret_cast<char*>(frameD->compressedDepthData.data()), static_cast<std::streamsize>(frameD->compressedDepthSize*4));
        //            file.write(reinterpret_cast<char*>(frameD->compressedImage.data()), static_cast<std::streamsize>(frameD->jpegColorSize));

        //            // write bodies
        //            for(auto &body : frameD->bodiesData){

        //                files::write(file, &body.id);
        //                files::write(file, static_cast<int>(body.engaged));
        //                files::write(file, body.tracked);
        //                files::write(file, body.restricted);
        //                files::write(file, static_cast<int>(body.leanTracking));
        //                files::write(file, static_cast<int>(body.leftHandState));
        //                files::write(file, static_cast<int>(body.rightHandState));
        //                files::write(file, static_cast<int>(body.leftHandHightConfidence));
        //                files::write(file, static_cast<int>(body.rightHandHightConfidence));
        //                files::write(file, &body.lean);
        //                files::write(file, body.joints.size());

        //                // write joints
        //                for(auto &joint : body.joints){
        //                    auto jointT = joint.first;
        //                    files::write(file, &jointT);
        //                    files::write(file, &joint.second.pos);
        //                    files::write(file, &joint.second.rotQuaternion);
        //                    files::write(file, &joint.second.state);
        //                }
        //            }
        //        }
        //        file.close();
        //        QtLogger::message("Video saved at " + path);
        //    }
    }


}
