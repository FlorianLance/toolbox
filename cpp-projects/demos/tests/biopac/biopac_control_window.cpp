
#include "biopac_control_window.hpp"

// std
#include <algorithm>

// opengl-utility
#include "opengl/vao.hpp"

using namespace tool::graphics;

bool BiopacControlWindow::initialize_gl(){

    // implot
    imPlotContext = ImPlot::CreateContext();
    gl::VAO::unbind();
    return m_glInitialized = true;
}

void BiopacControlWindow::post_update(){

}

void BiopacControlWindow::draw_imgui(){

    ImGui::Begin(m_imguiWindowTitle.c_str()); // begin window

    auto serial = std::string(std::begin(serialBuffer), std::end(serialBuffer));
    serial.erase(std::remove(serial.begin(), serial.end(), ' '), serial.end());

    ImGui::Checkbox("USB", &biopac.usb);
    ImGui::Checkbox("Automatic", &biopac.automaticSerial);
    ImGui::Checkbox("Use MP36", &biopac.mp36);

    if(!biopac.is_connected()){

        if(biopac.mp36){
            ImGui::TextColored(ImVec4(1.f,0,0,1),"No MP36 connected");
        }else{
            ImGui::TextColored(ImVec4(1.f,0,0,1),"No MP150 connected");
        }

        if(biopac.mp36){
            if(ImGui::Button("Connect MP36")){
                biopac.connect("auto");
            }
        }else{
            if(ImGui::Button("Connect MP150")){
                std::cout << serial << " " << serial.size() << "\n";
                biopac.connect(serial);
            }
        }
    }else{

        if(biopac.mp36){
            ImGui::TextColored(ImVec4(0.f,1,0,1), "MP36 connected");
        }else{
            ImGui::TextColored(ImVec4(0.f,1,0,1), "MP150 connected");
        }
        if(ImGui::Button("Disconnect")){
            biopac.disconnect();
        }

        if(!biopac.acquisition_started()){
            ImGui::TextColored(ImVec4(1.f,0,0,1),"Acquisition not started");
            if(ImGui::Button("Start acquisition")){
                biopac.start_acquisition();
            }
        }else{

            ImGui::TextColored(ImVec4(0.f,1,0,1),"Acquisition started");
            if(ImGui::Button("Stop acquisition")){
                biopac.stop_acquisition();
            }

            if(!biopac.thread_running()){
                ImGui::TextColored(ImVec4(1.f,0,0,1),"Reading thread not running");
                if(ImGui::Button("Start thread")){
                    biopac.start_reading_thread();
                }
            }else{
                ImGui::TextColored(ImVec4(0.f,1,0,1),"Reading thread running");
                if(ImGui::Button("Stop thread")){
                    biopac.stop_reading_thread();
                }
            }
        }
    }


    ImGui::InputText("Serial", serialBuffer.data(), serialBuffer.size());

    ImGui::InputDouble("Time per sample (ms)", &biopac.timePerSample, 1.0f, 1000.0f, "%.1f");

    for(int ii = 0; ii < 16; ++ii){
        bool v = biopac.analogCH[ii] == 1;
        ImGui::Checkbox((std::string("Enable channel ") + std::to_string(ii+1)).c_str(), &v);
        biopac.analogCH[ii] = v ? 1 : 0;
    }


//    ImGui::SliderFloat("scale", &x, 0.01f, 10.f, "ratio = %.2f");
//    if (ImGui::CollapsingHeader("Coords")){
//        ImGui::SliderFloat("scale", &graphics::Sample::scale, 0.01f, 10.f, "ratio = %.2f");
//        ImGui::SliderFloat("x", &graphics::Sample::x, -10.0f, 10.f, "ratio = %.2f");
//        ImGui::SliderFloat("y", &graphics::Sample::y, -10.0f, 10.f, "ratio = %.2f");
//        ImGui::SliderFloat("z", &graphics::Sample::z, -10.0f, 10.f, "ratio = %.2f");
//        ImGui::SliderFloat("x1", &graphics::Sample::x1, -10.0f, 10.f, "ratio = %.2f");
//        ImGui::SliderFloat("y1", &graphics::Sample::y1, -10.0f, 10.f, "ratio = %.2f");
//        ImGui::SliderFloat("z1", &graphics::Sample::z1, -10.0f, 10.f, "ratio = %.2f");
//        ImGui::SliderFloat("rx", &graphics::Sample::rx, -360.0f, 360.f, "ratio = %.2f");
//        ImGui::SliderFloat("ry", &graphics::Sample::ry, -360.0f, 360.f, "ratio = %.2f");
//        ImGui::SliderFloat("rz", &graphics::Sample::rz, -360.0f, 360.f, "ratio = %.2f");
//    }

    //    if (ImGui::CollapsingHeader("Scatter Plots")) { // crash
    //        srand(0);
    //        static float xs1[100], ys1[100];
    //        for (int i = 0; i < 100; ++i) {
    //            xs1[i] = i * 0.01f;
    //            ys1[i] = xs1[i] + 0.1f * ((float)rand() / (float)RAND_MAX);
    //        }
    //        static float xs2[50], ys2[50];
    //        for (int i = 0; i < 50; i++) {
    //            xs2[i] = 0.25f + 0.2f * ((float)rand() / (float)RAND_MAX);
    //            ys2[i] = 0.75f + 0.2f * ((float)rand() / (float)RAND_MAX);
    //        }

    //        if (ImPlot::BeginPlot("Scatter Plot", NULL, NULL)) {
    //            ImPlot::PlotScatter("Data 1", xs1, ys1, 100);
    //            ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
    //            ImPlot::SetNextMarkerStyle(ImPlotMarker_Square, 6, ImVec4(0,1,0,0.5f), IMPLOT_AUTO, ImVec4(0,1,0,1));
    //            ImPlot::PlotScatter("Data 2", xs2, ys2, 50);
    //            ImPlot::PopStyleVar();
    //            ImPlot::EndPlot();
    //        }
    //    }
    //    if (ImGui::CollapsingHeader("Stairstep Plots")) {
    //        static float ys1[101], ys2[101];
    //        for (int i = 0; i < 101; ++i) {
    //            ys1[i] = 0.5f + 0.4f * sinf(50 * i * 0.01f);
    //            ys2[i] = 0.5f + 0.2f * sinf(25 * i * 0.01f);
    //        }

    //        if (ImPlot::BeginPlot("Stairstep Plot", "x", "f(x)")) {
    //            ImPlot::PlotStairs("Signal 1", ys1, 101, 0.01f);
    //            ImPlot::SetNextMarkerStyle(ImPlotMarker_Square, 2.0f);
    //            ImPlot::PlotStairs("Signal 2", ys2, 101, 0.01f);
    //            ImPlot::EndPlot();
    //        }
    //    }
    ImGui::End();
}
