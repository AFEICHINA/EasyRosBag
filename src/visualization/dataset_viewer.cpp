#include "visualization/dataset_viewer.h"

using namespace std;

namespace easyrosbag
{
    RosBagViewer::RosBagViewer() :required_stop(false), is_finished(false)
    {
        isStoped = false;
        pangolin_thread = std::make_shared<std::thread>(std::bind(&RosBagViewer::run, this));

        // plotter = new pangolin::Plotter(&log, 0.0, 100, -10.0, 10.0, 0.01f, 0.01f);
    }

    void RosBagViewer::setStop() {
        std::lock_guard<std::mutex> lock(mutex_stop);
        required_stop = true;
    }

    bool RosBagViewer::isRequiredStop() {
        std::lock_guard<std::mutex> lock(mutex_stop);
        return required_stop;
    }

    bool RosBagViewer::waitForFinish() {
        if (!isRequiredStop())
            setStop();

        if (pangolin_thread->joinable())
            pangolin_thread->join();
        
        return true;
    }

    void RosBagViewer::setCurrentConfigFile(std::string strConfig)
    {
        std::lock_guard<std::mutex> lock(mutex_stop);
        mConfigPath = strConfig;
    }

    void RosBagViewer::clearState() {}

    void RosBagViewer::loadDataset() {
    }

    void RosBagViewer::regCallback(const std::function<void(int)>& callback)
    {
        mCbVec.emplace_back(callback);
    }

    void RosBagViewer::runCallBack(int oper)
    {    
        for (auto& it : mCbVec)
        {
            it(oper);
        }
    }

    void RosBagViewer::processDataset() {}

    void RosBagViewer::reset()
    {
        if(!isStoped)
        {
            std::cout<< "stop running first" <<std::endl;
            return;
        }

        clearState();
    }

    void RosBagViewer::exit()
    {
        runCallBack(-1);
    }

    void RosBagViewer::run() {

        using ViewerButton = pangolin::Var<std::function<void(void)>>;

        const string win_name = "Rosbag Viewer";

        int w, h;
        w = 640;
        h = 360;

        pangolin::CreateWindowAndBind(win_name, 2 * w, 2 * h); //create a display window
        glEnable(GL_DEPTH_TEST); //launch depth test
        glEnable(GL_BLEND);      //use blend function
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //set blend alpha value

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(2 * w, 2 * h, 500, 500, w, h, 0.1, 2000),
                pangolin::ModelViewLookAt(-5, 0, 5, 0, 0, 0, 1, 0, 1)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &main_display = pangolin::Display("camera")
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -w / (float) h)
                .SetHandler(new pangolin::Handler3D(s_cam));

        // This view will take up no more than a third of the windows width or height, and it
        // will have a fixed aspect ratio to match the image that it will display. When fitting
        // within the specified bounds, push to the top-left (as specified by SetLock).
        // int width = 752; //image_size.width;
        // int height = 480; //image_size.height;

        // pangolin::View &d_video0 = pangolin::Display("imgVideo0")
        //         .SetAspect(width / (float) height);

        // pangolin::GlTexture texVideo0(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

        // float swr = 0.7;
        // pangolin::CreateDisplay()
        //         .SetBounds(0.0, 1.0, swr, 1.0)
        //         .SetLayout(pangolin::LayoutEqual)
        //         .AddDisplay(d_video0);

        pangolin::View& img_view_display = pangolin::CreateDisplay()
                                                .SetBounds(0.3, 1.0, 0.7, 1.0)
                                                .SetLayout(pangolin::LayoutEqual);

        pangolin::View &plot_display = pangolin::CreateDisplay().SetBounds(
            0.0, 0.3, pangolin::Attach::Pix(175), 1.0);

        // plot_display.AddDisplay(*plotter);

        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175)); //new button and menu
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
        pangolin::Var<bool> menuViewScan("menu.Show Scan", true, true);
        pangolin::Var<bool> menuViewGPS("menu.Show GPS", true, true);
        pangolin::Var<bool> menuViewIMU("menu.Show IMU", true, true);
        pangolin::Var<bool> menuViewPoses("menu.Show Pose", true, true);
        pangolin::Var<bool> menuViewImage("menu.Show Image", true, true);
        pangolin::Var<bool> buttonViewFPS("menu.FPS", false, false);

        // ViewerButton LoadBtn("menu.LoadData", std::bind(&RosBagViewer::loadDataset, this));
        // ViewerButton ProcessBtn("menu.ProcessData", std::bind(&RosBagViewer::processDataset, this));
        ViewerButton ConfigBtn("menu.Reset", std::bind(&RosBagViewer::reset, this));
        ViewerButton ExitBtn("menu.Exit", std::bind(&RosBagViewer::exit, this));

        pangolin::OpenGlMatrix Twi;
        Twi.SetIdentity();
        pangolin::OpenGlMatrix Twc0;
        Twc0.SetIdentity();
        pangolin::OpenGlMatrix Twc1;
        Twc1.SetIdentity();

        while (!pangolin::ShouldQuit() && !isRequiredStop()) {

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            if (pangolin::Pushed(buttonViewFPS)) {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0, 1, 0, 0, 0, 1, 0, 0));
            }

            getCurrentAxisPose(Twi);
            if (menuFollowCamera)
                s_cam.Follow(Twi);

            main_display.Activate(s_cam);
            glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

            // draw origin coordinates
            pangolin::glDrawAxis(0.1);

            // draw current axis
            drawAxis(Twi);

            if(menuViewScan) {
                drawScan();
            }

            if(menuViewGPS) {
                drawGPS();
            }

            if(menuViewPoses) {
                drawTraj();
            }
            
            // if(menuViewIMU) {
            //     drawPlots(true,true);
            // }

            if(menuViewImage) {
                
            }

            if(isStoped) {
                glColor3f(1.0, 1.0, 0.0);   
                int curHeight = main_display.GetBounds().h;
                string strHeight = std::to_string(curHeight);
                string strTextRef = "STATUS :  Stoped";
                pangolin::GlFont::I().Text(strTextRef).DrawWindow(200, curHeight - 20 , 0);
            }
            else{
                glColor3f(0.0, 1.0, 0.0);
                int curHeight = main_display.GetBounds().h;
                string strHeight = std::to_string(curHeight);
                string strTextRef = "STATUS :  Processing";
                pangolin::GlFont::I().Text(strTextRef).DrawWindow(200, curHeight - 20 , 0);
            }
            
            pangolin::FinishFrame();
        }

        pangolin::DestroyWindow(win_name);
    }

    void RosBagViewer::drawPlots(bool show_accel,bool show_gyro)
    {
        // pcl::PointCloud<PointType>::Ptr laserScan = mData->getLaserScan();
        // if (laserScan&& laserScan->size() > 0) {
            
        // }

        // plotter->ClearSeries();
        // plotter->ClearMarkers();

        // if (show_accel) {
        //     plotter->AddSeries("$0", "$1", pangolin::DrawingModeDashed,
        //                     pangolin::Colour::Red(), "accel measurements x");
        //     plotter->AddSeries("$0", "$2", pangolin::DrawingModeDashed,
        //                     pangolin::Colour::Green(), "accel measurements y");
        //     plotter->AddSeries("$0", "$3", pangolin::DrawingModeDashed,
        //                     pangolin::Colour::Blue(), "accel measurements z");
        // }

        // if (show_gyro) {
        //     plotter->AddSeries("$0", "$4", pangolin::DrawingModeDashed,
        //                     pangolin::Colour::Red(), "gyro measurements x");
        //     plotter->AddSeries("$0", "$5", pangolin::DrawingModeDashed,
        //                     pangolin::Colour::Green(), "gyro measurements y");
        //     plotter->AddSeries("$0", "$6", pangolin::DrawingModeDashed,
        //                     pangolin::Colour::Blue(), "gyro measurements z");
        // }

        // double t = vio_dataset->get_image_timestamps()[show_frame] * 1e-9;
        // plotter->AddMarker(pangolin::Marker::Vertical, t, pangolin::Marker::Equal,
        //                     pangolin::Colour::White());

    }

    void RosBagViewer::drawAxis(pangolin::OpenGlMatrix &Twi) {
        const float &w = 1.08f; //mCameraSize;
        const float h = w;
        const float z = w;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twi.m);
#else
        glMultMatrixd(Twi.m);
#endif

        glLineWidth(2);  //set line width
        glBegin(GL_LINES);           //draw axis
        // axis z
        glColor3f(0.0f, 0.0f, 1.0f);   //blue
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, z);
        // axis x
        glColor3f(1.0f, 0.0f, 0.0f);   //red
        glVertex3f(0, 0, 0);
        glVertex3f(w, 0, 0);
        // axis y
        glColor3f(0.0f, 1.0f, 0.0f);   //green
        glVertex3f(0, 0, 0);
        glVertex3f(0, h, 0);

        glEnd();
        glPopMatrix();
    }

    void RosBagViewer::drawCamera(pangolin::OpenGlMatrix &Twc) {
        const float &w = 0.08f; //mCameraSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(0.5);  //set line width
        glColor3f(1.0f, 1.0f, 0.0f);   //yellow
        glBegin(GL_LINES);           //draw camera
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();
        glPopMatrix();
    }

    void RosBagViewer::getCurrentAxisPose(pangolin::OpenGlMatrix &M) {

        // if(mData->getCurrentTime() > 0)
        // {
        //     Eigen::Vector3d P;
        //     Eigen::Matrix3d R;
        //     mData->getCurrentPose(P,R);

        //     M.m[0] = R(0, 0);
        //     M.m[1] = R(1, 0);
        //     M.m[2] = R(2, 0);
        //     M.m[3] = 0.0;

        //     M.m[4] = R(0, 1);
        //     M.m[5] = R(1, 1);
        //     M.m[6] = R(2, 1);
        //     M.m[7] = 0.0;

        //     M.m[8] = R(0, 2);
        //     M.m[9] = R(1, 2);
        //     M.m[10] = R(2, 2);
        //     M.m[11] = 0.0;

        //     M.m[12] = P.x();
        //     M.m[13] = P.y();
        //     M.m[14] = P.z();
        //     M.m[15] = 1.0;
        // }
        // else
        // {
        //     M.SetIdentity();
        // }
    }

    // void
    // RosBagViewer::drawCurrentImage(pangolin::GlTexture &gl_texture, cv::Mat &image) {
    //     if (image.empty())
    //         return;

    //     if (image.type() == CV_8UC1)
    //         cv::cvtColor(image, image, CV_GRAY2RGB);

    //     {
    //         gl_texture.Upload(image.data, GL_RGB, GL_UNSIGNED_BYTE);
    //     } 
    // }

    void RosBagViewer::drawScan()
    {
        if (current_scan.size() > 0) {
            glPointSize(1);
            glColor3f(1.0, 1.0, 1.0);
            glBegin(GL_POINTS);
            for (size_t i = 0; i < current_scan.size(); i++) {
                PointType pt = current_scan[i];

                Eigen::Vector3d p(pt.x,pt.y,pt.z);
                glVertex3d(p(0),p(1),p(2));
            }
            glEnd();
        }
    }

    void RosBagViewer::drawGPS()
    {
        // std::vector<std::pair<double ,TrajPoint>> traj = mData->getGPSTraj();
        // if (traj.size() > 0) {

        //     Eigen::Vector3d tmp_p;

        //     // green
        //     glColor3f(0.0f, 1.0f, 0.0f);
        //     glLineWidth(2);
        //     glBegin(GL_LINE_STRIP);

        //     for (size_t i = 0; i < traj.size(); ++i) {
        //         TrajPoint prePose = traj[i].second;
        //         tmp_p = prePose.P;
        //         glVertex3f((float) tmp_p(0), (float) tmp_p(1), (float) tmp_p(2));
        //     }
        //     glEnd();
        // }
    }

    void RosBagViewer::getColorRampFromIntensity(float intensity,float &r,float &g,float &b)
    {
        intensity = intensity - std::floor(intensity);

        int reflection_map = intensity*10000;

        uint8_t pr,pg,pb;

        if (reflection_map < 30)
        {
            int green = (reflection_map * 255 / 30);
            pr = 0;
            pg = green & 0xff;
            pb = 0xff;
        }
        else if (reflection_map < 90)
        {
            int blue = (((90 - reflection_map) * 255) / 60);
            pr = 0x0;
            pg = 0xff;
            pb = blue & 0xff;
        }
        else if (reflection_map < 150)
        {
            int red = ((reflection_map-90) * 255 / 60);
            pr = red & 0xff;
            pg = 0xff;
            pb = 0x0;
        }
        else
        {
            int green = (((255-reflection_map) * 255) / (255-150));
            pr = 0xff;
            pg = green & 0xff;
            pb = 0;
        }

        r = pr;
        g = pg;
        b = pb;

        r /= 255.0;
        g /= 255.0;
        b /= 255.0;
    }

    void RosBagViewer::getColorRampFromHeight(float minz, float maxz, float pz,float &r,float &g,float &b) 
    {
        r = 0;
        g = 0;
        b = 1;
        float index = (pz - minz) / (maxz - minz) * 8.;
        if (index <= 2. ){
            r = 0;
            g = index* 0.5;
            b = 1;
        } else if (index > 2. && index <= 4.){
            r = 0;
            g = 1;
            b = 1 - (index - 2) * 0.5;
        }
        else if (index > 4. && index <= 6.){
            r = (index - 4) * 0.5;
            g = 1;
            b = 0;
        }
        else if (index > 6. && index <= 8.){
            r = 1;
            g = 1 - (index - 6) * 0.5;
            b = 0;
        }
        else {
            r = 1;
            g = 0;
            b = 0;
        }
    }

    void RosBagViewer::drawTraj()
    {
    //     std::vector<std::pair<double ,TrajPoint>> traj = mData->getTraj();
    //     if (traj.size() > 0) {

    //         Eigen::Vector3d tmp_p;

    //         // red
    //         glColor3f(1.0f, 0.0f, 0.0f);
    //         glLineWidth(1);
    //         glBegin(GL_LINE_STRIP);

    //         for (size_t i = 0; i < traj.size(); ++i) {
    //             TrajPoint prePose = traj[i].second;
    //             tmp_p = prePose.P;
    //             glVertex3f((float) tmp_p(0), (float) tmp_p(1), (float) tmp_p(2));
    //         }
    //         glEnd();

    //         // glColor3f(0.0f, 1.0f, 0.0f);

    //         // for (int i = 0; i < traj.size(); ++i) {
    //         //     TrajPoint prePose = traj[i].second;
    //         //     tmp_p = prePose.P;
              
    //         //     // draw text
    //             if(0)
    //             {
    //                 char id[32];
    //                 // int length = sprintf(id, "%d", i);
    //                 string txt(id);
    //                 pangolin::GlFont::I().Text( txt ).Draw( tmp_p(0),  tmp_p(1), tmp_p(2));
    //             }
    //         // }
    //     }
    }
}