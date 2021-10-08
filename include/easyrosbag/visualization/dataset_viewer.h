#ifndef PROJECT_DATASET_VIEWER_H
#define PROJECT_DATASET_VIEWER_H

#include <pangolin/pangolin.h>
#include <thread>
#include "utils/point_cloud.h"
#include "utils/common.h"

namespace easyrosbag
{
    class RosBagViewer
    {
    public:
        typedef std::shared_ptr<RosBagViewer> Ptr;

        static RosBagViewer::Ptr create() { return RosBagViewer::Ptr(new RosBagViewer());}

        RosBagViewer();

    public:
        void run();

        bool isRequiredStop();

        void setStop();

        bool waitForFinish();

        void setCurrentConfigFile(std::string strConfig);

        void regCallback(const std::function<void(int)>& callback);

        void setScan(std::vector<PointType> pcd){ current_scan = pcd;}

        void setIMU(easyrosbag::ImuOutput imu){ current_imu = imu;}

        void setTraj(){}
        
        void setGPS(){}

        void setIMU(){}

        void setImage(){}

    protected:
        void runCallBack(int oper);

        void clearState();

        void loadDataset();

        void reset();

        void processDataset();

        void exit();
      
    private:

        void drawAxis(pangolin::OpenGlMatrix &Twi);

        void drawCamera(pangolin::OpenGlMatrix &Twc);

        void getCurrentAxisPose(pangolin::OpenGlMatrix &M);

        //void drawCurrentImage(pangolin::GlTexture &gl_texture, cv::Mat &image);

        void drawGPS();

        void drawTraj();

        void drawScan();

        void drawPlots(bool show_accel,bool show_gyro);

        void getColorRampFromHeight(float minz, float maxz, float pz,float &r,float &g,float &b);

        void getColorRampFromIntensity(float intensity,float &r,float &g,float &b);

    private:

        std::shared_ptr<std::thread> pangolin_thread = nullptr;

        bool required_stop;
        bool is_finished;

        std::mutex mutex_stop;

        std::string mConfigPath;

        bool isStoped;
        
        // pangolin::DataLog imu_log;
        // pangolin::Plotter* plotter;

        std::vector<std::function<void(int)>> mCbVec;

        std::vector<PointType> current_scan;
        easyrosbag::ImuOutput current_imu;
    };
}

#endif