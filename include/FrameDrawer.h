/**
 * @file FrameDrawer.h
 * @author Raúl Mur-Artal, Yanmin Wu, thanks for your great work
 * @modified by Zhewei Ye
 * @Lastest modified on 19/04/2023
 */

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "SoSlam.h"
#include "line_descriptor.hpp"
#include "line_lbd_allclass.h"

#include <vector>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
namespace gtsam_soslam {

// class Tracking;
    class Viewer;

    class FrameDrawer {
    public:
        explicit FrameDrawer(SoSlamState *sState);

        // Update info from the last processed frame.
        void Update();

        // Draw last processed frame.
        cv::Mat DrawFrame();

        cv::Mat DrawYoloFrame();

        cv::Mat GetRawColorImage();

        cv::Mat GetQuadricImage(const bool menuShowQuadricObj, const bool menuShowGroundTruth);

    protected:

        cv::Mat DrawYoloInfo(cv::Mat &im, bool bText);

        std::vector<cv::Scalar> colors = {cv::Scalar(0, 0, 255),
                                          cv::Scalar(0, 255, 0),
                                          cv::Scalar(255, 0, 0),
                                          cv::Scalar(0, 255, 255)
        };

        std::vector<std::string> class_names = {
                "person",
                "bicycle",
                "car",
                "motorbike",
                "aeroplane",
                "bus",
                "train",
                "truck",
                "boat",
                "traffic light",
                "fire hydrant",
                "stop sign",
                "parking meter",
                "bench",
                "bird",
                "cat",
                "dog",
                "horse",
                "sheep",
                "cow",
                "elephant",
                "bear",
                "zebra",
                "giraffe",
                "backpack",
                "umbrella",
                "handbag",
                "tie",
                "suitcase",
                "frisbee",
                "skis",
                "snowboard",
                "sports ball",
                "kite",
                "baseball bat",
                "baseball glove",
                "skateboard",
                "surfboard",
                "tennis racket",
                "bottle",
                "wine glass",
                "cup",
                "fork",
                "knife",
                "spoon",
                "bowl",
                "banana",
                "apple",
                "sandwich",
                "orange",
                "broccoli",
                "carrot",
                "hot dog",
                "pizza",
                "donut",
                "cake",
                "chair",
                "sofa",
                "pottedplant",
                "bed",
                "diningtable",
                "toilet",
                "tvmonitor",
                "laptop",
                "mouse",
                "remote",
                "keyboard",
                "cell phone",
                "microwave",
                "oven",
                "toaster",
                "sink",
                "refrigerator",
                "book",
                "clock",
                "vase",
                "scissors",
                "teddy bear",
                "hair drier",
                "toothbrush"
        };

        // Info of the frame to be drawn
        cv::Mat mIm;
        cv::Mat mRGBIm;
        cv::Mat mDetIm;
        cv::Mat mQuadricIm;

        int N;
        vector <cv::KeyPoint> mvCurrentKeys;
        vector<bool> mvbMap, mvbVO;
        bool mbOnlyTracking;
        int mnTracked, mnTrackedVO;
        vector <cv::KeyPoint> mvIniKeys;
        vector<int> mvIniMatches;
        int mState;

        // lines.
        std::vector<KeyLine> Dkeylines_raw, Dkeylines_out;
        double DTimeStamp;

        // bounding box.
        std::vector<Detection> Dboxes;
        bool have_detected;
        std::vector<Eigen::MatrixXd> DObjsLines;    // object lines.
        // [EAO] project quadrics to the image.
        cv::Mat DrawQuadricProject(cv::Mat &im,
                                   const cv::Mat &P,
                                   const cv::Mat &axe,
                                   const cv::Mat &Twq,
                                   int nClassid,
                                   bool isGT = true,
                                   int nLatitudeNum = 7,
                                   int nLongitudeNum = 6);

        SoSlamState *s;

        std::mutex mMutex;
    };

} //namespace gtsam_soslam

#endif // FRAMEDRAWER_H
