#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <alvision/alimage.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alvisiondefinitions.h>

using namespace std;
using namespace AL;
using namespace cv;

class NaoVision {
public:
    enum NaoCamera {TOP_CAMERA, BOTTOM_CAMERA};

    //Initial and ending Functions
    NaoVision(const string ip, const int port, bool local);
    NaoVision(bool local);
    Mat getImageFrom(NaoCamera camera);
    double calculateAngleToBlackLine();
    void unsubscribe();
    void setSourceMat(Mat source);
    Mat getSourceMat();

    // Color Filters
    bool naoIsNearTheGoal(Mat originalImage);
    bool naoIsNearTheGoalRelayRace(Mat originalImage);
    int getAreaBlackColor(Mat originalImage);
    int getAreaRedColor(Mat originalImage);
    int getAreaYellowColor(Mat originalImage);
    double FinalLineFilterRelayRace(Mat originalImage);
    void ColorFilter(Mat originalImage);

    //Calibration
    void calibrateColorDetection();

private:
    RNG rng;
    Mat src;
    Mat src_gray;
    Point2f punto;
    Point2f puntoMax;
    vector<vector<Point> > contoursClean;
    AL::ALVideoDeviceProxy cameraProxy;

    bool local;             // Flag for the execution type (local or remote).
    int area;
    int areaColorDetection; // Store the area captured by the NAO camera of a preselected chosen color.
    int port;
    int length;
    int thresh;
    int umbral;             // Part of the frame that will not be taken into account.
    double angleToALine;    // Detected angle line.
    string ip;
    string clientName;
    string parameterClientName;

    // Variables that allow us to detect different colors.
    int iLowH;
    int iHighH;
    int iLowS;
    int iHighS;
    int iLowV;
    int iHighV;

    double getAngleDegrees(const vector<Point> &pts, Mat &img);
    void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale);
};
