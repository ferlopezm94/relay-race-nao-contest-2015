#include "NaoVision.h"

/*Parametros de verde JerseyMexicoPerro
 * LowH  = 070/179
 * HighH = 103/179
 * LowS  = 046/255
 * HighS = 255/255
 * LowV  = 000/255
 * HighV = 121/255
 *
*/

/*Parametros de verde Cinta
 * LowH  = 050/179
 * HighH = 162/179
 * LowS  = 169/255
 * HighS = 255/255
 * LowV  = 141/255
 * HighV = 255/255
*/

NaoVision::NaoVision(const string ip, const int port, bool local): cameraProxy(ip, port), rng(12345) {
    iLowH = 0;
    iHighH = 77;
    iLowS = 43;
    iHighS = 229;
    iLowV = 0;
    iHighV = 255;
    thresh = 110;
    umbral = 60;
    areaColorDetection = 0;
    this->ip = ip;
    this->port = port;
    this->local = local;
    this->parameterClientName = "test";
    this->clientName = cameraProxy.subscribe(parameterClientName, AL::kQVGA, AL::kBGRColorSpace, 30); // Subscribe to ALVideoDevice
}

NaoVision::NaoVision(bool local): rng(12345) {
    iLowH = 0;
    iHighH = 77;
    iLowS = 43;     // Este parametro es el primero que hay que mover en busca de la deteccion del verde.
    iHighS = 229;
    iLowV = 0;
    iHighV = 255;
    thresh = 110;
    umbral = 60;
    areaColorDetection = 0;
    this->local = local;
    this->parameterClientName = "test";
    this->clientName = cameraProxy.subscribe(parameterClientName, AL::kQVGA, AL::kBGRColorSpace, 30); // Subscribe to ALVideoDevice
    
}

// Get image from NAO.
Mat NaoVision::getImageFrom(NaoCamera camera) {
    if (camera == TOP_CAMERA)
        cameraProxy.setActiveCamera(AL::kTopCamera);        // Connect to top camera.
    else
        cameraProxy.setActiveCamera(AL::kBottomCamera);     // Connect to bottom camera.

    // Image of 320*240 px.
    cameraProxy.setResolution(parameterClientName, 1);

    // Create an cv::Mat header to wrap into an opencv image.
    Mat imgHeader = Mat(cv::Size(320, 240), CV_8UC3);

    // Retrieves the latest image from the video resource.
    ALValue img = cameraProxy.getImageRemote(clientName);

    // Access the image buffer (6th field) and assign it to the opencv image container.
    imgHeader.data = (uchar*)img[6].GetBinary();

    // Tells to ALVideoDevice that it can give back the image buffer to the driver.
    // Optional after a getImageRemote but MANDATORY after a getImageLocal.
    cameraProxy.releaseImage(clientName);

    // Display the iplImage on screen.
    src = imgHeader.clone();

    return src;
}

// Process an image containing a line and return the angle with respect to NAO.
double NaoVision::calculateAngleToBlackLine() {
    // Convert image to gray and blur it.
    cvtColor(src, src_gray, CV_BGR2GRAY);
    blur(src_gray, src_gray, Size(3,3));

    if(!local)
        imshow("src", src);

    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Detect edges using canny.
    Canny(src_gray, canny_output, thresh, thresh * 2, 3);

    // Find contours.
    findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Get the moments.
    vector<Moments> mu(contours.size());

    for(int i = 0; i < contours.size(); i++)
        mu[i] = moments(contours[i], false);

    // Get the mass centers.
    vector<Point2f> mc( contours.size());

    for(int i = 0; i < contours.size(); i++)
        mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);

    // Eliminate contours without area.
    contoursClean.clear();
    int indMax = 0;
    int lengthMax = 0;

    for(int i = 0; i < contours.size(); i++) {
        area = mu[i].m00;
        length = arcLength(contours[i], true);
        punto = mc[i];

        if(area != 0 && length > 200 && punto.x > 0 && punto.y > 0)
            contoursClean.push_back(contours.at(i));
    }

    if(contoursClean.size() != 0) {
        // Get moments and mass for new vector.
        vector<Moments> muClean(contoursClean.size());

        for(int i = 0; i < contoursClean.size(); i++)
            muClean[i] = moments(contoursClean[i], false);

        // Get the mass centers.
        vector<Point2f> mcClean( contoursClean.size());

        for(int i = 0; i < contoursClean.size(); i++)
            mcClean[i] = Point2f(muClean[i].m10/muClean[i].m00, muClean[i].m01/muClean[i].m00);

        for(int i = 0; i < contoursClean.size(); i++) {
            punto = mcClean[i];
            length = arcLength(contoursClean[i], true);
        }

        // Find the longest.
        for(int i = 0; i < contoursClean.size(); i++) {
            length = arcLength(contoursClean[i], true);
            lengthMax = arcLength(contoursClean[indMax], true);

            if(i > 0) {
                if(length  > lengthMax)
                    indMax = i;
            } else
                indMax = 0;
        }

        // Draw contours.
        Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);

        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
        drawContours( drawing, contoursClean, indMax, color, 2, 8, hierarchy, 0, Point());
        circle(drawing, mcClean[indMax], 4, color, 5, 8, 0 );

        // Calculate the angle of the line.
        angleToALine = getAngleDegrees(contoursClean[indMax], drawing);

        puntoMax = mcClean[indMax];
        lengthMax = arcLength(contoursClean[indMax], true);

        // Show in a window.
        if(!local) {
            namedWindow("Contours", CV_WINDOW_AUTOSIZE);
            imshow("Contours", drawing);

            // Draw grid.
            line(drawing, Point(260,0), Point(260, drawing.rows), Scalar(255,255,255));
            line(drawing, Point(umbral,0), Point(umbral, drawing.rows), Scalar(255,255,255));
            line(drawing, Point((drawing.cols/2),0), Point((drawing.cols/2), drawing.rows), Scalar(255,255,255));
            line(drawing, Point(0,120), Point(320,120), Scalar(255,255,255));
            imshow("Contours", drawing);
        }
    }
    else { // Go straight.
        angleToALine = 90.0;
    }

    return angleToALine;
}

// Detect if the NAO is near the goal.
bool NaoVision::naoIsNearTheGoal(Mat originalImage) {
    getAreaRedColor(originalImage);

    if (!local)
        cout << "Red Area :  " << areaColorDetection << endl;

    if (areaColorDetection >= 30)
        return true;
    else
        return false;
}

// Detect if the NAO is near the goal.
bool NaoVision::naoIsNearTheGoalRelayRace(Mat originalImage) {
    double areaColorDetection = FinalLineFilterRelayRace(originalImage);

    if (!local)
        cout << "Black Area :  " << areaColorDetection << endl;

    if (areaColorDetection >= 30)
        return true;
    else
        return false;
}

double NaoVision::FinalLineFilterRelayRace(Mat originalImage){
    int LowH = 0;
    int HighH = 180;
    int LowS = 0;
    int HighS = 255;
    int LowV = 100;
    int HighV = 255;
    double finalLine = 0;
    Mat src_gray;
    Mat imgHSV;
    Mat imgThresholded;

    cvtColor(originalImage, imgHSV, COLOR_BGR2HSV);       // Convert the captured frame from BGR to HSV.
    inRange(imgHSV, Scalar(LowH, LowS, LowV), Scalar(HighH,HighS, HighV), imgThresholded); // Threshold the image.

    // Morphological opening (remove small objects from the foreground).
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    // Morphological closing (fill small holes in the foreground).
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    //drawContours(imgThresholded,contours,1,theObjects.at(i).getColor(),3,8,hierarchy);

    // Get the moments.
    Moments oMoments = moments(imgThresholded);

    // Receive the centroid area.
    double dArea = oMoments.m00;
    finalLine = (dArea / 100000);

    // Cloned the modified image to calculate the points.
    src_gray = imgThresholded.clone();
    if(!local){
        imshow("FinalLineFilter", imgThresholded);      // Show the thresholded image.
        imshow("Original", originalImage);                // Show the original image.
    }
    // Blur to soften the image points.
    blur(src_gray, src_gray, Size(3,3));
    //cout <<area << endl;
    //if(area>=1 && area<=20)
        //return true;    // Green area detected.
    return (195-finalLine);
}

// Returns the area of black color captured of a certain image.
int NaoVision::getAreaBlackColor(Mat originalImage) {
    iLowH = 0;
    iHighH = 180;
    iLowS = 0;
    iHighS = 255;
    iLowV = 100;
    iHighV = 255;
    ColorFilter(originalImage);

    areaColorDetection = 195 - areaColorDetection;
    cout << areaColorDetection << endl;

    return areaColorDetection;
}

// Returns the area of red color captured of a certain image.
int NaoVision::getAreaRedColor(Mat originalImage) {
/*Parametros de Cinta Rojo
* LowH  = 160/179
* HighH = 179/179
* LowS  = 100/255
* HighS = 255/255
* LowV  = 100/255
* HighV = 255/255
*/
    iLowH = 160;
    iHighH = 179;
    iLowS = 0;
    iHighS = 255;
    iLowV = 0;
    iHighV = 255;

    //calibrateColorDetection();
    ColorFilter(originalImage);

    return areaColorDetection;
}

// Returns the area of yellow color captured of a certain image.
int NaoVision::getAreaYellowColor(Mat originalImage) {
/*Parametros de Cinta Amarilla
* LowH  = 020/179
* HighH = 071/179
* LowS  = 169/255
* HighS = 255/255
* LowV  = 141/255
* HighV = 255/255
*/
    iLowH = 20;
    iHighH = 100;
    iLowS = 80;
    iHighS = 255;
    iLowV = 141;
    iHighV = 255;

    ColorFilter(originalImage);

    return areaColorDetection;
}

// Adds a filter with the parameters preconfigured and calculates the area obtained for a certain preselected color.
void NaoVision::ColorFilter(Mat originalImage) {
    Mat src_gray;
    Mat imgHSV;
    Mat imgThresholded;

    cvtColor(originalImage, imgHSV, COLOR_BGR2HSV);       // Convert the captured frame from BGR to HSV.
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); // Threshold the image.

    // Morphological opening (remove small objects from the foreground).
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    // Morphological closing (fill small holes in the foreground).
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    //drawContours(imgThresholded,contours,1,theObjects.at(i).getColor(),3,8,hierarchy);

    // Get the moments.
    Moments oMoments = moments(imgThresholded);

    // Receive the centroid area.
    double dArea = oMoments.m00;
    areaColorDetection = dArea / 100000;

    // Cloned the modified image to calculate the points.
    src_gray = imgThresholded.clone();
    if(!local){
        imshow("Thresholded Image", imgThresholded);      // Show the thresholded image.
        imshow("Original", originalImage);                // Show the original image.
    }
    // Blur to soften the image points.
    blur(src_gray, src_gray, Size(3,3));
    //cout <<area << endl;
    //if(area>=1 && area<=20)
        //return true;    // Green area detected.
}

// Method that allows us to find the values for the detection of a certain color.
void NaoVision::calibrateColorDetection() {
    namedWindow("Control", CV_WINDOW_AUTOSIZE);         // Create a window called "Control".

    // Create trackbars in "Control" window.
    cvCreateTrackbar("LowH" , "Control", &iLowH, 179);   // Hue (0 - 179).
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);
    cvCreateTrackbar("LowS" , "Control", &iLowS, 255);   // Saturation (0 - 255).
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);
    cvCreateTrackbar("LowV" , "Control", &iLowV, 255);   // Value (0 - 255).
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);
}

// Calculate the angle in degrees of a certain line.
double NaoVision::getAngleDegrees(const vector<Point> &pts, Mat &img) {
    // Construct a buffer used by the pca analysis.
    int sz = static_cast<int>(pts.size());
    Mat data_pts = Mat(sz, 2, CV_64FC1);

    for(int i = 0; i < data_pts.rows; ++i) {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    // Perform PCA analysis.
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

    // Store the center of the object.
    Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                      static_cast<int>(pca_analysis.mean.at<double>(0, 1)));

    // Store the eigenvalues and eigenvectors.
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);

    for(int i = 0; i < 2; ++i) {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    // Draw the principal components.
    circle(img, cntr, 3, Scalar(255, 0, 255), 2);
    Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
    drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);

    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x);     // Angle in radians.
    double degrees = angle * 180 / CV_PI;                       // Convert radians to degrees (0-180 range).
    degrees = degrees < 0 ? degrees + 180 : degrees;

    return degrees;
}

// The principal components are drawn in lines.
void NaoVision::drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2) {
     double angle = atan2( (double) p.y - q.y, (double) p.x - q.x );     // Angle in radians.
     double hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));

    // Here we lengthen the arrow by a factor of scale.
    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    line(img, p, q, colour, 1, CV_AA);

    // Create the arrow hooks.
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
}

void NaoVision::unsubscribe() {
   cameraProxy.unsubscribe(clientName);
}

// Getters and setters
void NaoVision::setSourceMat(Mat source) {
    src = source;
}

Mat NaoVision::getSourceMat() {
    return src;
}
