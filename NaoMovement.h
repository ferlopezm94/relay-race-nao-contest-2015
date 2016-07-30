#include <alvision/alimage.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alvisiondefinitions.h>

using namespace std;
using namespace AL;

class NaoMovement {
public:
    enum NaoPositionOnLane {LEFT, CENTER, RIGHT};

    NaoMovement(const string ip, const int port, bool local);
    NaoMovement(bool local);

    void initialPositionIndividualRace();
    void initialPositionRelayRace();
    void moveInIndividualRace(double angleInDegrees);

    void moveInRelayRace(double angleInDegrees);
    void naoOnGoal();
    bool naoOnGoalRelayRace(double angleInDegrees);

    void leftCorrection();
    void middleCorrection();
    void rightCorrection();

    void stop();

private:
    AL::ALRobotPostureProxy posture;     // Posture Proxy
    AL::ALMotionProxy motion;            // Motion Proxy
    NaoPositionOnLane naoPositionOnLane; // Variable that determines where is the Nao on the lane.

    bool local;             // Flag for the execution type (local or remote).
    int port;
    string ip;

    double linearVelocityIndividualRace(double theta);
    double angularVelocityIndividualRace(double theta);
    double lateralVelocity(double theta);
    double linearVelocityRelayRace(double theta);
    double angularVelocityRelayRace(double theta);
    AL::ALValue walkingParametersIndividualRace();
    AL::ALValue walkingParametersRelayRace();
    AL::ALValue walkingParametersOnGoalRelayRace();
};
