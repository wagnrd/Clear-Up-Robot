#include <Aria.h>

class Swerve : public ArAction
{
public:
    enum Direction{
        Left = 1,
        Right = -1
    };

    Swerve(Direction turnDir, int avoid=500, int standoff=1000,
           int turnSpeed=20);
    virtual ArActionDesired *fire(ArActionDesired currentDesired);

    bool isAvoiding() const;
    int getObstacleDist() const;

    int getTurnSpeed() const;
    void setTurnSpeed(int value);

    Direction getTurnDir() const;
    void setTurnDir(const Direction &value);

    int getAvoidDist() const;
    void setAvoidDist(int value);

    int getStandOff() const;
    void setStandOff(int value);


protected:
    ArActionDesired actionDesired;	// what the action wants to do
    Direction turnDir;  // direction to turn, Left or Right
    int turnSpeed;      // rotation speed
    int avoid;          // close contact distance(full throttle on turning)
    int obstacleDist;   // how close is the obstacle
    int standoff;       // start obstacle avoidance at this distance
    bool avoiding;      // whether an obstacle has been detected
};
