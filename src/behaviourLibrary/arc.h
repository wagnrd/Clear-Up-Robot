
#include <Aria.h>

class Arc : public ArAction
{
public:
    Arc(double vel, double rotVel);
    virtual ArActionDesired *fire(ArActionDesired currentDesired);

    double getVel() const;
    void setVel(double value);
    double getRotVel() const;
    void setRotVel(double value);

protected:
    ArActionDesired actionDesired;  // what the action wants to do
    double vel;     // how fast we move
    double rotVel;  // how fast we turn
};
