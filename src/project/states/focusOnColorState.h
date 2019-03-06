#pragma once

#include "../stateMachine/state.h"
#include <ArACTS.h>
#include <Aria.h>
#include "../utils/pid.hpp"

/*
 * Lets the robot focus on a toy
 * 
 * AUTHOR: Jonas Eckstein
 * Some smaller adjustments and help with tweaking the PID controllers: Denis Wagner
 */
class FocusOnColorState : public State
{
	/*
     * Top Left is 0,0
     * Robot 1: 720,480
	 * Robot 2: 720,480
     * y Axis is inverted,o x nt
     */

  public:
	void enterState();
	ArActionDesired *updateState(const ArActionDesired &currentDesired);
	void exitState();

  private:
	int targetChannel = 1; //TODO set via method
	int framesFocused = 0;
	int framesWithoutBlob = 0;
	ArACTS_1_2 *acts;
	ArPTZ *camera;
	int minBlobArea = 250; //minimum size of a blob to be tracked
	double xDiffTreshold = 1.5;

	Utils::PidController<double> xAxisPid = Utils::PidController<double>(0.05, 0.00005, 0.0003); //old: (0.3, 0.05, 0.1);
	Utils::PidController<double> yAxisPid = Utils::PidController<double>(0.1, 0, 0.02);			 //old: (0.1, 0.06, 0.02);
};
