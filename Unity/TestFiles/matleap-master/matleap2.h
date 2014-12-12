/**************************************************************************
 * matleap2.h
 *  Author: Kevin O'Neill
 *  Date: 2014.11.26
 *
 *  This file defines the mex Leap Motion data streamer for MATLAB.
 *
 *************************************************************************/

#ifndef MATLEAP2_H
#define MATLEAP2_H

#define MAJOR_REVISION 0
#define MINOR_REVISION 4

#include "Leap.h"
#include "mex.h"

namespace matleap2
{

/// @brief a leap frame
struct frame
{
    int64_t id;
    int64_t timestamp;
    Leap::HandList allHands;
};

/// @brief leap frame grabber interface
class frame_grabber
{
    private:
    bool debug;
    Leap::Controller controller;
    frame current_frame;
    public:
    /// @brief constructor
    frame_grabber ()
        : debug (false)
    {
        // receive frames even when you don't have focus
        controller.setPolicyFlags (Leap::Controller::POLICY_BACKGROUND_FRAMES);
    }
    /// @brief destructor
    ~frame_grabber ()
    {
        if (debug)
            mexPrintf ("Closing matleap frame grabber\n");
    }
    /// @brief debug member access
    ///
    /// @param flag turn it on/off
    void set_debug (bool flag)
    {
        if (flag == debug)
            return;
        if (flag)
            mexPrintf ("Setting debug on\n");
        debug = flag;
    }
    /// @brief get a frame from the controller
    ///
    /// @return the frame
    const frame &get_frame ()
    {
        const Leap::Frame &f = controller.frame ();
        current_frame.id = f.id ();
        if (debug)
            mexPrintf ("Got frame with id %d\n", current_frame.id);
        current_frame.timestamp = f.timestamp ();
        current_frame.allHands = f.hands ();
        return current_frame;
    }
};

} // namespace matleap

#endif
