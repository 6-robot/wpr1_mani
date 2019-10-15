#ifndef NAV_GRAB_SCRIPT_H
#define NAV_GRAB_SCRIPT_H
#include "action_manager.h"

class CNavGrabScript : public CActionManager
{
public:
	CNavGrabScript();
	~CNavGrabScript();
    void Queue();
};

#endif // NAV_GRAB_SCRIPT_H