#include "nav_grab_script.h"
CNavGrabScript::CNavGrabScript()
{
    
}

CNavGrabScript::~CNavGrabScript()
{

}

void CNavGrabScript::Queue()
{
    stAct newAct;

    newAct.nAct = ACT_SPEAK_SP;
    newAct.strTarget = "I will go to the kitchen";
    newAct.nDuration = 50;
    arAct.push_back(newAct);

    newAct.nAct = ACT_GOTO;
    newAct.strTarget = "kitchen";
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK_SP;
    newAct.strTarget = "Get the drink";
    newAct.nDuration = 1;
    arAct.push_back(newAct);

    newAct.nAct = ACT_GRAB;
    newAct.strTarget = "饮料";
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK_SP;
    newAct.strTarget = "Go to the master";
    newAct.nDuration = 1;
    arAct.push_back(newAct);

    newAct.nAct = ACT_GOTO;
    newAct.strTarget = "master";
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK_SP;
    newAct.strTarget = "This is the drink, Please hold it.";
    newAct.nDuration = 1;
    arAct.push_back(newAct);

    newAct.nAct = ACT_PASS;
    newAct.strTarget = "饮料";
    arAct.push_back(newAct);

}