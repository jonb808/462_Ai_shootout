/*
 * Defines the classes used for decision trees.
 *
 * Part of the Artificial Intelligence for Games system.
 *
 * Copyright (c) Ian Millington 2003-2006. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */
#include <exception>
#include "dectree.h"
#include "playing.h" // needed for controlPanel
#define TRACE_DECTREE

namespace aicore
{

    
	DecisionTreeNode* DecisionPtr::makeDecision(RobotPlayer* bot, float dt)
    {
        // Choose a branch based on the getBranch method
        if ( getBranch(bot, dt) ) {
            // Make sure its not null before recursing.
            if (trueBranch == NULL) {
#ifdef TRACE_DECTREE
				controlPanel->addMessage("NULL true branch");
				throw "NULL true branch";
#endif
				return NULL;
			}
            else return trueBranch->makeDecision(bot, dt);
        } else {
            // Make sure its not null before recursing.
            if (falseBranch == NULL) {
#ifdef TRACE_DECTREE
				controlPanel->addMessage("NULL false branch");
				throw "NULL false branch";
#endif
				return NULL;
			}
            else return falseBranch->makeDecision(bot, dt);
        }
    }

	bool DecisionPtr::getBranch(RobotPlayer* bot, float dt)
	{
		if (decFuncPtr == NULL) {
#ifdef TRACE_DECTREE
			controlPanel->addMessage("NULL decFunctPtr");
			throw "NULL decFunctPtr";
#endif
		}
		return (bot->*decFuncPtr)(dt);
	}

	void DecisionPtr::runDecisionTree(DecisionPtr decTree[], RobotPlayer* bot, float dt)
	{
		// Find the decision
		DecisionTreeNode *node = decTree[0].makeDecision(bot, dt);
		void (RobotPlayer::*actFuncPtr)(float dt) = ((ActionPtr*)node)->actFuncPtr;
		if (actFuncPtr == NULL) {
#ifdef TRACE_DECTREE
			controlPanel->addMessage("NULL action function pointer in decision tree.");
			throw "NULL action function pointer in decision tree.";
#endif // TRACE_DECTREE
		}
		else {
			(bot->*actFuncPtr)(dt);
		}
	}

	// Set up the trees
	void DecisionTrees::init()
	{
		// decision tree for doUpdateMotion
		doUpdateMotionDecisions[0].decFuncPtr = &RobotPlayer::amAlive;
		doUpdateMotionDecisions[0].trueBranch = &doUpdateMotionDecisions[1];
		doUpdateMotionDecisions[0].falseBranch = &doUpdateMotionActions[0];

		doUpdateMotionDecisions[1].decFuncPtr = &RobotPlayer::shotComing;
		doUpdateMotionDecisions[1].trueBranch = &doUpdateMotionActions[1];
		doUpdateMotionDecisions[1].falseBranch = &doUpdateMotionActions[2];

		doUpdateMotionActions[0].actFuncPtr = &RobotPlayer::doNothing;
		doUpdateMotionActions[1].actFuncPtr = &RobotPlayer::evade;
		doUpdateMotionActions[2].actFuncPtr = &RobotPlayer::followPath;

		// decision tree for doUpdate, shooting
		doUpdateShootingDecisions[0].decFuncPtr = &RobotPlayer::amAlive;
		doUpdateShootingDecisions[0].trueBranch = &doUpdateShootingDecisions[1];
		doUpdateShootingDecisions[0].falseBranch = &doUpdateShootingActions[0];
		
		doUpdateShootingDecisions[1].decFuncPtr = &RobotPlayer::isFiringStatusReady;
		doUpdateShootingDecisions[1].trueBranch = &doUpdateShootingDecisions[2];
		doUpdateShootingDecisions[1].falseBranch = &doUpdateShootingActions[0];
		
		doUpdateShootingDecisions[2].decFuncPtr = &RobotPlayer::hasShotTimerElapsed;
		doUpdateShootingDecisions[2].trueBranch = &doUpdateShootingDecisions[3];
		doUpdateShootingDecisions[2].falseBranch = &doUpdateShootingActions[0];
		
		doUpdateShootingDecisions[3].decFuncPtr = &RobotPlayer::isShotCloseToTarget;
		doUpdateShootingDecisions[3].trueBranch = &doUpdateShootingDecisions[4];
		doUpdateShootingDecisions[3].falseBranch = &doUpdateShootingActions[0];
		
		doUpdateShootingDecisions[4].decFuncPtr = &RobotPlayer::isBuildingInWay;
		doUpdateShootingDecisions[4].trueBranch = &doUpdateShootingActions[0];
		doUpdateShootingDecisions[4].falseBranch = &doUpdateShootingDecisions[5];
		
		doUpdateShootingDecisions[5].decFuncPtr = &RobotPlayer::isTeammateInWay;
		doUpdateShootingDecisions[5].trueBranch = &doUpdateShootingActions[1];
		doUpdateShootingDecisions[5].falseBranch = &doUpdateShootingActions[2];

		doUpdateShootingActions[0].actFuncPtr = &RobotPlayer::doNothing;
		doUpdateShootingActions[1].actFuncPtr = &RobotPlayer::setShortShotTimer;
		doUpdateShootingActions[2].actFuncPtr = &RobotPlayer::shootAndResetShotTimer;
		
		// decision tree for doUpdate, dropping flags
		doUpdateDropFlagDecisions[0].decFuncPtr = &RobotPlayer::amAlive;
		doUpdateDropFlagDecisions[0].trueBranch = &doUpdateDropFlagDecisions[1];
		doUpdateDropFlagDecisions[0].falseBranch = &doUpdateDropFlagActions[0];
		
		doUpdateDropFlagDecisions[1].decFuncPtr = &RobotPlayer::isHoldingFlag;
		doUpdateDropFlagDecisions[1].trueBranch = &doUpdateDropFlagDecisions[2];
		doUpdateDropFlagDecisions[1].falseBranch = &doUpdateDropFlagActions[0];
		
		doUpdateDropFlagDecisions[2].decFuncPtr = &RobotPlayer::isFlagSticky;
		doUpdateDropFlagDecisions[2].trueBranch = &doUpdateDropFlagActions[0];
		doUpdateDropFlagDecisions[2].falseBranch = &doUpdateDropFlagDecisions[3];
		
		doUpdateDropFlagDecisions[3].decFuncPtr = &RobotPlayer::isTeamFlag;
		doUpdateDropFlagDecisions[3].trueBranch = &doUpdateDropFlagDecisions[4];
		doUpdateDropFlagDecisions[3].falseBranch = &doUpdateDropFlagActions[1];
		
		doUpdateDropFlagDecisions[4].decFuncPtr = &RobotPlayer::isMyTeamFlag;
		doUpdateDropFlagDecisions[4].trueBranch = &doUpdateDropFlagActions[1];
		doUpdateDropFlagDecisions[4].falseBranch = &doUpdateDropFlagActions[0];

		doUpdateDropFlagActions[0].actFuncPtr = &RobotPlayer::doNothing;
		doUpdateDropFlagActions[1].actFuncPtr = &RobotPlayer::dropFlag;
	}

	DecisionPtr DecisionTrees::doUpdateMotionDecisions[2];
	ActionPtr DecisionTrees::doUpdateMotionActions[3];
	DecisionPtr DecisionTrees::doUpdateShootingDecisions[6];
	ActionPtr DecisionTrees::doUpdateShootingActions[3];
	DecisionPtr DecisionTrees::doUpdateDropFlagDecisions[5];
	ActionPtr DecisionTrees::doUpdateDropFlagActions[2];

}; // end of namespace
