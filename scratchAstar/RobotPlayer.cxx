/* bzflag
 * Copyright (c) 1993-2018 Tim Riker
 *
 * This package is free software;  you can redistribute it and/or
 * modify it under the terms of the license found in the file
 * named COPYING that should have accompanied this file.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 *
 */

/* lines added by David Chin */
//#define TRACE
//#define TRACE2
//#define TRACE3
//#define TRACE_ASTAR
//#define TRACE_DECTREE
//#define TRACE_SHOOT_DECTREE
//#define TRACE_MOTION_DECTREE
//#define TRACE_FLAG_DECTREE
/* end of lines added by David Chin */

// interface header
#include "RobotPlayer.h"

// common implementation headers
#include "BZDBCache.h"

// local implementation headers
#include "World.h"
#include "Intersect.h"
#include "TargetingUtils.h"

/* lines added by David Chin */
#include "playing.h" // needed for numFlags, controlPanel, serverLink
#include "Roster.h" // needed for robots[]
#include "BZDBCache.h" // needed for worldSize, tankRadius
#include "Astar.h" // needed for aStarSearch
#include <time.h>  // needed for clock_t, clock, CLOCKS_PER_SECOND
#include "dectree.h" // needed for decision trees
/* end of lines added by David Chin */

std::vector<BzfRegion*>* RobotPlayer::obstacleList = NULL;

/* lines added by David Chin */
const float RobotPlayer::CohesionW = 1.0f;
const float RobotPlayer::SeparationW = 2.0f;
const float RobotPlayer::AlignW = 0.5f;
const float RobotPlayer::PathW = 3.0f;
/* end of lines added by David Chin */

RobotPlayer::RobotPlayer(const PlayerId& _id, const char* _name,
                         ServerLink* _server,
                         const char* _motto = "") :
    LocalPlayer(_id, _name, _motto),
    target(NULL),
    /* lines modified by David Chin */
    pathIndex(-1),
    /* end of lines modified by David Chin */
    timerForShot(0.0f),
    drivingForward(true)
{
    gettingSound = false;
    server       = _server;
}

// estimate a player's position at now+t, similar to dead reckoning
void RobotPlayer::projectPosition(const Player *targ,const float t,float &x,float &y,float &z) const
{
    double hisx=targ->getPosition()[0];
    double hisy=targ->getPosition()[1];
    double hisz=targ->getPosition()[2];
    double hisvx=targ->getVelocity()[0];
    double hisvy=targ->getVelocity()[1];
    double hisvz=targ->getVelocity()[2];
    double omega=fabs(targ->getAngularVelocity());
    double sx,sy;

    if ((targ->getStatus() & PlayerState::Falling) || fabs(omega) < 2*M_PI / 360 * 0.5)
    {
        sx=t*hisvx;
        sy=t*hisvy;
    }
    else
    {
        double hisspeed = hypotf(hisvx, hisvy);
        double alfa = omega * t;
        double r = hisspeed / fabs(omega);
        double dx = r * sin(alfa);
        double dy2 = r * (1 - cos(alfa));
        double beta = atan2(dy2, dx) * (targ->getAngularVelocity() > 0 ? 1 : -1);
        double gamma = atan2(hisvy, hisvx);
        double rho = gamma+beta;
        sx = hisspeed * t * cos(rho);
        sy = hisspeed * t * sin(rho);
    }
    x=(float)hisx+(float)sx;
    y=(float)hisy+(float)sy;
    z=(float)hisz+(float)hisvz*t;
    if (targ->getStatus() & PlayerState::Falling)
        z += 0.5f * BZDBCache::gravity * t * t;
    if (z < 0) z = 0;
}


// get coordinates to aim at when shooting a player; steps:
// 1. estimate how long it will take shot to hit target
// 2. calc position of target at that point of time
// 3. jump to 1., using projected position, loop until result is stable
void RobotPlayer::getProjectedPosition(const Player *targ, float *projpos) const
{
    double myx = getPosition()[0];
    double myy = getPosition()[1];
    double hisx = targ->getPosition()[0];
    double hisy = targ->getPosition()[1];
    double deltax = hisx - myx;
    double deltay = hisy - myy;
    double distance = hypotf(deltax,deltay) - BZDB.eval(StateDatabase::BZDB_MUZZLEFRONT) - BZDBCache::tankRadius;
    if (distance <= 0) distance = 0;
    double shotspeed = BZDB.eval(StateDatabase::BZDB_SHOTSPEED)*
                       (getFlag() == Flags::Laser ? BZDB.eval(StateDatabase::BZDB_LASERADVEL) :
                        getFlag() == Flags::RapidFire ? BZDB.eval(StateDatabase::BZDB_RFIREADVEL) :
                        getFlag() == Flags::MachineGun ? BZDB.eval(StateDatabase::BZDB_MGUNADVEL) : 1) +
                       hypotf(getVelocity()[0], getVelocity()[1]);

    double errdistance = 1.0;
    float tx, ty, tz;
    for (int tries=0 ; errdistance > 0.05 && tries < 4 ; tries++)
    {
        float t = (float)distance / (float)shotspeed;
        projectPosition(targ, t + 0.05f, tx, ty, tz); // add 50ms for lag
        double distance2 = hypotf(tx - myx, ty - myy);
        errdistance = fabs(distance2-distance) / (distance + ZERO_TOLERANCE);
        distance = distance2;
    }
    projpos[0] = tx;
    projpos[1] = ty;
    projpos[2] = tz;

    // projected pos in building -> use current pos
    if (World::getWorld()->inBuilding(projpos, 0.0f, BZDBCache::tankHeight))
    {
        projpos[0] = targ->getPosition()[0];
        projpos[1] = targ->getPosition()[1];
        projpos[2] = targ->getPosition()[2];
    }
}


void            RobotPlayer::doUpdate(float dt)
{
    LocalPlayer::doUpdate(dt);

    float tankRadius = BZDBCache::tankRadius;
    const float shotRange = BZDB.eval(StateDatabase::BZDB_SHOTRANGE);
    const float shotRadius = BZDB.eval(StateDatabase::BZDB_SHOTRADIUS);

    // fire shot if any available
    timerForShot -= dt;
    if (timerForShot < 0.0f)
        timerForShot = 0.0f;

    /* lines modified by David Chin */
      // Find the shooting decision
    aicore::DecisionPtr::runDecisionTree(aicore::DecisionTrees::doUpdateShootingDecisions, this, dt);

    // Find the drop flag decision
    aicore::DecisionPtr::runDecisionTree(aicore::DecisionTrees::doUpdateDropFlagDecisions, this, dt);
    if (getFiringStatus() != Ready)
        return;
}

    /*
     * would the shot be close to the target?
     * store distance to target in this.targetdistance
     */
bool		RobotPlayer::isShotCloseToTarget(float dt)
{
    if (!target) return false;
    const float azimuth = getAngle();
    float tankRadius = BZDBCache::tankRadius;
    const float shotRadius = BZDB.eval(StateDatabase::BZDB_SHOTRADIUS);
    float p1[3];
    getProjectedPosition(target, p1);
    const float* p2 = getPosition();
    float shootingAngle = atan2f(p1[1] - p2[1], p1[0] - p2[0]);
    if (shootingAngle < 0.0f)
        shootingAngle += (float)(2.0 * M_PI);
    float azimuthDiff = shootingAngle - azimuth;
    if (azimuthDiff > M_PI)
        azimuthDiff -= (float)(2.0 * M_PI);
    else if (azimuthDiff < -M_PI)
        azimuthDiff += (float)(2.0 * M_PI);

    targetdistance = hypotf(p1[0] - p2[0], p1[1] - p2[1]) -
        BZDB.eval(StateDatabase::BZDB_MUZZLEFRONT) - tankRadius;

    const float missby = fabs(azimuthDiff) *
        (targetdistance - BZDBCache::tankLength);
    // only shoot if we miss by less than half a tanklength and no building inbetween
    bool isShotCloseToTarget = (missby < 0.5f * BZDBCache::tankLength && p1[2] < shotRadius);
#ifdef TRACE_SHOOT_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d isShotCloseToTarget is %d", getTeam(), getId(), isShotCloseToTarget);
    controlPanel->addMessage(buffer);
#endif
    return isShotCloseToTarget;
}
/*
 * is a building in the way of the shot?
 * distance to target is assumed to be in this.targetdistance
 */
bool		RobotPlayer::isBuildingInWay(float dt)
{
    const float azimuth = getAngle();
    float pos[3] = { getPosition()[0], getPosition()[1],
                    getPosition()[2] + BZDB.eval(StateDatabase::BZDB_MUZZLEHEIGHT)
    };
    targetdir[0] = cosf(azimuth);
    targetdir[1] = sinf(azimuth);
    targetdir[2] = 0.0f;
    Ray tankRay(pos, targetdir);
    float maxdistance = targetdistance;
    bool isBuildingInWay = ShotStrategy::getFirstBuilding(tankRay, -0.5f, maxdistance);
#ifdef TRACE_SHOOT_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d isBuildingInWay is %d", getTeam(), getId(), isBuildingInWay);
    controlPanel->addMessage(buffer);
#endif
    return isBuildingInWay;
}

/*
 * is any teammate in the way of the shot?
 */
bool		RobotPlayer::isTeammateInWay(float dt)
{
    bool isTeammateInWay = false;
    const float shotRange = BZDB.eval(StateDatabase::BZDB_SHOTRANGE);

    for (int i = 0; i <= World::getWorld()->getCurMaxPlayers(); i++)
    {
        Player* p = 0;
        if (i < World::getWorld()->getCurMaxPlayers())
            p = World::getWorld()->getPlayer(i);
        else
            p = LocalPlayer::getMyTank();
        if (!p || p->getId() == getId() || validTeamTarget(p) ||
            !p->isAlive()) continue;
        float relpos[3] = { getPosition()[0] - p->getPosition()[0],
                           getPosition()[1] - p->getPosition()[1],
                           getPosition()[2] - p->getPosition()[2]
        };
        Ray ray(relpos, targetdir);
        float impact = rayAtDistanceFromOrigin(ray, 5 * BZDBCache::tankRadius);
        if (impact > 0 && impact < shotRange) isTeammateInWay = true;
    }
#ifdef TRACE_SHOOT_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d isTeammateInWay is %d", getTeam(), getId(), isTeammateInWay);
    controlPanel->addMessage(buffer);
#endif
    return isTeammateInWay;
}
/*
 * Set a short shot timer (stored in this.timerForShot)
 */
void			RobotPlayer::setShortShotTimer(float dt)
{
    timerForShot = 0.1f;
#ifdef TRACE_SHOOT_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d set short shot timer", getTeam(), getId());
    controlPanel->addMessage(buffer);
#endif
}
/*
 * Shoot at target (stored in this.target) and reset shot timer
 * (stored in this.timerForShot)
 */
void			RobotPlayer::shootAndResetShotTimer(float dt)
{
    if (fireShot())
        // separate shot by 0.2 - 0.8 sec (experimental value)
        timerForShot = float(bzfrand()) * 0.6f + 0.2f;
//#ifdef TRACE_SHOOT_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d shoot and reset shot timer", getTeam(), getId());
    controlPanel->addMessage(buffer);
//#endif
}
/*
 * is the robot tank holding a flag?
 */
bool		RobotPlayer::isHoldingFlag(float dt)
{
    FlagType* myFlag = getFlag();
    bool isHoldingFlag = (myFlag && (myFlag != Flags::Null));
#ifdef TRACE_FLAG_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d isHoldingFlag is %d", getTeam(), getId(), isHoldingFlag);
    controlPanel->addMessage(buffer);
#endif
    return isHoldingFlag;
}

/*
 * is the robot tank's flag sticky?
 */
bool		RobotPlayer::isFlagSticky(float dt)
{
    FlagType* myFlag = getFlag();
    bool isFlagSticky = (myFlag->endurance == FlagSticky);
#ifdef TRACE_FLAG_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d isFlagSticky is %d", getTeam(), getId(), isFlagSticky);
    controlPanel->addMessage(buffer);
#endif
    return isFlagSticky;
}

/*
 * is the robot tank's flag a team flag of any team?
 */
bool		RobotPlayer::isTeamFlag(float dt)
{
    FlagType* myFlag = getFlag();
    bool isTeamFlag = (myFlag->flagTeam != NoTeam);
#ifdef TRACE_FLAG_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d isTeamFlag is %d", getTeam(), getId(), isTeamFlag);
    controlPanel->addMessage(buffer);
#endif
    return isTeamFlag;
}

/*
 * is the robot tank holding its own team flag?
 */
bool		RobotPlayer::isMyTeamFlag(float dt)
{
    FlagType* myFlag = getFlag();
    bool isMyTeamFlag = (myFlag->flagTeam == getTeam());
#ifdef TRACE_FLAG_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d isMyTeamFlag is %d", getTeam(), getId(), isMyTeamFlag);
    controlPanel->addMessage(buffer);
#endif
    return isMyTeamFlag;
}

/*
 * Drop the flag that the robot tank is carrying
 */
void			RobotPlayer::dropFlag(float dt)
{
    serverLink->sendDropFlag(getId(), getPosition());
#ifdef TRACE_FLAG_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d dropping flag", getTeam(), getId());
    controlPanel->addMessage(buffer);
#endif
}

void            RobotPlayer::doUpdateMotion(float dt)
{
    /* lines modified by David Chin */
        // Find the decision
    aicore::DecisionPtr::runDecisionTree(aicore::DecisionTrees::doUpdateMotionDecisions, this, dt);
    LocalPlayer::doUpdateMotion(dt);
    /* end of lines modified by David Chin */
}

/*
 * is there is a non-invisible, non-isExpired() shot about to hit bot?
 * store the shot's azimuth in this.shotAngle
 */
bool		RobotPlayer::shotComing(float dt)
{
    bool shotComing;
    // record previous position
    const float oldAzimuth = getAngle();
    const float* oldPosition = getPosition();
    float position[3];
    position[0] = oldPosition[0];
    position[1] = oldPosition[1];
    position[2] = oldPosition[2];
    float azimuth = oldAzimuth;
    float tankAngVel = BZDB.eval(StateDatabase::BZDB_TANKANGVEL);
    float tankSpeed = BZDBCache::tankSpeed;


    // basically a clone of Roger's evasive code
    for (int t = 0; t <= World::getWorld()->getCurMaxPlayers(); t++)
    {
        Player* p = 0;
        if (t < World::getWorld()->getCurMaxPlayers())
            p = World::getWorld()->getPlayer(t);
        else
            p = LocalPlayer::getMyTank();
        if (!p || p->getId() == getId())
            continue;
        const int maxShots = p->getMaxShots();
        for (int s = 0; s < maxShots; s++)
        {
            ShotPath* shot = p->getShot(s);
            if (!shot || shot->isExpired())
                continue;
            // ignore invisible bullets completely for now (even when visible)
            if (shot->getFlag() == Flags::InvisibleBullet)
                continue;

            const float* shotPos = shot->getPosition();
            if ((fabs(shotPos[2] - position[2]) > BZDBCache::tankHeight) && (shot->getFlag() != Flags::GuidedMissile))
                continue;
            const float dist = TargetingUtils::getTargetDistance(position, shotPos);
            if (dist < 150.0f)
            {
                const float* shotVel = shot->getVelocity();
                float shotAngle = atan2f(shotVel[1], shotVel[0]);
                float shotUnitVec[2] = { cosf(shotAngle), sinf(shotAngle) };

                float trueVec[2] = { (position[0] - shotPos[0]) / dist,(position[1] - shotPos[1]) / dist };
                float dotProd = trueVec[0] * shotUnitVec[0] + trueVec[1] * shotUnitVec[1];

                if (dotProd > 0.97f)
                    shotComing = true;
            }
        }
    }
    shotComing = false;
#ifdef TRACE_MOTION_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d shotComing is %d", getTeam(), getId(), shotComing);
    controlPanel->addMessage(buffer);
#endif
    return shotComing;
}

/*
 * The robot tank should do nothing
 */
void			RobotPlayer::doNothing(float dt)
{
#ifdef TRACE_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d doing nothing", getTeam(), getId());
    controlPanel->addMessage(buffer);
#endif
}
/*
 * The robot tank should evade the incoming bullet, whose
 * azimuth is assumed to have been stored in this.shotAngle.
 */
void			RobotPlayer::evade(float dt)
{
    float azimuth = getAngle();
    float rotation;
    float rotation1 = (float)((shotAngle + M_PI / 2.0) - azimuth);
    if (rotation1 < -1.0f * M_PI) rotation1 += (float)(2.0 * M_PI);
    if (rotation1 > 1.0f * M_PI) rotation1 -= (float)(2.0 * M_PI);

    float rotation2 = (float)((shotAngle - M_PI / 2.0) - azimuth);
    if (rotation2 < -1.0f * M_PI) rotation2 += (float)(2.0 * M_PI);
    if (rotation2 > 1.0f * M_PI) rotation2 -= (float)(2.0 * M_PI);

    if (fabs(rotation1) < fabs(rotation2))
        rotation = rotation1;
    else
        rotation = rotation2;
    setDesiredSpeed(1.0f);
    setDesiredAngVel(rotation);
#ifdef TRACE_MOTION_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d evading", getTeam(), getId());
    controlPanel->addMessage(buffer);
#endif
}

/*
 * follow A* search path
 */
void			RobotPlayer::followPath(float dt)
{
    // record previous position
    const float oldAzimuth = getAngle();
    const float* oldPosition = getPosition();
    float position[3];
    position[0] = oldPosition[0];
    position[1] = oldPosition[1];
    position[2] = oldPosition[2];
    float azimuth = oldAzimuth;
    float tankAngVel = BZDB.eval(StateDatabase::BZDB_TANKANGVEL);
    float tankSpeed = BZDBCache::tankSpeed;

    TeamColor myteam = getTeam();
#ifdef TRACE2
    char buffer[128];
    sprintf(buffer, "Robot(%d)'s pathIndex=%d, AstarPath.size()=%d",
        getId(), pathIndex, (int)AstarPath.size());
    controlPanel->addMessage(buffer);
#endif
    if (dt > 0.0 && pathIndex > -1)
    {
        float distance;
        float v[2];
        float endPoint[3];
        endPoint[0] = AstarPath[pathIndex].getScaledX();
        endPoint[1] = AstarPath[pathIndex].getScaledY();
#ifdef TRACE2
        char buffer[128];
        sprintf(buffer, "Robot(%d) at (%f, %f) heading toward (%f, %f)",
            getId(), position[0], position[1], endPoint[0], endPoint[1]);
        controlPanel->addMessage(buffer);
#endif
        // find how long it will take to get to next path segment
        float path[3];
        path[0] = endPoint[0] - position[0];
        path[1] = endPoint[1] - position[1];
        distance = hypotf(path[0], path[1]);
        float tankRadius = BZDBCache::tankRadius;
        // find how long it will take to get to next path segment
        if (distance <= dt * tankSpeed + 2.0f * BZDBCache::tankRadius)
            pathIndex--;

        float centerOfMass[3];
        float cohesionV[2], separationV[3];
        int numCohesionNeighbors = computeCenterOfMass(BZDBCache::tankRadius * 15.0f, centerOfMass);
        // uncomment out one of the following 2 code sections to get either
        // lines 1-7: repulsion from center of mass of neighbors
        // line 9: inverse square law repulsion from each neighbor
        //float separation[3];
        //int numSeparationNeighbors = computeCenterOfMass(BZDBCache::tankRadius * 5.0f, separation);
        //if (numSeparationNeighbors) {
        //    separationV[0] = position[0] - separation[0];
        //    separationV[1] = position[1] - separation[1];
        //} else
        //    separationV[0] = separationV[1] = 0;
        // above lines for repulsion from CoM, line below for repulsion from each neighbor
        int numSeparationNeighbors = computeRepulsion(BZDBCache::tankRadius * 5.0f, separationV);
        float alignAzimuth;
        float align[3] = { 0.0f, 0.0f, 0.0f };
        int numAlignNeighbors = computeAlign(BZDBCache::tankRadius * 15.0f, align, &alignAzimuth);
        if (numCohesionNeighbors) {
            cohesionV[0] = centerOfMass[0] - position[0];
            cohesionV[1] = centerOfMass[1] - position[1];
            distance = hypotf(cohesionV[0], cohesionV[1]);
            cohesionV[0] /= distance;
            cohesionV[1] /= distance;
        }
        else
            cohesionV[0] = cohesionV[1] = 0;
        //const float* endPoint = path[pathIndex].get();
        // find how long it will take to get to next path segment
        //v[0] = endPoint[0] - position[0];
        //v[1] = endPoint[1] - position[1];
        v[0] = CohesionW * cohesionV[0] + SeparationW * separationV[0] + AlignW * align[0] + PathW * path[0];
        v[1] = CohesionW * cohesionV[1] + SeparationW * separationV[1] + AlignW * align[1] + PathW * path[1];
        float weightSum = CohesionW + SeparationW + AlignW + PathW;
        v[0] /= weightSum;
        v[1] /= weightSum;
        distance = hypotf(v[0], v[1]);
        //float tankRadius = BZDBCache::tankRadius;
        // smooth path a little by turning early at corners, might get us stuck, though
        //if (distance <= 2.5f * tankRadius)
            //pathIndex++;

        float segmentAzimuth = atan2f(v[1], v[0]);
        float azimuthDiff = segmentAzimuth - azimuth;
        if (azimuthDiff > M_PI) azimuthDiff -= (float)(2.0 * M_PI);
        else if (azimuthDiff < -M_PI) azimuthDiff += (float)(2.0 * M_PI);
        if (fabs(azimuthDiff) > 0.01f)
        {
            // drive backward when target is behind, try to stick to last direction
            if (drivingForward)
                drivingForward = fabs(azimuthDiff) < M_PI / 2 * 0.9 ? true : false;
            else
                drivingForward = fabs(azimuthDiff) < M_PI / 2 * 0.3 ? true : false;
            setDesiredSpeed(drivingForward ? 1.0f : -1.0f);
            // set desired turn speed
            if (azimuthDiff >= dt * tankAngVel)
                setDesiredAngVel(1.0f);
            else if (azimuthDiff <= -dt * tankAngVel)
                setDesiredAngVel(-1.0f);
            else
                setDesiredAngVel(azimuthDiff / dt / tankAngVel);
        }
        else
        {
            drivingForward = true;
            // tank doesn't turn while moving forward
            setDesiredAngVel(0.0f);
            // find how long it will take to get to next path segment
            if (distance <= dt * tankSpeed)
            {
                /* lines modified by David Chin */
                                    //pathIndex++;
                /* end of lines modified by David Chin */
                                    // set desired speed
                setDesiredSpeed(distance / dt / tankSpeed);
            }
            else
                setDesiredSpeed(1.0f);
        }
        /* lines added by David Chin */
#ifdef TRACE3
        char buffer[128];
        sprintf(buffer, "bot(%d) at (%f, %f) -> (%f, %f), d = %f, dt = %f, v = (%f, %f)",
            getId(), position[0], position[1], endPoint[0], endPoint[1], distance, dt * tankSpeed, v[0], v[1]);
        controlPanel->addMessage(buffer);
#endif
    }
#ifdef TRACE_MOTION_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d following path", getTeam(), getId());
    controlPanel->addMessage(buffer);
#endif
}

/*
 * is firingStatus == Ready?
 */
bool		RobotPlayer::isFiringStatusReady(float dt)
{
    bool isFiringStatusReady = getFiringStatus() == Ready;
#ifdef TRACE_SHOOT_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d isFiringStatusReady is %d", getTeam(), getId(), isFiringStatusReady);
    controlPanel->addMessage(buffer);
#endif
    return isFiringStatusReady;
}

/*
 * Has the shot timer elapsed (fallen to <= 0)?
 */
bool		RobotPlayer::hasShotTimerElapsed(float dt)
{
    bool hasShotTimerElapsed = timerForShot <= 0.0f;
#ifdef TRACE_SHOOT_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d isFiringStatusReady is %d", getTeam(), getId(), hasShotTimerElapsed);
    controlPanel->addMessage(buffer);
#endif
    return hasShotTimerElapsed;
}


void            RobotPlayer::explodeTank()
{
    LocalPlayer::explodeTank();
    target = NULL;
    path.clear();
/* lines added by David Chin */
    AstarPath.clear();
/* end of lines added by David Chin */
}

void            RobotPlayer::restart(const float* pos, float _azimuth)
{
    LocalPlayer::restart(pos, _azimuth);
    // no target
    path.clear();
    target = NULL;
/* lines added/modified by David Chin */
    AstarPath.clear();
    pathIndex = -1;
/* end of lines added/modified by David Chin */

}

float           RobotPlayer::getTargetPriority(const
        Player* _target) const
{
    // don't target teammates or myself
    if (!this->validTeamTarget(_target))
        return 0.0f;

    // go after closest player
    // FIXME -- this is a pretty stupid heuristic
    const float worldSize = BZDBCache::worldSize;
    const float* p1 = getPosition();
    const float* p2 = _target->getPosition();

    float basePriority = 1.0f;
    // give bonus to non-paused player
    if (!_target->isPaused())
        basePriority += 2.0f;
    // give bonus to non-deadzone targets
    if (obstacleList)
    {
        float nearest[2];
        const BzfRegion* targetRegion = findRegion (p2, nearest);
        if (targetRegion && targetRegion->isInside(p2))
            basePriority += 1.0f;
    }
    return basePriority
           - 0.5f * hypotf(p2[0] - p1[0], p2[1] - p1[1]) / worldSize;
}

void            RobotPlayer::setObstacleList(std::vector<BzfRegion*>*
        _obstacleList)
{
    obstacleList = _obstacleList;
/* lines added by David Chin */
  aicore::DecisionTrees::init();
/* end of lines added by David Chin */
}

const Player*       RobotPlayer::getTarget() const
{
    return target;
}

void            RobotPlayer::setTarget(const Player* _target)
{
/* lines added/modified by David Chin */
    //static int mailbox = 0;

    //path.clear();
    target = _target;
    //if (!target) return;

    TeamColor myteam = getTeam();
    float goalPos[3];
    if (myTeamHoldingOpponentFlag())
        findHomeBase(myteam, goalPos);
    else
        findOpponentFlag(goalPos);

    AStarNode goalNode(goalPos);
    if (!AstarPath.empty() && goalNode == pathGoalNode)
        return; // same goal so no need to plan again

#ifdef TRACE_ASTAR
    clock_t start_s = clock();
#endif
    //float start[3] = { 12.96f, -30.24, 0.0f };
    //float goal[3] = { 0.0f, -190.08, 0.0f };
    //AStarGraph::aStarSearch(start, goal, AstarPath);
    AStarGraph::aStarSearch(getPosition(), goalPos, AstarPath);
#ifdef TRACE_ASTAR
    clock_t stop_s = clock();
    float sum = (float)(stop_s - start_s) / CLOCKS_PER_SEC;
    char buffer[128];
    sprintf(buffer, "\nA* search took %f seconds", sum);
    controlPanel->addMessage(buffer);
#endif
    if (!AstarPath.empty()) {
        pathGoalNode.setX(AstarPath[0].getX());
        pathGoalNode.setY(AstarPath[0].getY());
        pathIndex = AstarPath.size() - 2; // last index is start node
    }
#ifdef TRACE2
    sprintf(buffer, "\nNumber of A* nodes: %d\nPath coordinates: \n[ ", aStarPath.size());
    controlPanel->addMessage(buffer);
    for (int a = 0; a < AstarPath.size(); a++) {
        sprintf(buffer, "[%d, %d]; ", AstarPath[a].getX(), AstarPath[a].getY());
        controlPanel->addMessage(buffer);
    }
    controlPanel->addMessage(" ]\n\n");
#endif

/*
    // work backwards (from target to me)
    float proj[3];
    getProjectedPosition(target, proj);
    const float *p1 = proj;
    const float* p2 = getPosition();
    float q1[2], q2[2];
    BzfRegion* headRegion = findRegion(p1, q1);
    BzfRegion* tailRegion = findRegion(p2, q2);
    if (!headRegion || !tailRegion)
    {
        // if can't reach target then forget it
        return;
    }

    mailbox++;
    headRegion->setPathStuff(0.0f, NULL, q1, mailbox);
    RegionPriorityQueue queue;
    queue.insert(headRegion, 0.0f);
    BzfRegion* next;
    while (!queue.isEmpty() && (next = queue.remove()) != tailRegion)
        findPath(queue, next, tailRegion, q2, mailbox);

    // get list of points to go through to reach the target
    next = tailRegion;
    do
    {
        p1 = next->getA();
        path.push_back(p1);
        next = next->getTarget();
    }
    while (next && next != headRegion);
    if (next || tailRegion == headRegion)
        path.push_back(q1);
    else
        path.clear();
    pathIndex = 0;
*/
/* end of lines added/modified by David Chin */
}

BzfRegion*      RobotPlayer::findRegion(const float p[2],
                                        float nearest[2]) const
{
    nearest[0] = p[0];
    nearest[1] = p[1];
    const int count = obstacleList->size();
    for (int o = 0; o < count; o++)
        if ((*obstacleList)[o]->isInside(p))
            return (*obstacleList)[o];

    // point is outside: find nearest region
    float      distance      = maxDistance;
    BzfRegion* nearestRegion = NULL;
    for (int i = 0; i < count; i++)
    {
        float currNearest[2];
        float currDistance = (*obstacleList)[i]->getDistance(p, currNearest);
        if (currDistance < distance)
        {
            nearestRegion = (*obstacleList)[i];
            distance = currDistance;
            nearest[0] = currNearest[0];
            nearest[1] = currNearest[1];
        }
    }
    return nearestRegion;
}

float           RobotPlayer::getRegionExitPoint(
    const float p1[2], const float p2[2],
    const float a[2], const float targetPoint[2],
    float mid[2], float& priority)
{
    float b[2];
    b[0] = targetPoint[0] - a[0];
    b[1] = targetPoint[1] - a[1];
    float d[2];
    d[0] = p2[0] - p1[0];
    d[1] = p2[1] - p1[1];

    float vect = d[0] * b[1] - d[1] * b[0];
    float t    = 0.0f;  // safe value
    if (fabs(vect) > ZERO_TOLERANCE)
    {
        // compute intersection along (p1,d) with (a,b)
        t = (a[0] * b[1] - a[1] * b[0] - p1[0] * b[1] + p1[1] * b[0]) / vect;
        if (t > 1.0f)
            t = 1.0f;
        else if (t < 0.0f)
            t = 0.0f;
    }

    mid[0] = p1[0] + t * d[0];
    mid[1] = p1[1] + t * d[1];

    const float distance = hypotf(a[0] - mid[0], a[1] - mid[1]);
    // priority is to minimize distance traveled and distance left to go
    priority = distance + hypotf(targetPoint[0] - mid[0], targetPoint[1] - mid[1]);
    // return distance traveled
    return distance;
}

void            RobotPlayer::findPath(RegionPriorityQueue& queue,
                                      BzfRegion* region,
                                      BzfRegion* targetRegion,
                                      const float targetPoint[2],
                                      int mailbox)
{
    const int numEdges = region->getNumSides();
    for (int i = 0; i < numEdges; i++)
    {
        BzfRegion* neighbor = region->getNeighbor(i);
        if (!neighbor) continue;

        const float* p1 = region->getCorner(i).get();
        const float* p2 = region->getCorner((i+1)%numEdges).get();
        float mid[2], priority;
        float total = getRegionExitPoint(p1, p2, region->getA(),
                                         targetPoint, mid, priority);
        priority += region->getDistance();
        if (neighbor == targetRegion)
            total += hypotf(targetPoint[0] - mid[0], targetPoint[1] - mid[1]);
        total += region->getDistance();
        if (neighbor->test(mailbox) || total < neighbor->getDistance())
        {
            neighbor->setPathStuff(total, region, mid, mailbox);
            queue.insert(neighbor, priority);
        }
    }
}

/* lines added by David Chin */
/*
 * Return the number of teammates (not including self)
 * within neighborhoodSize distance of self; also return
 * their center of mass position (in cmOut)
 */
int		RobotPlayer::computeCenterOfMass(float neighborhoodSize, float cmOut[3])
{
	cmOut[0] = cmOut[1] = cmOut[2] = 0.0f;
	const float* mypos = getPosition();
	int numTeammates = 0;
	TeamColor myTeam = getTeam();
#ifdef TRACE
	char buffer[128];
	sprintf (buffer, "my (id=%d) team color is %d", getId(), myTeam);
	controlPanel->addMessage(buffer);
	sprintf (buffer, "curMaxPlayers() is  %d", World::getWorld()->getCurMaxPlayers());
	controlPanel->addMessage(buffer);
#endif
	for (int i=0; i <= World::getWorld()->getCurMaxPlayers(); i++)
	{
		Player* p = NULL;
		const float* pos; // p's position
		double distance = 0; // distance from p to this robot
		if (i < World::getWorld()->getCurMaxPlayers())
			p = World::getWorld()->getPlayer(i);
		else
			p = LocalPlayer::getMyTank();

#ifdef TRACE
		if(p) sprintf (buffer, "getPlayer(%d), id=%d has team color %d",
			i, p->getId(), p->getTeam());
		controlPanel->addMessage(buffer);
#endif
		if (p && p->getTeam() == myTeam && p->getId() != getId()) {
			pos = p->getPosition();
			double deltax = pos[0] - mypos[0];
			double deltay = pos[1] - mypos[1];
			distance = hypotf(deltax,deltay);
#ifdef TRACE
			sprintf (buffer, "getPlayer(%d), id=%d has location (%f, %f, %f)",
				i, p->getId(), pos[0], pos[1], pos[2]);
			controlPanel->addMessage(buffer);
			sprintf (buffer, "distance = %f, neighborhood = %f",
				distance, neighborhoodSize);
			controlPanel->addMessage(buffer);
#endif
			if (distance < neighborhoodSize) {
				numTeammates++;
				cmOut[0] += pos[0];
				cmOut[1] += pos[1];
				cmOut[2] += pos[2];
			}
		}
	}
#ifdef TRACE
	sprintf (buffer, "numTeammates = %d",
		numTeammates);
	controlPanel->addMessage(buffer);
#endif
	if (numTeammates) {
		cmOut[0] /= numTeammates;
		cmOut[1] /= numTeammates;
		cmOut[2] /= numTeammates;	
	}
	return numTeammates;
}

/*
 * Return the number of teammates (not including self)
 * within neighborhoodSize distance of self; also return
 * the inverse square repulsion of those teammates in repulseOut[3]
 */
int		RobotPlayer::computeRepulsion(float neighborhoodSize, float repulseOut[3])
{
	repulseOut[0] = repulseOut[1] = repulseOut[2] = 0.0f;
	const float* mypos = getPosition();
	int numTeammates = 0;
	TeamColor myTeam = getTeam();
#ifdef TRACE
	char buffer[128];
	sprintf (buffer, "my (id=%d) team color is %d", getId(), myTeam);
	controlPanel->addMessage(buffer);
	sprintf (buffer, "curMaxPlayers() is  %d", World::getWorld()->getCurMaxPlayers());
	controlPanel->addMessage(buffer);
#endif
	for (int i=0; i <= World::getWorld()->getCurMaxPlayers(); i++)
	{
		Player* p = NULL;
		const float* pos; // p's position
		float direction[3]; // flee direction
		float distance = 0; // distance from p to this robot
		if (i < World::getWorld()->getCurMaxPlayers())
			p = World::getWorld()->getPlayer(i);
		else
			p = LocalPlayer::getMyTank();

#ifdef TRACE
		if(p) sprintf (buffer, "getPlayer(%d), id=%d has team color %d",
			i, p->getId(), p->getTeam());
		controlPanel->addMessage(buffer);
#endif
		if (p && p->getTeam() == myTeam && p->getId() != getId()) {
			pos = p->getPosition();
			direction[0] = mypos[0] - pos[0];
			direction[1] = mypos[1] - pos[1];
			distance = hypotf(direction[0],direction[1])/BZDBCache::tankRadius;
			if (distance == 0.0) {
				distance = 0.0001f; // in case tanks on top of each other
				direction[0] = float(bzfrand());
				direction[1] = float(bzfrand());
			}
#ifdef TRACE
			sprintf (buffer, "getPlayer(%d), id=%d has location (%f, %f, %f)",
				i, p->getId(), pos[0], pos[1], pos[2]);
			controlPanel->addMessage(buffer);
			sprintf (buffer, "distance = %f, neighborhood = %f",
				distance, neighborhoodSize);
			controlPanel->addMessage(buffer);
#endif
			if (distance < neighborhoodSize) {
				numTeammates++;
				// normalize to unit vector and inverse square law
				float dd = distance * distance * distance;
				repulseOut[0] += direction[0]/dd;
				repulseOut[1] += direction[1]/dd;
				repulseOut[2] += direction[2]/dd;
			}
		}
	}
#ifdef TRACE
	sprintf (buffer, "numTeammates = %d",
		numTeammates);
	controlPanel->addMessage(buffer);
#endif
	return numTeammates;
}

/**
 * Return the number of teammates (not including self)
 * within neighborhoodSize distance of self; also return
 * their average velocity (in avVOut) and their average angle (in avAzimuthOut)
 */
int		RobotPlayer::computeAlign(float neighborhoodSize, float avVOut[3], float* avAzimuthOut)
{
	avVOut[0] = avVOut[1] = avVOut[2] = 0.0f;
	const float* mypos = getPosition();
	int numTeammates = 0;
	TeamColor myTeam = getTeam();
	*avAzimuthOut = 0;
#ifdef TRACE
	char buffer[128];
	sprintf (buffer, "my (id=%d) team color is %d", getId(), myTeam);
	controlPanel->addMessage(buffer);
	sprintf (buffer, "curMaxPlayers() is  %d", World::getWorld()->getCurMaxPlayers());
	controlPanel->addMessage(buffer);
#endif
	for (int i=0; i <= World::getWorld()->getCurMaxPlayers(); i++)
	{
		Player* p = NULL;
		const float* pos; // position of p
		const float* v; // velocity of v
		//const float* azimuth; // angle of v
		double distance = 0; // distance from p to this robot
		if (i < World::getWorld()->getCurMaxPlayers())
			p = World::getWorld()->getPlayer(i);
		else
			p = LocalPlayer::getMyTank();

#ifdef TRACE
		if(p) sprintf (buffer, "getPlayer(%d), id=%d has team color %d",
			i, p->getId(), p->getTeam());
		controlPanel->addMessage(buffer);
#endif
		if (p && p->getTeam() == myTeam && p->getId() != getId()) {
			pos = p->getPosition();
			double deltax = pos[0] - mypos[0];
			double deltay = pos[1] - mypos[1];
			distance = hypotf(deltax,deltay);
			v = p->getVelocity();
#ifdef TRACE
			sprintf (buffer, "getPlayer(%d), id=%d has velocity (%f, %f, %f)",
				i, p->getId(), v[0], v[1], v[2]);
			controlPanel->addMessage(buffer);
			sprintf (buffer, "distance = %f, neighborhood = %f",
				distance, neighborhoodSize);
			controlPanel->addMessage(buffer);
#endif
			if (distance < neighborhoodSize) {
				numTeammates++;
				avVOut[0] += v[0];
				avVOut[1] += v[1];
				avVOut[2] += v[2];
				*avAzimuthOut += p->getAngle();
			}
		}
	}
#ifdef TRACE
	sprintf (buffer, "numTeammates = %d",
		numTeammates);
	controlPanel->addMessage(buffer);
#endif
	if (numTeammates) {
		avVOut[0] /= numTeammates;
		avVOut[1] /= numTeammates;
		avVOut[2] /= numTeammates;
		*avAzimuthOut /= numTeammates;
	}
	return numTeammates;
}

/*
 * Return the location (in location[3]) of the first home base for team color teamColor
 */
void		RobotPlayer::findHomeBase(TeamColor teamColor, float location[3])
{
	World* world = World::getWorld();
	if(!world->allowTeamFlags()) return;
	const float* baseParms = world->getBase(teamColor, 0);
#ifdef TRACE2
	char buffer[128];
	sprintf (buffer, "Base for color %d is at (%f, %f, %f)",
		teamColor, baseParms[0], baseParms[1], baseParms[2]);
	controlPanel->addMessage(buffer);
#endif
	location[0] = baseParms[0]; 
	location[1] = baseParms[1];
	location[2] = baseParms[2];
}

/*
 * Return true if any player on my team
 * is holding an opponent team flag, and false otherwise
 */
bool		RobotPlayer::myTeamHoldingOpponentFlag(void)
{
	TeamColor myTeamColor = getTeam();
	if (!World::getWorld()->allowTeamFlags()) return false;
	for (int i = 0; i < numFlags; i++) {
		Flag& flag = World::getWorld()->getFlag(i);
		TeamColor flagTeamColor = flag.type->flagTeam;
		if (flagTeamColor != NoTeam && flagTeamColor != myTeamColor
			&& flag.status == FlagOnTank) {
			PlayerId ownerId = flag.owner;
#ifdef TRACE2
			char buffer[128];
			sprintf (buffer, "Looking for a Player with id=%d",
				ownerId);
			controlPanel->addMessage(buffer);
#endif
			Player* p = lookupLocalPlayer(ownerId);
			if (p && (p->getTeam() == myTeamColor)) {
#ifdef TRACE2
				sprintf (buffer, "Player id=%d, TeamColor=%d holds flag %d, robots[0]=%d",
					p->getId(), p->getTeam(), myTeamColor, robots[0]->getId());
				controlPanel->addMessage(buffer);
#endif
				return true;
			}
		}
	}
#ifdef TRACE2
    char buffer[128];
    sprintf(buffer, "Robot(%d)'s team is not holding a team flag",
        getId());
    controlPanel->addMessage(buffer);
#endif
	return false;
}

/*
 * Find any opponent flag and return its location
 */
void		RobotPlayer::findOpponentFlag(float location[3])
{
	TeamColor myTeamColor = getTeam();
	if (!World::getWorld()->allowTeamFlags()) return;
	for (int i = 0; i < numFlags; i++) {
		Flag& flag = World::getWorld()->getFlag(i);
		TeamColor flagTeamColor = flag.type->flagTeam;
		if (flagTeamColor != NoTeam && flagTeamColor != myTeamColor) {
			location[0] = flag.position[0];
			location[1] = flag.position[1];
			location[2] = flag.position[2];
#ifdef TRACE2
			char buffer[128];
			sprintf (buffer, "Robot(%d) found a flag at (%f, %f, %f)",
                     getId(), location[0], location[1], location[2]);
			controlPanel->addMessage(buffer);
#endif
			return;
		}
	}
}

/*
 * Given a PlayerId, find the corresponding local Player
 */
Player*		RobotPlayer::lookupLocalPlayer(PlayerId id)
{
	for (int i=0; i <= World::getWorld()->getCurMaxPlayers(); i++)
	{
		Player* p = NULL;
		if (i < World::getWorld()->getCurMaxPlayers())
			p = World::getWorld()->getPlayer(i);
		else
			p = LocalPlayer::getMyTank();
		if (p && p->getId() == id) return p;
	}
	return NULL;
}

/*
 * same as Player::isAlive(), needed to match type of decision tree
 */
bool		RobotPlayer::amAlive(float dt)
{
	bool alive = isAlive();
#ifdef TRACE_DECTREE
    char buffer[128];
    sprintf(buffer, "R%d-%d amAlive is %d", getTeam(), getId(), alive);
    controlPanel->addMessage(buffer);
#endif
    return alive;
}

/* end of lines added by David Chin */

// Local Variables: ***
// mode: C++ ***
// tab-width: 4 ***
// c-basic-offset: 4 ***
// indent-tabs-mode: nil ***
// End: ***
// ex: shiftwidth=4 tabstop=4
