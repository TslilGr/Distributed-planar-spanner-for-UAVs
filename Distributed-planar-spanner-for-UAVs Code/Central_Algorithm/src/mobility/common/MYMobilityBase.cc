/* -*- mode:c++ -*- ********************************************************
 * file:        MYMobilityBase.cc
 *
 * author:      Daniel Willkomm, Andras Varga, Zoltan Bojthe
 *
 * copyright:   (C) 2004 Telecommunication Networks Group (TKN) at
 *              Technische Universitaet Berlin, Germany.
 *
 *              (C) 2005 Andras Varga
 *              (C) 2011 Zoltan Bojthe
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 ***************************************************************************
 * part of:     framework implementation developed by tkn
 **************************************************************************/


#include "MYMobilityBase.h"
#include "FWMath.h"


Register_Abstract_Class(MYMobilityBase);

//MOSHE: get the number from *s in destValue
static bool parseIntTo(const char *s, double& destValue)
{
    if (!s || !*s)
        return false;

    char *endptr;
    int value = strtol(s, &endptr, 10);

    if (*endptr)
        return false;

    destValue = value;
    return true;
}

//MOSHE: check if double is in range pf double?!?!?
static bool isFiniteNumber(double value)
{
    return value <= DBL_MAX && value >= -DBL_MAX;
}

MYMobilityBase::MYMobilityBase()
{
    visualRepresentation = NULL;
    constraintAreaMin = Coord::ZERO;
    constraintAreaMax = Coord::ZERO;
    lastPosition = Coord::ZERO;
}

void MYMobilityBase::initialize(int stage)
{
    cSimpleModule::initialize(stage);

    EV_TRACE << "initializing MYMobilityBase stage " << stage << endl;
    if (stage == 0)
    {
        constraintAreaMin.x = par("constraintAreaMinX");
        constraintAreaMin.y = par("constraintAreaMinY");
        constraintAreaMin.z = par("constraintAreaMinZ");
        constraintAreaMax.x = par("constraintAreaMaxX");
        constraintAreaMax.y = par("constraintAreaMaxY");
        constraintAreaMax.z = par("constraintAreaMaxZ");
        visualRepresentation = findVisualRepresentation();


        if (visualRepresentation) {
            const char *s = visualRepresentation->getDisplayString().getTagArg("p", 2);
            if (s && *s)
                error("The coordinates of '%s' are invalid. Please remove automatic arrangement"
                        " (3rd argument of 'p' tag) from '@display' attribute.", visualRepresentation->getFullPath().c_str());
        }
    }
    else if (stage == 1)
    {
        initializePosition();
    }
}

void MYMobilityBase::initializePosition()
{
    setInitialPosition();               //sets initial position form initialXY in .ini or according to function calculation (now getRandomPosition in ::setInitialPosition )
    checkPosition();                    //making sure this a valid position according to playground
    emitMobilityStateChangedSignal();   //signal about the change of the mobility
    updateVisualRepresentation();       //MOSHE: set the visual object
}

void MYMobilityBase::setInitialPosition()
{
    // reading the coordinates from omnetpp.ini makes predefined scenarios a lot easier
    bool filled = false;
    if (hasPar("initFromDisplayString") && par("initFromDisplayString").boolValue() && visualRepresentation)
    {
        filled = parseIntTo(visualRepresentation->getDisplayString().getTagArg("p", 0), lastPosition.x)
                                      && parseIntTo(visualRepresentation->getDisplayString().getTagArg("p", 1), lastPosition.y);
        if (filled)
            lastPosition.z = 0;

    }
    // not all mobility models have "initialX", "initialY" and "initialZ" parameters
    else if (hasPar("initialX") && hasPar("initialY") && hasPar("initialZ"))
    {
        lastPosition.x = par("initialX");
        lastPosition.y = par("initialY");
        lastPosition.z = par("initialZ");
        filled = true;
    }
    if (!filled)
        //for now not used cuz hasPar so enters to above else if statement, but if when wont be set from initialXY them sould be smt else insted of "getRandomPosition();" in next line
        lastPosition = getRandomPosition();
}

//MOSHE: check if position is number and if in map
void MYMobilityBase::checkPosition()
{
    if (!isFiniteNumber(lastPosition.x) || !isFiniteNumber(lastPosition.y) || !isFiniteNumber(lastPosition.z))
        throw cRuntimeError("Mobility position is not a finite number after initialize (x=%g,y=%g,z=%g)", lastPosition.x, lastPosition.y, lastPosition.z);
    if (isOutside())
        throw cRuntimeError("Mobility position (x=%g,y=%g,z=%g) is outside the constraint area (%g,%g,%g - %g,%g,%g)",
                lastPosition.x, lastPosition.y, lastPosition.z,
                constraintAreaMin.x, constraintAreaMin.y, constraintAreaMin.z,
                constraintAreaMax.x, constraintAreaMax.y, constraintAreaMax.z);
}

void MYMobilityBase::handleMessage(cMessage * message)
{
    if (message->isSelfMessage())
        handleSelfMessage(message);
    else
        throw cRuntimeError("Mobility modules can only receive self messages");
}

//MOSHE: set visual position
void MYMobilityBase::updateVisualRepresentation()
{
    EV_INFO << "current position = " << lastPosition << endl;
    if (ev.isGUI() && visualRepresentation)
    {
        visualRepresentation->getDisplayString().setTagArg("p", 0, (long)lastPosition.x);
        visualRepresentation->getDisplayString().setTagArg("p", 1, (long)lastPosition.y);
        /* && visualRepresentation->getDisplayString().getTagArg("p",0)!=0
                                                        && visualRepresentation->getDisplayString().getTagArg("p",1)!=0*/
        if (strcmp(getParentModule()->getName(),"host") && strcmp(getParentModule()->getName(),"cc")
                &&  strcmp(visualRepresentation->getDisplayString().getTagArg("p",0),"0")
                &&  strcmp(visualRepresentation->getDisplayString().getTagArg("p",1),"0")){
            visualRepresentation->getDisplayString().setTagArg("r",0, 100);
            visualRepresentation->getDisplayString().setTagArg("r",2, "black");
            visualRepresentation->getDisplayString().setTagArg("r",3, 2);
        }
    }
}

//MOSHE: emit Mobility State Changed Signal
void MYMobilityBase::emitMobilityStateChangedSignal()
{
    emit(mobilityStateChangedSignal, this);
}

//MOSHE: get random position
Coord MYMobilityBase::getRandomPosition()
{
    Coord p;
    p.x = uniform(constraintAreaMin.x, constraintAreaMax.x);
    p.y = uniform(constraintAreaMin.y, constraintAreaMax.y);
    p.z = uniform(constraintAreaMin.z, constraintAreaMax.z);


    return p;
}

////////////////////////////////////////////////////////////////////////////////////////////
//need to calculate the new position of drone according to all its hosts, the center of its SEC
//so need to get the locations of its host neighbors (not the drones locations)
//called from MYConstSpeedMobilityDrone::setTargetPosition()
//AS LONG AS DISTANCE IS THE SAME LENGHT OF TRAVEL IS 0 SO WE ADD SOME TIME IN MYConstSpeedMobilityDrone::setTargetPosition()
//as a start need to print here hosts pos according to the drone[], and then to calc drones new pos and return p
Coord MYMobilityBase::calcMYDronePosition()
{
    //EV_INFO << "XXXXXXXXXXXXXXXXXXXXXXXX MYMobilityBase::calcMYDronePosition before emit XXXXXXXXXXXXXXXXXXXXXXXX " << endl;
    //emitRequestDroneNextPositoinSignal();
    //EV_INFO << "XXXXXXXXXXXXXXXXXXXXXXXX MYMobilityBase::calcMYDronePosition after emit XXXXXXXXXXXXXXXXXXXXXXXX " << endl;
    Coord p;
    p.x = par("initialX");
    p.y = par("initialY");
    p.z = par("initialZ");

    //EV_INFO << "XXXXXXXXXXXXXXXXXXXXXXXX MYMobilityBase::calcMYDronePosition Name XXXXXXXXXXXXXXXXXXXXXXXX " << this->getParentModule()->getFullName() << endl;

    return p;
}

//MOSHE: check if inside map
bool MYMobilityBase::isOutside()
{
    return lastPosition.x < constraintAreaMin.x || lastPosition.x > constraintAreaMax.x
            || lastPosition.y < constraintAreaMin.y || lastPosition.y > constraintAreaMax.y
            || lastPosition.z < constraintAreaMin.z || lastPosition.z > constraintAreaMax.z;
}

//This function check if the drone entered to a No-Fly-Zone
//returns: int - the number of the NFZ which (x,y) are inside, else -1
int MYMobilityBase::isAtNFZ(int x, int y)                       //if entered to No-Fly-Zone
{
    int numOfNFZ=getParentModule()->getParentModule()->par("numNFZ");
    int NFZ_X,NFZ_Y,NFZ_Radius;
    for(int i=0;i<numOfNFZ;i++)
    {
        NFZ_X=getParentModule()->getParentModule()->getSubmodule("NFZ",i)->par("x");
        NFZ_Y=getParentModule()->getParentModule()->getSubmodule("NFZ",i)->par("y");
        NFZ_Radius=getParentModule()->getParentModule()->getSubmodule("NFZ",i)->par("radius");
        if (euclidean_distance(x,y,NFZ_X,NFZ_Y) < NFZ_Radius)
            return i;
    }
    return -1;
}

//MOSHE: return sqrt(pow(x1-x2,2)+pow(y1-y2,2)
double MYMobilityBase::euclidean_distance(int x1, int y1, int x2, int y2){
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}

static int reflect(double min, double max, double &coordinate, double &speed)
{
    double size = max - min;
    double value = coordinate - min;
    int sign = 1 - FWMath::modulo(floor(value / size), 2) * 2;
    ASSERT(sign == 1 || sign == -1);
    coordinate = FWMath::modulo(sign * value, size) + min;
    speed = sign * speed;
    return sign;
}

void MYMobilityBase::reflectIfOutside(Coord& targetPosition, Coord& speed, double angle)   //was double& angle
{
    int sign;
    double dummy;
    if (lastPosition.x < constraintAreaMin.x || constraintAreaMax.x < lastPosition.x) {
        sign = reflect(constraintAreaMin.x, constraintAreaMax.x, lastPosition.x, speed.x);
        reflect(constraintAreaMin.x, constraintAreaMax.x, targetPosition.x, dummy);
        angle = 90 + sign * (angle - 90);
    }
    if (lastPosition.y < constraintAreaMin.y || constraintAreaMax.y < lastPosition.y) {
        sign = reflect(constraintAreaMin.y, constraintAreaMax.y, lastPosition.y, speed.y);
        reflect(constraintAreaMin.y, constraintAreaMax.y, targetPosition.y, dummy);
        angle = sign * angle;
    }
    if (lastPosition.z < constraintAreaMin.z || constraintAreaMax.z < lastPosition.z) {
        sign = reflect(constraintAreaMin.z, constraintAreaMax.z, lastPosition.z, speed.z);
        reflect(constraintAreaMin.z, constraintAreaMax.z, targetPosition.z, dummy);
        // NOTE: angle is not affected
    }
}

static void wrap(double min, double max, double &coordinate)
{
    coordinate = FWMath::modulo(coordinate - min, max - min) + min;
}

void MYMobilityBase::wrapIfOutside(Coord& targetPosition)
{
    if (lastPosition.x < constraintAreaMin.x || constraintAreaMax.x < lastPosition.x) {
        wrap(constraintAreaMin.x, constraintAreaMax.x, lastPosition.x);
        wrap(constraintAreaMin.x, constraintAreaMax.x, targetPosition.x);
    }
    if (lastPosition.y < constraintAreaMin.y || constraintAreaMax.y < lastPosition.y) {
        wrap(constraintAreaMin.y, constraintAreaMax.y, lastPosition.y);
        wrap(constraintAreaMin.y, constraintAreaMax.y, targetPosition.y);
    }
    if (lastPosition.z < constraintAreaMin.z || constraintAreaMax.z < lastPosition.z) {
        wrap(constraintAreaMin.z, constraintAreaMax.z, lastPosition.z);
        wrap(constraintAreaMin.z, constraintAreaMax.z, targetPosition.z);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
void MYMobilityBase::placeRandomlyIfOutside(Coord& targetPosition)
{
    if (isOutside())
    {
        //Coord newPosition = getMYDronePosition();
        Coord newPosition = getRandomPosition();
        targetPosition += newPosition - lastPosition;
        lastPosition = newPosition;
    }
}

void MYMobilityBase::raiseErrorIfOutside()
{
    if (isOutside())//was just if
    {
        throw cRuntimeError("Mobility moved outside the area %g,%g,%g - %g,%g,%g (x=%g,y=%g,z=%g)",
                constraintAreaMin.x, constraintAreaMin.y, constraintAreaMin.z,
                constraintAreaMax.x, constraintAreaMax.y, constraintAreaMax.z,
                lastPosition.x, lastPosition.y, lastPosition.z);
    }
}

void MYMobilityBase::handleIfOutside(BorderPolicy policy, Coord& targetPosition, Coord& speed, double& angle)
{
    //    policy=REFLECT;
    switch (policy)
    {
    case REFLECT:       reflectIfOutside(targetPosition, speed, angle); break;
    case WRAP:          wrapIfOutside(targetPosition); break;
    case PLACERANDOMLY: {placeRandomlyIfOutside(targetPosition);} break;
    case RAISEERROR:    raiseErrorIfOutside(); break;
    default:            throw cRuntimeError("Invalid outside policy=%d in module", policy, getFullPath().c_str());
    }
}
