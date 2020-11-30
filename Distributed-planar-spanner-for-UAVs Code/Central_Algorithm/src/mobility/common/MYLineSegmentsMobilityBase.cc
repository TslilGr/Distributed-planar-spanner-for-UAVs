//
// Copyright (C) 2005 Andras Varga
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include "MYLineSegmentsMobilityBase.h"
#include "FWMath.h"


MYLineSegmentsMobilityBase::MYLineSegmentsMobilityBase()
{
    targetPosition = Coord::ZERO;
}

void MYLineSegmentsMobilityBase::initializePosition()
{
    MYMobilityBase::initializePosition();
    if (!stationary) {
        setTargetPosition();
    //    EV_INFO << "current target position = " << targetPosition << ", next change = " << nextChange << endl;
        lastSpeed = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
    }
    lastUpdate = simTime();
    scheduleUpdate();
}

//MOSHE: moving the object to its place
void MYLineSegmentsMobilityBase::move()
{
    //   Adi & Tamir
   // EV_INFO <<"me: "<<getFullName()<< endl << "my parent: " << getParentModule()->getFullName()<<"my parent type: " << getParentModule()<< endl;
    //this module is mobility  -> parent is drone
    //if this is a drone in case of priority movement
    if(strcmp(getParentModule()->getName(),"host") && getParentModule()->par("priority_case")){
        //   lastPosition=getParentModule()->par("position");
    //    EV_INFO << "Priority case!!!!!!!!! " << endl;
        lastPosition.x=getParentModule()->par("xPosition");
        lastPosition.y=getParentModule()->par("yPosition");
        // Coord f;
        return;
    }
    simtime_t now = simTime();
    if (now == nextChange) {
        lastPosition = targetPosition;
    //    EV_INFO << "reached current target position = " << lastPosition << endl;
        setTargetPosition();
    //    EV_INFO << "new target position = " << targetPosition << ", next change = " << nextChange << endl;
        lastSpeed = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
    }
    else if (now > lastUpdate) {
        ASSERT(nextChange == -1 || now < nextChange);

        Coord possiblePosition = lastPosition + lastSpeed * (now - lastUpdate).dbl();
        int NFZNum = isAtNFZ(possiblePosition.x, possiblePosition.y);
        if (strcmp(getParentModule()->getName(),"host") && (NFZNum != -1)){
            lastPosition = calculate_TP_Around_NFZ(NFZNum, possiblePosition);
            lastSpeed = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
        }
        else{
            lastPosition = possiblePosition;

        }
    }
    //this module is mobility and its parent is host
   // EV_INFO << "123 me: "<< getFullName() << endl << "321 my parent: "<< getParentModule()->getFullName()<< endl;
    //lastPosition.x
    //update NED's drone parameters
    if (strcmp(getParentModule()->getName(),"host")){       // if this is a drone
        getParentModule()->par("xPosition").setDoubleValue(lastPosition.x);
        getParentModule()->par("yPosition").setDoubleValue(lastPosition.y);
    }


}

//ADDED BY TAMIR & ADI
// if next target position is set to be in a NFZ, set a new valid one
// the coordinate will be at the intersection between the NFZ boundary and the
// vector created by the nextTragetPosition and the NFZ center
Coord MYLineSegmentsMobilityBase::calculate_TP_Around_NFZ(int numOfNFZ, Coord badPosition){
    Coord nextTargetPos;
    nextTargetPos.z=0;
    int NFZx = getParentModule()->getParentModule()->getSubmodule("NFZ",numOfNFZ)->par("x");
    int NFZy = getParentModule()->getParentModule()->getSubmodule("NFZ",numOfNFZ)->par("y");
    int NFZradius = getParentModule()->getParentModule()->getSubmodule("NFZ",numOfNFZ)->par("radius");

    double Ax = badPosition.x, Ay = badPosition.y, Bx = NFZx, By = NFZy, R = NFZradius;
    double Cx = Bx, Cy = By;

    double LAB = sqrt( pow(Bx-Ax,2)+pow(By-Ay,2) );

    // compute the direction vector D from A to B
    double Dx = (Bx-Ax)/LAB;
    double Dy = (By-Ay)/LAB;

    // Now the line equation is x = Dx*t + Ax, y = Dy*t + Ay with 0 <= t <= 1.

    // compute the value t of the closest point to the circle center (Cx, Cy)
    double t = Dx*(Cx-Ax) + Dy*(Cy-Ay);

    // This is the projection of C on the line from A to B.

    // compute the coordinates of the point E on line and closest to C
    double Ex = t*Dx+Ax;
    double Ey = t*Dy+Ay;

    // compute the euclidean distance from E to C
    double LEC = sqrt( pow(Ex-Cx,2)+pow(Ey-Cy,2) );

    // compute distance from t to circle intersection point
    double dt = sqrt( R*R - LEC*LEC);

    // compute first intersection point
    double Fx = (t-dt)*Dx + Ax;
    double Fy = (t-dt)*Dy + Ay;

    // compute second intersection point
    double Gx = (t+dt)*Dx + Ax;
    double Gy = (t+dt)*Dy + Ay;

    if (euclidean_distance(Fx,Fy,badPosition.x,badPosition.y) < euclidean_distance(Gx,Gy,badPosition.x,badPosition.y)){
        nextTargetPos.x=Fx;
        nextTargetPos.y=Fy;
    }
    else{
        nextTargetPos.x=Gx;
        nextTargetPos.y=Gy;
    }


    return nextTargetPos;
}
