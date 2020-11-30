//
// Copyright (C) 2005 Georg Lutz, Institut fuer Telematik, University of Karlsruhe
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


#include "MYRandomWPMobility.h"


Define_Module(MYRandomWPMobility);


MYRandomWPMobility::MYRandomWPMobility()
{
    nextMoveIsWait = false;
}

void MYRandomWPMobility::initialize(int stage)
{
    MYLineSegmentsMobilityBase::initialize(stage);

    if (stage == 0)
    {
        goToSinglePoint = false;
        stationary = (par("speed").getType()=='L' || par("speed").getType()=='D') && (double)par("speed") == 0;
    }
}

void MYRandomWPMobility::setTargetPosition()
{
    double distance;
    if(goToSinglePoint)
    {
        lastSpeed = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();

        if(targetPosition.x == 150 && targetPosition.y == 150)
        {
            targetPosition = {150,150,0};
            distance = 10;
        }
        else
        {
            Coord V,newPosition;
            double V_Magnitude;
            V.x = lastPosition.x - 150;
            V.y = lastPosition.y - 150;
            V_Magnitude = sqrt(V.x*V.x + V.y*V.y);
            newPosition.x = lastPosition.x - (V.x/V_Magnitude)*30;
            newPosition.y = lastPosition.y - (V.y/V_Magnitude)*30;

            lastPosition = newPosition;
            distance = lastPosition.distance(targetPosition);
            targetPosition = lastPosition;
        }

        double speed = par("speed");
        simtime_t travelTime = distance / speed;
        nextChange = simTime() + travelTime;

    }

    else if (nextMoveIsWait)
    {
        simtime_t waitTime = par("waitTime");
        nextChange = simTime() + waitTime;
    }
    else
    {
        targetPosition = getRandomPosition();
        double speed = par("speed");
        double distance = lastPosition.distance(targetPosition);
        simtime_t travelTime = distance / speed;
        nextChange = simTime() + travelTime;
    }
    nextMoveIsWait = !nextMoveIsWait;
}

void MYRandomWPMobility::move()
{
//    double speed = par("speed");          //added by me
//    if(simulation.getSimTime() > 130)
//    {
//        goToSinglePoint = true;
//        setTargetPosition();
//    }
//    else
//    {
        MYLineSegmentsMobilityBase::move();
        raiseErrorIfOutside();              //commented by me
//        if (isAtNFZ(lastPosition.x,lastPosition.y)){
//            throw cRuntimeError("Drone was shut down!!!");
//        }
   //     reflectIfOutside(lastPosition,speed,90);    //added by me
   // }
}
