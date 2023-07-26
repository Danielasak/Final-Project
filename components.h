//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __FINALPROJECTYD_TCX_H
#define __FINALPROJECTYD_TCX_H


#include <omnetpp.h>
#include <iostream>
#include <string.h>
using namespace std;

using namespace omnetpp;

namespace finalprojectyd {

/**
 * Implements the Txc simple module. See the NED file for more information.
 */
class target : public cSimpleModule
{
  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
};

class droneEtend : public cSimpleModule
{
  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    virtual void detemineCenter();
    virtual void detemineCenterAfter();
    virtual double getMaxDistance(string new_pos);
    virtual int isBaseStationCover(string new_target);
    virtual void chcekCenter(string new_target,double *pos);
};

class baseEtend : public cSimpleModule
{
  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
};

class csEtend : public cSimpleModule
{
  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    virtual int isbsCover(string new_target);
};

class  testudpApp:public cSimpleModule
{

  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
};






}; // namespace

#endif
