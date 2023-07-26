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

#include "components.h"
#include <iostream>
#include <string.h>
#include <algorithm>
#include <cmath>
using namespace std;
namespace finalprojectyd {

Define_Module(target);
Define_Module(droneEtend);
Define_Module(baseEtend);
Define_Module(csEtend);
Define_Module(testudpApp);

 static double distance(double x1,double x2,double y1 ,double y2)
{
    return sqrt(pow(x1-x2,2)+pow(y1- y2,2));
}

void target::initialize()
{
    cDisplayString& cd= getDisplayString();
    string str(cd.str());
    size_t pos=str.find("p=");

    string str2=str.substr(pos+2);
    str2=str2+";";
    str2=str2+getParentModule()->par("targetPosition").stdstringValue();
    getParentModule()->par("targetPosition").setStringValue(str2);
    string temp= getParentModule()->par("targetPosition").stdstringValue();
    cMessage *msg = new cMessage("SelfMessage");
    //cMessage *msg = new cMessage("SelfMessage");
    double time = uniform(10, 30);
    scheduleAt(simTime() + time, msg);

}

void target::handleMessage(cMessage *msg)
{
    //string todelete="target"+to_string(getParentModule()->par("targetToDelet").intValue());
    string todelete="target"+to_string(getParentModule()->par("targetToDelet").intValue());
    if(todelete.compare(this->getName())==0)
    {
        getParentModule()->par("counter").setIntValue(getParentModule()->par("counter").intValue()-1);
        this->deleteModule();

    }
    else
    {
      double time = uniform(10, 30);
      scheduleAt(simTime() + time, msg);
    }

}
///////////////////////////////////////////////////////////////////////

int droneEtend::isBaseStationCover(string new_target)
{
    int numBase=getParentModule()->getParentModule()->par("baseNumber").intValue();
    double base_r=getParentModule()->getParentModule()->par("base_r").doubleValue();
    double pos_base_x;
    double pos_base_y;
    string cordinets;
    size_t pos_base_new;
    size_t pos_base_old;
    double drone_center[2]={0};
    double dis;
    double r_drone= getParentModule()->getSubmodule("mobility")->par("r").doubleValue();
    string pos_all_base=getParentModule()->getParentModule()->par("basePosition");
    pos_base_old=pos_all_base.find(";");
    cordinets=pos_all_base.substr(0, pos_base_old);
    size_t psik_pos=cordinets.find(",");
    string cord_x=cordinets.substr(0, psik_pos);
    string cord_y=cordinets.substr(psik_pos+1);
    pos_base_x=stod(cord_x);
    pos_base_y=stod(cord_y);
    chcekCenter(new_target,drone_center);
    dis=distance(pos_base_x,drone_center[0],pos_base_y,drone_center[1]);
    dis=dis+r_drone;
    if(dis<base_r)
        return 0;
    for(int i=0;i<numBase-1;i++)
    {
        pos_base_new=pos_all_base.find(";",pos_base_old+1);
        cordinets=pos_all_base.substr(pos_base_old+1, pos_base_new);
        psik_pos=cordinets.find(",");
        pos_base_x=stod(cordinets.substr(0, psik_pos));
        pos_base_y=stod(cordinets.substr(psik_pos+1));
        pos_base_old=pos_base_new;

        dis=distance(pos_base_x,drone_center[0],pos_base_y,drone_center[1]);
        dis=dis+r_drone;
        if(dis<base_r)
            return i;
    }
    return -1;

}


double droneEtend::getMaxDistance(string new_target)
{
    if(new_target=="222.811060,346.850907")
        int n=4;
    size_t new_pos;
    double r=getParentModule()->getSubmodule("mobility")->par("r").doubleValue();
    int num_targets=getParentModule()->par("num_targets").intValue();
    num_targets=num_targets+1;
    double arry_pos [num_targets][2];
    string cordinets;
    string posOfALLTargets=getParentModule()->par("related_targets").stdstringValue();
    posOfALLTargets=posOfALLTargets+new_target+";";
//Find first cord:
    size_t old_pos=posOfALLTargets.find(";");
    cordinets=posOfALLTargets.substr(0, old_pos);
    size_t psik_pos=cordinets.find(",");
    string cord_x=cordinets.substr(0, psik_pos);
    string cord_y=cordinets.substr(psik_pos+1);
    arry_pos[0][0]=stod(cord_x);
    arry_pos[0][1]=stod(cord_y);

////////find all cord
    for(int i=1;i<num_targets;i++)
    {
      new_pos=posOfALLTargets.find(";",old_pos+1);
      cordinets=posOfALLTargets.substr(old_pos+1, new_pos);
      psik_pos=cordinets.find(",");
      cord_x=cordinets.substr(0, psik_pos);
      cord_y=cordinets.substr(psik_pos+1);
      arry_pos[i][0]=stod(cord_x);
      arry_pos[i][1]=stod(cord_y);
      old_pos=new_pos;
    }

//////compute distance
    double max_distance=0;
    double temp=0;
    int maxi;
    int maxj;
    for(int i=0;i<num_targets;i++)
    {
        for(int j=i+1;j<num_targets;j++)
        {
            temp=sqrt(pow(arry_pos[i][0]- arry_pos[j][0],2)+pow(arry_pos[i][1]- arry_pos[j][1],2));
            if(temp>max_distance){
                max_distance=temp;
                maxi=i;
                maxj=j;
                if(max_distance<=r*2)
                  {
                    getParentModule()->par("x_cor1").setDoubleValue(arry_pos[maxi][0]);
                    getParentModule()->par("y_cor1").setDoubleValue(arry_pos[maxi][1]);
                    getParentModule()->par("x_cor2").setDoubleValue(arry_pos[maxj][0]);
                    getParentModule()->par("y_cor2").setDoubleValue(arry_pos[maxj][1]);
                   }


            }
        }
    }



    return max_distance;

}
void droneEtend::chcekCenter(string new_target,double *pos)
{
    if(getParentModule()->par("num_targets").intValue()+1==1)
     {
         size_t psik_pos=new_target.find(",");
         string cord_x=new_target.substr(0, psik_pos);
         string cord_y=new_target.substr(psik_pos+1,new_target.length()-psik_pos-1);
         pos[0]=stod(cord_x);
         pos[1]=stod(cord_y);
         return;
     }

    getMaxDistance(new_target);
    double x_cor1=getParentModule()->par("x_cor1").doubleValue();
    double y_cor1=getParentModule()->par("y_cor1").doubleValue();
    double x_cor2=getParentModule()->par("x_cor2").doubleValue();
    double y_cor2=getParentModule()->par("y_cor2").doubleValue();
    double center_x = (x_cor1+x_cor2)/2;
    double center_y = (y_cor1+y_cor2)/2;
    pos[0]=center_x;
    pos[1]=center_y;

}

void droneEtend::detemineCenter()
{
    if(getParentModule()->par("num_targets").intValue()==1)
    {
        string posOfALLTargets=getParentModule()->par("related_targets").stdstringValue();
        size_t psik_pos=posOfALLTargets.find(",");
        string cord_x=posOfALLTargets.substr(0, psik_pos);
        string cord_y=posOfALLTargets.substr(psik_pos+1,posOfALLTargets.length()-psik_pos-1);
        getParentModule()->getSubmodule("mobility")->par("cx").setDoubleValue(stod(cord_x));
        getParentModule()->getSubmodule("mobility")->par("cy").setDoubleValue(stod(cord_y));
        getParentModule()->getSubmodule("mobility")->callInitialize();
        return;
    }

    double x_cor1=getParentModule()->par("x_cor1").doubleValue();
    double y_cor1=getParentModule()->par("y_cor1").doubleValue();
    double x_cor2=getParentModule()->par("x_cor2").doubleValue();
    double y_cor2=getParentModule()->par("y_cor2").doubleValue();
    double center_x = (x_cor1+x_cor2)/2;
    double center_y = (y_cor1+y_cor2)/2;
   getParentModule()->getSubmodule("mobility")->par("cx").setDoubleValue(center_x);
   getParentModule()->getSubmodule("mobility")->par("cy").setDoubleValue(center_y);
    getParentModule()->getSubmodule("mobility")->callInitialize();

}



void droneEtend::detemineCenterAfter()
{
    if(getParentModule()->par("num_targets").intValue()==1)
    {
        string posOfALLTargets=getParentModule()->par("related_targets").stdstringValue();
        size_t psik_pos=posOfALLTargets.find(",");
        string cord_x=posOfALLTargets.substr(0, psik_pos);
        string cord_y=posOfALLTargets.substr(psik_pos+1,posOfALLTargets.length()-psik_pos-1);

        getParentModule()->getSubmodule("mobility")->par("cx").setDoubleValue(stod(cord_x));
        getParentModule()->getSubmodule("mobility")->par("cy").setDoubleValue(stod(cord_y));
        return;
    }

    double x_cor1=getParentModule()->par("x_cor1").doubleValue();
    double y_cor1=getParentModule()->par("y_cor1").doubleValue();
    double x_cor2=getParentModule()->par("x_cor2").doubleValue();
    double y_cor2=getParentModule()->par("y_cor2").doubleValue();
    double center_x = (x_cor1+x_cor2)/2;
    double center_y = (y_cor1+y_cor2)/2;
    getParentModule()->getSubmodule("mobility")->par("cx").setDoubleValue(center_x);
    getParentModule()->getSubmodule("mobility")->par("cy").setDoubleValue(center_y);

}

void droneEtend::initialize()
{

    string cordinets;
    string posOfALLTargets=getParentModule()->getParentModule()->par("targetPosition").stdstringValue();
    size_t new_pos=posOfALLTargets.find(";");
    int num_targets=getParentModule()->getParentModule()->par("counter").intValue();
    double r=getParentModule()->getSubmodule("mobility")->par("r").doubleValue();
    //size_t new_pos=posOfALLTargets.find(";",old_pos+1);
    size_t old_pos=0;
    cMessage *msg = new cMessage("SelfMessage");
    if(num_targets==0)
    {
        detemineCenter();
        scheduleAt(simTime() + 1, msg);
        return;
    }
    cordinets=posOfALLTargets.substr(old_pos,new_pos);
    int iscover=isBaseStationCover(cordinets);
    if(getMaxDistance(cordinets)<(r*2)&&iscover!=-1)
    {
        getParentModule()->par("related_targets").setStringValue( getParentModule()->par("related_targets").stringValue()+cordinets+";");
        getParentModule()->par("num_targets").setIntValue( getParentModule()->par("num_targets").intValue()+1);
        getParentModule()->par("related_BS_ID").setIntValue(iscover);
        string newtargetPosition=getParentModule()->getParentModule()->par("targetPosition").stdstringValue();
        size_t tempos=newtargetPosition.find(cordinets);
        newtargetPosition.replace(tempos,cordinets.length()+1,"");
        getParentModule()->getParentModule()->par("targetPosition").setStringValue(newtargetPosition);
        getParentModule()->getParentModule()->par("counter").setIntValue(getParentModule()->getParentModule()->par("counter").intValue()-1);
    }
    old_pos=new_pos+1;
    new_pos=posOfALLTargets.find(";",old_pos);

    for(int i=1;i<num_targets;i++)
    {

        cordinets=posOfALLTargets.substr(old_pos ,new_pos-old_pos);
        int iscover=isBaseStationCover(cordinets);
        if(getMaxDistance(cordinets)<(r*2)&&iscover!=-1)
        {

        getParentModule()->par("related_targets").setStringValue( getParentModule()->par("related_targets").stringValue()+cordinets+";");
        getParentModule()->par("num_targets").setIntValue( getParentModule()->par("num_targets").intValue()+1);
        getParentModule()->par("related_BS_ID").setIntValue(iscover);
        string newtargetPosition=getParentModule()->getParentModule()->par("targetPosition").stdstringValue();
        size_t tempos=newtargetPosition.find(cordinets);
        newtargetPosition.replace(tempos,cordinets.length()+1,"");
        getParentModule()->getParentModule()->par("targetPosition").setStringValue(newtargetPosition);
        getParentModule()->getParentModule()->par("counter").setIntValue(getParentModule()->getParentModule()->par("counter").intValue()-1);
        }
        old_pos=new_pos+1;
        new_pos=posOfALLTargets.find(";",old_pos);
    }

    detemineCenter();

    double time = uniform(10, 30);

    scheduleAt(simTime() + time, msg);

}

void droneEtend::handleMessage(cMessage *msg)
{

    string cordinets;
    int flag=0;
    string posOfALLTargets=getParentModule()->getParentModule()->par("targetPosition").stdstringValue();
    size_t new_pos=posOfALLTargets.find(";");
    int num_targets=getParentModule()->getParentModule()->par("counter").intValue();
    double r=getParentModule()->getSubmodule("mobility")->par("r").doubleValue();
    size_t old_pos=0;
        if(num_targets==0)
        {
            //detemineCenterAfter();
            scheduleAt(simTime() + 1, msg);
            return;
        }
        cordinets=posOfALLTargets.substr(old_pos,new_pos);
           int iscover=isBaseStationCover(cordinets);
           if(getMaxDistance(cordinets)<(r*2)&&iscover!=-1)
           {
               flag=1;
               string rt= getParentModule()->par("related_targets").stringValue();
               getParentModule()->par("related_targets").setStringValue( getParentModule()->par("related_targets").stringValue()+cordinets+";");
               getParentModule()->par("num_targets").setIntValue( getParentModule()->par("num_targets").intValue()+1);
               getParentModule()->par("related_BS_ID").setIntValue(iscover);
               string newtargetPosition=getParentModule()->getParentModule()->par("targetPosition").stdstringValue();
               size_t tempos=newtargetPosition.find(cordinets);
               newtargetPosition.replace(tempos,cordinets.length()+1,"");
               getParentModule()->getParentModule()->par("targetPosition").setStringValue(newtargetPosition);
               getParentModule()->getParentModule()->par("counter").setIntValue(getParentModule()->getParentModule()->par("counter").intValue()-1);
           }
           old_pos=new_pos+1;
           new_pos=posOfALLTargets.find(";",old_pos);

           for(int i=1;i<num_targets;i++)
           {

               cordinets=posOfALLTargets.substr(old_pos ,new_pos-old_pos);
               int iscover=isBaseStationCover(cordinets);
               if(getMaxDistance(cordinets)<(r*2)&&iscover!=-1)
               {
               flag=1;
               getParentModule()->par("related_targets").setStringValue( getParentModule()->par("related_targets").stringValue()+cordinets+";");
               getParentModule()->par("num_targets").setIntValue( getParentModule()->par("num_targets").intValue()+1);
               getParentModule()->par("related_BS_ID").setIntValue(iscover);
               string newtargetPosition=getParentModule()->getParentModule()->par("targetPosition").stdstringValue();
               size_t tempos=newtargetPosition.find(cordinets);
               newtargetPosition.replace(tempos,cordinets.length()+1,"");
               getParentModule()->getParentModule()->par("targetPosition").setStringValue(newtargetPosition);
               getParentModule()->getParentModule()->par("counter").setIntValue(getParentModule()->getParentModule()->par("counter").intValue()-1);
               }
               old_pos=new_pos+1;
               new_pos=posOfALLTargets.find(";",old_pos);
           }
           if(flag==1)
           {

           detemineCenterAfter();
           flag=0;
           }

           double time = uniform(10, 30);
           scheduleAt(simTime() + 1, msg);
           string test=getParentModule()->getFullName();
           char number=test.at(4);
           string parname="Host";
           parname=parname+number;
           if(getParentModule()->par("related_targets").stdstringValue()=="")
               getParentModule()->getParentModule()->par(parname.c_str()).setIntValue(1);
           else
           {
               getParentModule()->getParentModule()->par(parname.c_str()).setIntValue(0);
           }


}


void baseEtend::initialize()
{

     cDisplayString& cd= getDisplayString();
     string str(cd.str());
     size_t pos=str.find("p=");
     size_t pos2=str.find(";");
     string str2=str.substr(pos+2, pos2-(pos+2));
     str2=str2+";";
     str2=str2+ getParentModule()->par("basePosition").stdstringValue();
     getParentModule()->par("basePosition").setStringValue(str2);
     getParentModule()->par("baseNumber").setIntValue(getParentModule()->par("baseNumber").intValue()+1);
}

void baseEtend::handleMessage(cMessage *msg)
{
    // just send back the message we received
}

void csEtend::initialize()
{
    cMessage *msg = new cMessage("SelfMessage");
    cMessage *msg2 = new cMessage("SelfMessage");
    double time = uniform(10, 30);
    msg->setKind(1);
    msg2->setKind(10);
    scheduleAt(simTime() + time, msg);
    scheduleAt(simTime() + 1, msg2);

}

int csEtend::isbsCover(string new_target)
{
    if(simTime()==23)
    {
        int zibi=100;
    }
    ///////////define variables
    double pos_base_x;
    double pos_base_y;
    double new_target_x;
    double new_target_y;
    double dis;
    int numBase=getParentModule()->getParentModule()->par("baseNumber").intValue();
    double base_r=getParentModule()->getParentModule()->par("base_r").doubleValue();
    string pos_all_base=getParentModule()->getParentModule()->par("basePosition");
    string cordinets;
    //////////////////
    size_t pos_base_new;
    size_t pos_base_old;

    //////get bs cord
    pos_base_old=pos_all_base.find(";");
    cordinets=pos_all_base.substr(0, pos_base_old);
    size_t psik_pos=cordinets.find(",");
    string cord_x=cordinets.substr(0, psik_pos);
    string cord_y=cordinets.substr(psik_pos+1);
    if(cord_y.find(";")!=string::npos)
    {
        int num=4;
        num=num+4;
    }
    if(cord_x.find(";")!=string::npos)
      {
          int num2=4;
          num2=num2+8;
      }

    if(cord_x.find(",")!=string::npos)
        {
            size_t tempos2=cord_x.find(",");
            cord_x.replace(tempos2,tempos2+1,"");
        }
    if(cord_y.find(",")!=string::npos)
     {
        size_t tempos2=cord_y.find(",");
        cord_y.replace(tempos2,tempos2+1,"");
     }


    cout<<"\ntest10\n";
    cout<<cord_x;
    cout<<"\n";
    cout<<cord_y;
    pos_base_x=stod(cord_x);
    pos_base_y=stod(cord_y);
    //////

    //get target cord////
    size_t psik_pos_target=new_target.find(",");
    string cord_x_target=new_target.substr(0, psik_pos_target);
    string cord_y_target=new_target.substr(psik_pos_target+1);

    if(cord_x_target.find(",")!=string::npos)
    {
        size_t tempos=cord_x_target.find(",");
        cord_x_target.replace(tempos,tempos+1,"");
    }
    if(cord_y_target.find(",")!=string::npos)
      {
        size_t tempos=cord_y_target.find(",");
        cord_y_target.replace(tempos,tempos+1,"");
     }


    if(cord_x_target.find(";")!=string::npos)
       {
           size_t tempos=cord_x_target.find(";");
           cord_x_target.replace(tempos,tempos+1,"");
       }
       if(cord_y_target.find(";")!=string::npos)
         {
           size_t tempos=cord_y_target.find(";");
           cord_y_target.replace(tempos,tempos+1,"");
        }
    new_target_x=stod(cord_x_target);
    new_target_y=stod(cord_y_target);
    //////////////


    dis=distance(pos_base_x,new_target_x,pos_base_y,new_target_y);
    if(dis<base_r)
        return 1;
    for(int i=0;i<numBase-1;i++)
        {
            pos_base_new=pos_all_base.find(";",pos_base_old+1);
            cordinets=pos_all_base.substr(pos_base_old+1, pos_base_new-pos_base_old-1);
            psik_pos=cordinets.find(",");

            //cout<<"\ntest20\n";
            //cout<<cordinets.substr(0, psik_pos);
           // cout<<"\n";
           // cout<<cordinets.substr(psik_pos+1);
           // cout<<"\n";

            string ps_bs_x=cordinets.substr(0, psik_pos);
            string ps_bs_y=cordinets.substr(psik_pos+1);
            if(ps_bs_x.find(",")!=string::npos)
              {
                size_t tempos=ps_bs_x.find(",");
                ps_bs_x.replace(tempos,tempos+1,"");
              }
           if(ps_bs_y.find(",")!=string::npos)
            {
                size_t tempos=ps_bs_y.find(",");
                ps_bs_y.replace(tempos,tempos+1,"");
             }
           if(ps_bs_x.find(";")!=string::npos)
            {
             size_t tempos=ps_bs_x.find(";");
             ps_bs_x.replace(tempos,tempos+1,"");
             }
           if(ps_bs_y.find(";")!=string::npos)
            {
             size_t tempos=ps_bs_y.find(";");
              ps_bs_y.replace(tempos,tempos+1,"");
              }


            pos_base_x=stod(cordinets.substr(0, psik_pos));
            pos_base_y=stod(cordinets.substr(psik_pos+1));
            pos_base_old=pos_base_new;

            dis=distance(pos_base_x,new_target_x,pos_base_y,new_target_y);
            if(dis<base_r)
                return 1;
        }
        return -1;


}

void csEtend::handleMessage(cMessage *msg)
{
    cMessage *msg2 = new cMessage("SelfMessage");
    msg2->setKind(10);
    if(msg->getKind()==1)//1= create a target
    {
        // just send back the message we received
        if(getParentModule()->getParentModule()->par("counter_all_drones").intValue()>=getParentModule()->getParentModule()->par("MaxTarget").intValue())
        {
            double time = uniform(10, 30);
            msg->setKind(1);
            scheduleAt(simTime() + time, msg);
            return;
        }
        // just send back the message we received
        // Create an instance of an existing module
        double Min, Max; // define the range of x positions
        Min = 15.0;
        Max = 500.0;
        double xPosition = uniform(Min, Max);
        double yPosition = uniform(Min, Max);
        string xP = to_string(xPosition);
        string yP = to_string(yPosition);
        string display="i=status/stop;p=";
        string psik=",";
        display=display +xP+psik+yP;

        getParentModule()->getParentModule()->par("counter").setIntValue(getParentModule()->getParentModule()->par("counter").intValue()+1);
        getParentModule()->getParentModule()->par("counter_all_drones").setIntValue(getParentModule()->getParentModule()->par("counter_all_drones").intValue()+1);
        string name="target"+to_string(getParentModule()->getParentModule()->par("counter_all_drones").intValue());

        cModuleType *moduleType = cModuleType::get("finalprojectyd.target");
        cModule *newModule = moduleType->create(name.c_str(),getParentModule()->getParentModule());

        // Set the parent module of the newly created module
        newModule->setDisplayString(display.c_str());
        // Build the new module and initialize it
        newModule->finalizeParameters();
        newModule->buildInside();
        newModule->callInitialize();


        //sendMsgFromParentModule();
        double flag=uniform(0, 10);
        if(flag>=2)
        {
            msg->setKind(1);
            double time = uniform(10, 30);
            scheduleAt(simTime() + time, msg);
        }
        else
        {
            msg->setKind(2);
            double time = uniform(10, 30);
            scheduleAt(simTime() + time, msg);
        }
    }
    else if(msg->getKind()==2)
    {
       // cModuleType *moduleType = cModuleType::get("finalprojectyd.target");
       // getParentModule()->getParentModule()->par("targetToDelet").setIntValue(getParentModule()->getParentModule()->par("counter").intValue());
        msg->setKind(1);
        double time = uniform(10, 30);
        scheduleAt(simTime() + time, msg);
    }

    else if(msg->getKind()==10)
    {
        //////Check if there is a need to open a new bs
        scheduleAt(simTime() + 1, msg2);
        size_t pos_first_target;
        string target;
        string Targets_not_occupied=getParentModule()->getParentModule()->par("targetPosition").stdstringValue();
        pos_first_target=Targets_not_occupied.find(";");
        target=Targets_not_occupied.substr(0, pos_first_target);
        if(target=="")
        {
            return;
        }
        int test=isbsCover(target);
        int numberOfDrones=getParentModule()->getParentModule()->par("drones").intValue();
        string parname="Host";
        int flag;
        double posx;
        double posy;
        if(test==-1)
        {
            for(int i=0;i<numberOfDrones;i++)
            {
                string temp=parname+to_string(i+1);
                flag=getParentModule()->getParentModule()->par(temp.c_str()).intValue();
                if(flag==1)//open a base staiton
                {

                    string displaybs="i=device/antennatower;p=";
                    size_t psik_pos=target.find(",");
                    string cord_x=target.substr(0, psik_pos);
                    string cord_y=target.substr(psik_pos+1,target.length()-psik_pos-1);
                    cout<<"\ntest1\n";
                    cout<<cord_x;
                    cout<<"\n";
                    cout<<cord_y;






                    if(cord_x.find(",")!=string::npos)
                       {
                       size_t tempos=cord_x.find(",");
                       cord_x.replace(tempos,tempos+1,"");
                        }
                    if(cord_y.find(",")!=string::npos)
                      {
                       size_t tempos=cord_y.find(",");
                       cord_y.replace(tempos,tempos+1,"");
                         }
                      if(cord_x.find(";")!=string::npos)
                        {
                       size_t tempos=cord_x.find(";");
                       cord_x.replace(tempos,tempos+1,"");
                        }
                     if(cord_y.find(";")!=string::npos)
                        {
                        size_t tempos=cord_y.find(";");
                        cord_y.replace(tempos,tempos+1,"");
                       }

                    posx=stod(cord_x);
                    posy=stod(cord_y);
                    posx=posx+15;
                    posy=posy+15;
                    displaybs+=to_string(posx);
                    displaybs+=',';
                    displaybs+=to_string(posy);
                    cModuleType *moduleType_bs = cModuleType::get("finalprojectyd.baseEtend");
                    cModule *newModule_bs = moduleType_bs->create("bs",getParentModule()->getParentModule());
                    newModule_bs->setDisplayString(displaybs.c_str());
                    // Build the new module and initialize it
                    newModule_bs->finalizeParameters();
                    newModule_bs->buildInside();
                    newModule_bs->callInitialize();
                    return;
                }

            }
        }
    }



}
///////////////////////////////////////////////


void testudpApp::initialize()
{

cMessage *msg=new  cMessage;

send(msg,"socketOut");
}

void testudpApp::handleMessage(cMessage *msg)
{

}




}; // namespace
