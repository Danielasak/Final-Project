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

package finalprojectyd;

import inet.node.inet.WirelessHost;
import inet.applications.contract.IApp;
import inet.applications.udpapp.UdpBasicApp;
//
// Immediately sends out any message it receives. It can optionally generate
// a message at the beginning of the simulation, to bootstrap the process.
//
simple target
{

}

simple csEtend
{
    
 
}


simple droneEtend
{
    
}
simple baseEtend
{
    
}
module drone extends WirelessHost
{
    parameters:
        int drone_ID=0; //this drones ID
        int related_BS_ID=-1; //each drone is related to one base station
        string related_targets; //each drone servers several targets
        int num_targets=0; //each drone servers several targets
        double x_cor1=0;
        double y_cor1=0;
        double x_cor2=0;
        double y_cor2=0;
        int isfree=0;
        
        submodules:
          extd:droneEtend
          {
          }
    //containes the wirelessHost mod
    //submodules:
     //   WH:WirelessHost

}

module bs extends WirelessHost
{
    parameters:
        int BS_ID; //this BS ID
		string related_drones; //each BS servers several drones      
          submodules:
          extd:baseEtend
          {
          }
    //containes the wirelessHost mod

}


module cs extends WirelessHost
{
        

        
    parameters:
		string related_drones=""; //each BS servers several drones      
        //string related_bs;       
 
      submodules:
          ext:csEtend
          {
              
          }

          
      
    //containes the wirelessHost mod

}

