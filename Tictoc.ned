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

package finalprojectyd.simulations;
package inet.networklayer.configurator.ipv4;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.physicallayer.contract.packetlevel.IRadioMedium;
import inet.node.inet.WirelessHost;
import inet.visualizer.integrated.IntegratedCanvasVisualizer;

import finalprojectyd.target;
import finalprojectyd.drone;
import finalprojectyd.bs;
import finalprojectyd.baseEtend;
import finalprojectyd.cs;
import finalprojectyd.csEtend;


//
// Two instances (tic and toc) of components connected.
//
network Tictoc
{
    parameters:
        @display("bgb=600,600;bgi=background/streetmap,s");
        @figure[title](type=label; pos=0,-1; anchor=sw; color=darkblue);
        string targetPosition="";
        string alltargetPosition="";
        string basePosition="";
        int baseNumber = 0;
        int counter = 2;
        int counter_all_drones = 2;
        int MaxTarget;
        int targetToDelet=0;
        double base_r = 100;
        int drones = 5;
        int Host1 = 0;
        int Host2 = 0;
        int Host3 = 0;
        int Host4 = 0;
        int Host5 = 0;
        int busyDrones=0;

        

    submodules:
        target1: target {
            @display("i=status/stop;p=176.368,410.72");
        }
        target2: target {
            @display("i=status/stop;p=199.31999,380.52");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=399.848,253.68");
        }
        radioMedium: <default("UnitDiskRadioMedium")> like IRadioMedium {
            @display("p=372.064,177.576");
        }
        Bs: baseEtend {
            @display("p=147.37599,380.52;i=device/antennatower");
        }
        host1: drone {
            @display("p=126.84,241.59999;i=misc/drone");
        }
        host2: drone {
            @display("p=228.312,280.256;i=misc/drone");
        }
        host3: drone {
            @display("p=100.312,280.256;i=misc/drone");
        }
        host4: drone {
            @display("p=228.312,240.392;i=misc/drone");
        }
        host5: drone {
            @display("p=228.312,240.392;i=misc/drone");
        }



        ControlStation1: cs {
            @display("p=548.432,204.152;i=misc/desktoppc2");
        }
       // ControlStation2: cs {
       //     @display("p=495.28,277.84;i=device/antennatower");
       // }
}
