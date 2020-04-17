/*
 * This script simulates a simple network in which one end device sends one
 * packet to the gateway.
 */

#include "ns3/csv.h"
#include "ns3/garbage_station.h"
#include "ns3/node_info.h"

#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/simulator.h"
//#include "ns3/log.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/node-container.h"
#include "ns3/position-allocator.h"
#include "ns3/command-line.h"
#include "ns3/names.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/core-module.h"

#include "ns3/lora-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/file-helper.h"
#include "ns3/trace-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/network-server-helper.h"

#include <algorithm>
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>
#include <limits>
#include <stdio.h>

using namespace std;
using namespace ns3;
using namespace tarako;
using namespace lorawan;

// ROW const value
const int ID = 0;
const int LONGITUDE = 1;
const int LATITUDE = 2;
const int BURNABLE = 3;
const int INCOMBUSTIBLE = 4;
const int RESOURCE = 5;

NS_LOG_COMPONENT_DEFINE ("OnlyLoRaWANNetworkModel");

// --- Parse CSV --- //
double convert_string_to_double(std::string value)
{
    return stod(value);
}

bool is_supported_garbage_type(std::string value)
{
    if (value == "○") return true;
    else return false;
}
// --- Trace Callback Fuction --- //
void OnLoRaWANEnergyConsumptionChange (Ptr<OutputStreamWrapper> stream, int node_id, NodeInfo* node, double oldEnergyConsumption, double newEnergyConsumption)
{
  ostringstream oss;
  node->energy_consumption = newEnergyConsumption;
  cout <<std::fixed << "[UPDATE] node id: " << node_id << " | " << "new: " << newEnergyConsumption << "mA" << endl;
}

void OnPacketRecieved (Ptr<const Packet> packet) {
    uint8_t *buffer = new uint8_t[packet->GetSize ()];
    packet->CopyData (buffer, packet->GetSize ());
    string message ((char *)buffer);
    cout << message << endl;
}

int main (int argc, char *argv[])
{
  LogComponentEnable ("OnlyLoRaWANNetworkModel", LOG_LEVEL_ALL);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_NODE);
  LogComponentEnableAll (LOG_PREFIX_TIME);
  
  /************************
  *  Setup Simulation Area (愛知県東浦町) *
  ************************/
  
  // --- (BEGIN) Read Garbage Station Map from CSV --- //
  const string csv_file = "/home/vagrant/ns-3.30/scratch/test_copy.csv";
  vector<vector<string>> data;
  vector<GarbageStation> g_stations;

  try {
        Csv objCsv(csv_file);
        if (!objCsv.getCsv(data)) {
            cout << "[error] can not read file" << endl;
            return 1;
        }
        for (unsigned int row = 1; row < data.size(); row++) {
            vector<string> rec = data[row];
            GarbageStation g_station;

            g_station.id = rec[ID];
            g_station.latitude = stod(rec[LATITUDE]);
            g_station.longitude = stod(rec[LONGITUDE]);
            g_station.burnable = is_supported_garbage_type(rec[BURNABLE]);
            g_station.incombustible = is_supported_garbage_type(rec[INCOMBUSTIBLE]);
            g_station.resource = is_supported_garbage_type(rec[RESOURCE]);
 
            g_stations.push_back(g_station);
        }
    }
    catch (exception& ex) {
        cout << endl;
        cerr << ex.what() << endl;
        return 1;
    }
    // --- (END) Read Garbage Station Map from CSV --- //
    
    NS_LOG_INFO ("--- initialize end devices position ---");
    MobilityHelper mobility;
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    NodeContainer endDevices;
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
    int cnt_node = 0;
    
    for (const auto& g_station: g_stations) {
        if (g_station.burnable) {
            allocator->Add (Vector (g_station.latitude, g_station.longitude,0));
            cnt_node++;
        }
        if (g_station.incombustible) {
            allocator->Add (Vector (g_station.latitude + 0.000001, g_station.longitude,0));
            cnt_node++;
        }
        if (g_station.resource) {
            allocator->Add (Vector (g_station.latitude + 0.000002, g_station.longitude,0));
            cnt_node++;
        }
    }
    NS_LOG_INFO("create " << cnt_node << " end devices");
    endDevices.Create(cnt_node);
    mobility.SetPositionAllocator (allocator);
    mobility.Install (endDevices);

    NS_LOG_INFO ("--- initialize LoRaWAN channel ---");
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
    loss->SetPathLossExponent (3.76);
    loss->SetReference (1, 7.7);
    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
    Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);
    LoraPhyHelper phyHelper = LoraPhyHelper ();
    phyHelper.SetChannel (channel);
    LorawanMacHelper macHelper = LorawanMacHelper ();
    LoraHelper helper = LoraHelper ();
    helper.EnablePacketTracking();
    
    phyHelper.SetDeviceType (LoraPhyHelper::ED);
    macHelper.SetDeviceType (LorawanMacHelper::ED_A);
    NetDeviceContainer endDevicesNetDevices = helper.Install (phyHelper, macHelper, endDevices);
    
    for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
    }

    // iterate our nodes and print their position.
    // for(NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    // {
    //   Ptr<Node> object = *j;
    //   Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
    //   NS_ASSERT (position != 0);
    //   Vector pos = position->GetPosition ();
    //   std::cout << std::fixed << "x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;
    // }
    
    NS_LOG_INFO ("--- initialize LoRaWAN gateway ---");
    
    NodeContainer gateways;
    gateways.Create (3);
    Ptr<ListPositionAllocator> gw_allocator = CreateObject<ListPositionAllocator> ();
    gw_allocator->Add (Vector (34.969392, 136.924615, 15.0));
    gw_allocator->Add (Vector (34.953981, 136.962864, 15.0));
    gw_allocator->Add (Vector (34.973183, 136.967018, 15.0));
    mobility.SetPositionAllocator (gw_allocator);
    mobility.Install (gateways);
    phyHelper.SetDeviceType (LoraPhyHelper::GW);
    macHelper.SetDeviceType (LorawanMacHelper::GW);
    helper.Install (phyHelper, macHelper, gateways);
    macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
    PeriodicSenderHelper periodicSenderHelper;
    periodicSenderHelper.SetPeriod (Seconds (5));
    periodicSenderHelper.Install (endDevices);
    BasicEnergySourceHelper basicSourceHelper;
    LoraRadioEnergyModelHelper radioEnergyHelper;

    NS_LOG_INFO ("--- initialize LoRaWAN Energy Consumption Model ---");
    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (10000)); // Energy in J
    basicSourceHelper.Set ("BasicEnergySupplyVoltageV", DoubleValue (3.3));
    radioEnergyHelper.Set ("StandbyCurrentA", DoubleValue (0.0014));
    radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.028));
    radioEnergyHelper.Set ("SleepCurrentA", DoubleValue (0.0000015));
    radioEnergyHelper.Set ("RxCurrentA", DoubleValue (0.0112));
    radioEnergyHelper.SetTxCurrentModel ("ns3::ConstantLoraTxCurrentModel","TxCurrent", DoubleValue (0.028));
    // install source on EDs' nodes
    EnergySourceContainer sources = basicSourceHelper.Install (endDevices);
    Names::Add ("/Names/EnergySource", sources.Get (0));
    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install(endDevicesNetDevices, sources);

    NS_LOG_INFO ("--- initialize LoRaWAN application scenario ---");
    Time simulationTime = Minutes (11);

    PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
    appHelper.SetPeriod (Seconds(10));
    appHelper.SetPacketSize (23);
    ApplicationContainer appContainer = appHelper.Install (endDevices);
    appContainer.Start (Seconds (0));
    appContainer.Stop (simulationTime);
    /**************
    * Get output *
    **************/
    FileHelper fileHelper;
    fileHelper.ConfigureFile ("battery-level", FileAggregator::SPACE_SEPARATED);
    fileHelper.WriteProbe ("ns3::DoubleProbe", "/Names/EnergySource/RemainingEnergy", "Output");
    
    string energy_efficiency_file = "energy_efficiency.csv";
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream(energy_efficiency_file);

    vector<NodeInfo> node_infos(cnt_node);
    for (int i=0; i < (int)deviceModels.GetN(); i++) {
        node_infos[i].id = std::to_string(i);
        deviceModels.Get(i) -> TraceConnectWithoutContext(
            "TotalEnergyConsumption", 
            MakeBoundCallback(&OnLoRaWANEnergyConsumptionChange, stream, i, &node_infos[i])
        );
    }
    
    // --- setup network server --- ..
    NodeContainer networkServers;
    networkServers.Create (1);
    NetworkServerHelper networkServerHelper;
    networkServerHelper.SetGateways (gateways);
    networkServerHelper.SetEndDevices (endDevices);
    networkServerHelper.EnableAdr (true);
    networkServerHelper.SetAdr ("ns3::AdrComponent");
    ApplicationContainer nsModels = networkServerHelper.Install (networkServers);

    ForwarderHelper forwarderHelper;
    forwarderHelper.Install (gateways);

    Ptr<NetworkServer> ns = nsModels.Get(0)->GetObject<NetworkServer>();
    ns->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(&OnPacketRecieved));
    /****************
    *  Simulation  *
    ****************/
    Simulator::Stop (simulationTime);
    Simulator::Run ();
    Simulator::Destroy ();

    for (const auto& info: node_infos) {
        *stream->GetStream () << info.id << "," << info.energy_consumption << endl;
    }
    return 0;
}
