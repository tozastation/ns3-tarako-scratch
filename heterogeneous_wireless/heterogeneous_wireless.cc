/*
 * This script simulates a simple network in which one end device sends one
 * packet to the gateway.
 */

/* Custom Tarako Module */
#include "ns3/const.h"
#include "ns3/util.h"
#include "ns3/tracer.h"
#include "ns3/node_payload.h"
#include "ns3/logger.h"
#include "ns3/garbage_station.h"
#include "ns3/group_node.h"

#include "ns3/simulator.h"
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
#include "ns3/basic-energy-source-helper.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/file-helper.h"
#include "ns3/trace-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/network-server-helper.h"

#include "ns3/ble-helper.h"

#include <algorithm>
// #include <ctime>
#include <string>
#include <vector>
// #include <exception>
// #include <limits>
// #include <stdio.h>
#include <unordered_map> 
// #include <sstream>

// using namespace std;
using namespace ns3;
using namespace lorawan;

// --- Global Object --- //
tarako::TarakoLogger tarako_logger;
tarako::TarakoUtil tarako_util;
// --- LoRaWAN Gateway, LoRaWAN & BLE End Device --- //
MobilityHelper mobility_gw, mobility_ed;
Ptr<ListPositionAllocator> ed_allocator = CreateObject<ListPositionAllocator> ();
Ptr<ListPositionAllocator> gw_allocator = CreateObject<ListPositionAllocator> ();
NodeContainer end_devices, gateways, network_servers;
ApplicationContainer lora_network_apps;
NetDeviceContainer ble_net_devices;

Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
Ptr<RandomPropagationLossModel> randomLoss = CreateObject<RandomPropagationLossModel> ();
Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

LoraPhyHelper lora_phy_helper = LoraPhyHelper ();
LorawanMacHelper lora_mac_helper = LorawanMacHelper ();
LoraHelper lora_helper = LoraHelper ();
BasicEnergySourceHelper basic_src_helper;
LoraRadioEnergyModelHelper radio_energy_helper;
NetworkServerHelper network_server_helper;
ForwarderHelper forwarderHelper;

BleHelper ble_helper;
// --- Global Const Variable --- //
const std::string GARBAGE_BOX_MAP_FILE = "/home/vagrant/workspace/tozastation/ns-3.30/scratch/test_copy.csv";
const int LORAWAN_GATEWAY_NUM = 3;
const int LORAWAN_NETWORK_SERVER_NUM = 1;
// --- Global Variable --- //
int cnt_node = 0;
std::vector<std::vector<int>> available_connections;
std::vector<tarako::TarakoNodeData> tarako_nodes;
std::unordered_map<int, tarako::TarakoNodeData> trace_node_data_map;
std::unordered_map<int, tarako::GarbageBoxSensor> trace_garbage_box_sensor_map;

NS_LOG_COMPONENT_DEFINE ("HeterogeneousWirelessNetworkModel");

int main (int argc, char *argv[])
{
    // --- Logging --- //
    LogComponentEnable ("HeterogeneousWirelessNetworkModel", LOG_LEVEL_ALL);
    // LogComponentEnable ("AdrComponent", LOG_LEVEL_ALL);
    // LogComponentEnable ("EndDeviceLorawanMac", LOG_LEVEL_ALL);
    // LogComponentEnable ("LoraPacketTracker", LOG_LEVEL_ALL);
    // LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
    // LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
    LogComponentEnableAll (LOG_PREFIX_FUNC);
    LogComponentEnableAll (LOG_PREFIX_NODE);
    LogComponentEnableAll (LOG_PREFIX_TIME);
    // --- Set the EDs to require Data Rate control from the NS --- //
    Config::SetDefault ("ns3::EndDeviceLorawanMac::DRControl", BooleanValue (true));
    PacketMetadata::Enable();
    // [CSV READ] Garbage Box //
    const auto garbage_boxes = tarako::TarakoUtil::GetGarbageBox(GARBAGE_BOX_MAP_FILE);  
    // [ADD] garbage boxes geolocation in allocator 
    
    for (const auto& g_box: garbage_boxes) {
        std::vector<int> available_connection;
        if (g_box.burnable) {
            ns3::Vector3D pos = Vector (g_box.latitude, g_box.longitude,1);
            ed_allocator->Add (pos);
            cnt_node++;
            
            tarako::TarakoNodeData node;
            node.id = cnt_node;
            node.position = pos;
            tarako_nodes.push_back(node);
            available_connection.push_back(cnt_node-1);
        }
        if (g_box.incombustible) {
            ns3::Vector3D pos = Vector (g_box.latitude + 0.000001, g_box.longitude,1);
            ed_allocator->Add(pos);
            cnt_node++;

            tarako::TarakoNodeData node;
            node.id = cnt_node;
            node.position = pos;
            tarako_nodes.push_back(node);
            available_connection.push_back(cnt_node-1);
        }
        if (g_box.resource) {
            ns3::Vector3D pos = Vector (g_box.latitude + 0.000002, g_box.longitude,1);
            ed_allocator->Add(pos);
            cnt_node++;

            tarako::TarakoNodeData node;
            node.id = cnt_node;
            node.position = pos;
            tarako_nodes.push_back(node);
            available_connection.push_back(cnt_node-1);
        }
        available_connections.push_back(available_connection);
    }
    // [INIT] End Devices (LoRaWAN, BLE)
    end_devices.Create(cnt_node);
    mobility_ed.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility_ed.SetPositionAllocator(ed_allocator);
    mobility_ed.Install (end_devices);
    
    // [INIT] LoRaWAN Gateway
    gateways.Create (LORAWAN_GATEWAY_NUM);
    mobility_gw.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    gw_allocator->Add (Vector (34.969392, 136.924615, 15.0));
    gw_allocator->Add (Vector (34.953981, 136.962864, 15.0));
    gw_allocator->Add (Vector (34.973183, 136.967018, 15.0));
    mobility_gw.SetPositionAllocator (gw_allocator);
    mobility_gw.Install (gateways);
    mobility_gw.SetPositionAllocator (gw_allocator);
    mobility_gw.Install (gateways);

    // [INIT] LoRaWAN Setting
    // -- Channel --- //
    loss->SetPathLossExponent (3.76);
    loss->SetReference (1, 7.7);
    x->SetAttribute ("Min", DoubleValue (0.0));
    x->SetAttribute ("Max", DoubleValue (10));
    randomLoss->SetAttribute ("Variable", PointerValue (x));
    loss->SetNext (randomLoss);
    Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);
    // --- Helper --- //
    lora_phy_helper.SetChannel (channel);
    lora_helper.EnablePacketTracking();
    // --- Install Helper to EndDevices ---
    uint8_t nwk_id = 54;
    uint32_t nwk_addr = 1864;
    Ptr<LoraDeviceAddressGenerator> addr_gen = CreateObject<LoraDeviceAddressGenerator> (nwk_id,nwk_addr);
    lora_phy_helper.SetDeviceType (LoraPhyHelper::ED);
    lora_mac_helper.SetDeviceType (LorawanMacHelper::ED_A);
    lora_mac_helper.SetAddressGenerator (addr_gen);
    lora_mac_helper.SetRegion (LorawanMacHelper::EU);
    NetDeviceContainer ed_net_devices = lora_helper.Install (lora_phy_helper, lora_mac_helper, end_devices);
    // --- Install Helper to Gateway --- //
    lora_phy_helper.SetDeviceType (LoraPhyHelper::GW);
    lora_mac_helper.SetDeviceType (LorawanMacHelper::GW);
    NetDeviceContainer gw_net_devices = lora_helper.Install (lora_phy_helper, lora_mac_helper, gateways);
    // --- Install EndDevices and Gateway --- //
    lora_mac_helper.SetSpreadingFactorsUp (end_devices, gateways, channel);
    // --- Install Energy Consumption --- //
    basic_src_helper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (10000));
    basic_src_helper.Set ("BasicEnergySupplyVoltageV", DoubleValue (3.3));
    radio_energy_helper.Set ("StandbyCurrentA", DoubleValue (0.0014));
    radio_energy_helper.Set ("TxCurrentA", DoubleValue (0.028));
    radio_energy_helper.Set ("SleepCurrentA", DoubleValue (0.0000015));
    radio_energy_helper.Set ("RxCurrentA", DoubleValue (0.0112));
    radio_energy_helper.SetTxCurrentModel ("ns3::ConstantLoraTxCurrentModel","TxCurrent", DoubleValue (0.028));
    EnergySourceContainer lora_energy_sources = basic_src_helper.Install (end_devices);
    Names::Add ("/Names/EnergySource", lora_energy_sources.Get (0));
    DeviceEnergyModelContainer device_energy_models = radio_energy_helper.Install(ed_net_devices, lora_energy_sources);
    // [INIT] LoRaWAN Network Server
    network_servers.Create (LORAWAN_NETWORK_SERVER_NUM);
    network_server_helper.SetGateways (gateways);
    network_server_helper.SetEndDevices (end_devices);
    network_server_helper.EnableAdr (true);
    network_server_helper.SetAdr ("ns3::AdrComponent");
    lora_network_apps = network_server_helper.Install (network_servers);
    // Connect Gateway 
    forwarderHelper.Install (gateways);

    // --- [INIT] BLE Setting --- //
    ble_net_devices = ble_helper.Install(end_devices);
    // Set Adresses
    NS_LOG_INFO("[INFO] Set BLE Device Address");
    for (uint32_t i = 0; i < end_devices.GetN(); i++)
    {
      std::stringstream stream;
      stream << std::hex << i+1;
      std::string s( stream.str());
      while (s.size() < 4)
        s.insert(0,1,'0');
      s.insert(2,1,':');
      char const * buffer = s.c_str();
      DynamicCast<BleNetDevice>(ble_net_devices.Get(i))->SetAddress (Mac16Address (buffer));
      tarako_nodes[i].ble_network_addr = buffer;
      NS_LOG_INFO ("address = " << DynamicCast<BleNetDevice>(ble_net_devices.Get(i))->GetAddress ());
    }
    // Create Links between The Nodes  
    const bool LINK_SCHEDULED = true; // Schedule the TX windows instead of random parameters.
    const uint32_t NB_CONN_INTERVAL = 3200; 
    // [MAX 3200]  nbConnInterval*1,25ms = size of connection interval. 
    // if nbConnInterval = 0, each link will get a random conn interval
    ble_helper.CreateAllLinks(ble_net_devices, LINK_SCHEDULED, NB_CONN_INTERVAL);
    // --- [INIT] trace_node_data_map && trace_garbage_box_sensor_map --- //
    NS_LOG_INFO("[INIT] init trace_node_data_map && trace_garbage_box_sensor_map");
    for (int i=0; i < (int)ed_net_devices.GetN(); i++) {
        // init object
        tarako::TarakoNodeData node_data = tarako_nodes[i];
        ns3::Time activate_time = Seconds(60*i+1);
        node_data.conn_interval = activate_time;
        node_data.current_status = tarako::TarakoNodeStatus::only_lorawan;
        node_data.total_energy_consumption = 0;
        node_data.lora_energy_consumption = 0;
        node_data.ble_energy_consumption = 0;
        // add net devices
        node_data.ble_net_device = ble_net_devices.Get(i)->GetObject<BleNetDevice>();
        node_data.lora_net_device = ed_net_devices.Get(i)->GetObject<LoraNetDevice>();
        node_data.lora_network_addr = node_data.lora_net_device->GetMac()->GetObject<EndDeviceLorawanMac>()->GetDeviceAddress().GetNwkAddr();
        
        bool is_multiple_node = false;
        int target_index = 0;
        for (auto& conn: available_connections)
        {
            for (auto& node_id: conn)
            {
                if (node_id == i)
                {
                    is_multiple_node = true;
                    break;
                }
            }
            if(is_multiple_node) break;
            target_index++;
        }

        if (is_multiple_node) 
        {
            NS_LOG_INFO("checkpoint: (available_connections) size -> " << available_connections.size());
            NS_LOG_INFO("checkpoint: (target index) -> " << target_index);
            auto conn = available_connections.at(target_index);
            std::cout << "checkpoint" << std::endl;
            for (auto& node_id: conn)
            {
                if (node_id != i)
                {
                    std::cout << "found" << std::endl;
                    auto lora_nd = ed_net_devices.Get(node_id)->GetObject<LoraNetDevice>();
                    auto ble_net_addr = tarako_nodes[node_id].ble_network_addr;
                    auto lora_net_addr = lora_nd->GetMac()->GetObject<EndDeviceLorawanMac>()->GetDeviceAddress().GetNwkAddr();
                    node_data.group_node_addrs.push_back({lora_net_addr,ble_net_addr});
                }
            }
        }
        // add node_data to hash map
        trace_node_data_map[node_data.lora_network_addr] = node_data;
        // init garbage sensor
        tarako::GarbageBoxSensor garbage_box_sensor;
        garbage_box_sensor.current_volume = 0;
        trace_garbage_box_sensor_map[node_data.lora_network_addr] = garbage_box_sensor; 
    }
    // --- [Declare] Trace --- //
    NS_LOG_INFO("[TRACE] tarako::OnPacketRecievedAtNetworkServer");
    Ptr<NetworkServer> ns = lora_network_apps.Get(0)->GetObject<NetworkServer>();
    ns->TraceConnectWithoutContext(
        "ReceivedPacket", 
        MakeBoundCallback(&tarako::OnPacketRecievedAtNetworkServerForGroup, &trace_node_data_map)
    );
    for (auto itr = trace_node_data_map.begin(); itr != trace_node_data_map.end(); ++itr)
    {
        // Energy Consumption
        NS_LOG_INFO("[TRACE] tarako::OnLoRaWANEnergyConsumptionChange");
        uint index = itr->second.id - 1;
        device_energy_models.Get(index) -> TraceConnectWithoutContext(
            "TotalEnergyConsumption", 
            MakeBoundCallback(&tarako::OnLoRaWANEnergyConsumptionChangeForGroup, &trace_node_data_map.at(itr->second.lora_network_addr))
        );
        Simulator::Schedule(
            itr->second.conn_interval, 
            &tarako::OnActivateNodeForGroup, 
            &trace_garbage_box_sensor_map.at(itr->second.lora_network_addr),
            &itr->second
        );
        NS_LOG_INFO(std::endl << std::fixed << itr->second.ToString());
    }
    // [INFO] Simulation
    Time simulationTime = Hours(1);
    Simulator::Stop (simulationTime);
    Simulator::Run ();
    Simulator::Destroy ();
    // [Output] Result
    for (auto itr = trace_node_data_map.begin(); itr != trace_node_data_map.end(); ++itr)
    {
        std::cout << itr->second.ToString() << std::endl;
    }
    // --- Write Log ---
    const std::string log_path = "./scratch/heterogeneous_wireless/"; 
    // tarako_logger.WriteLog(trace_node_map, log_path);
//     WriteLog(node_map);
//     LoraPacketTracker &tracker = helper.GetPacketTracker ();
//     std::cout << tracker.CountMacPacketsGlobally (Seconds (0), simulationTime + Minutes (1)) << std::endl;

    return 0;
}