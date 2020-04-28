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
#include <ns3/lr-wpan-net-device.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/single-model-spectrum-channel.h>
#include "ns3/lr-wpan-helper.h"
#include "ns3/lr-wpan-mac-header.h"

#include <algorithm>
#include <string>
#include <vector>
#include <unordered_map> 

// using namespace std;
using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("HeterogeneousWirelessNetworkModel");
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
LrWpanHelper lr_wpan_helper;
Ptr<SingleModelSpectrumChannel> lr_wpan_channel = CreateObject<SingleModelSpectrumChannel> ();
// --- Global Const Variable --- //
const std::string GARBAGE_BOX_MAP_FILE = "/home/vagrant/workspace/tozastation/ns-3.30/scratch/test.csv";
const int LORAWAN_GATEWAY_NUM = 3;
const int LORAWAN_NETWORK_SERVER_NUM = 1;
const double BLE_RX_POWER = 0.003;
const double BLE_TX_POWER = 0.003;
// --- Global Variable --- //
int cnt_node = 0;
std::vector<std::vector<std::tuple<int, bool>>> available_connections;
std::vector<tarako::TarakoNodeData> tarako_nodes;
std::unordered_map<int, tarako::TarakoNodeData> trace_node_data_map;
std::unordered_map<int, tarako::GarbageBoxSensor> trace_garbage_box_sensor_map;

// LrWpan Callback
static void DataIndication (tarako::TarakoNodeData* node_data , McpsDataIndicationParams params, Ptr<Packet> packet)
{
    std::ostringstream src_addr;
    src_addr << params.m_srcAddr;
    switch (node_data->current_status)
    {
        case tarako::TarakoNodeStatus::group_leader:
        {
            // [Function] Send by LoRaWAN
            int group_member_n = node_data->group_node_addrs.size();
            int buffered_packet_n = node_data->buffered_packets.size();
            std::vector<std::tuple<std::string, double>> node_energy_consumptions;
            double total_energy_consumption;
            if (group_member_n == buffered_packet_n)
            {
                tarako::TarakoGroupLeaderPayload gl_payload;
                for (auto& bp: node_data->buffered_packets)
                {
                    std::string sent_from = std::get<0>(bp);
                    Ptr<Packet> data_sent = std::get<1>(bp);

                    uint8_t *data_sent_buffer = new uint8_t[data_sent->GetSize()];
                    data_sent->CopyData(data_sent_buffer, data_sent->GetSize());
                    tarako::TarakoGroupMemberPayload gm_payload;
                    memcpy(&gm_payload, data_sent_buffer, sizeof(gm_payload));
                    gl_payload.node_infos.push_back({sent_from, gm_payload.status});
                    total_energy_consumption = gm_payload.lora_energy_consumption + (gm_payload.cnt_ble * 0.00005);
                    node_energy_consumptions.push_back({sent_from, total_energy_consumption});
                }
                total_energy_consumption = node_data->lora_energy_consumption + (node_data->sent_packets_by_ble.size() * 0.00005);
                node_energy_consumptions.push_back({src_addr.str(), total_energy_consumption});
                // TODO: Garbage Box Sensor
                unsigned int random_value = tarako::TarakoUtil::CreateRandomInt(
                    tarako::TarakoConst::RANDOM_VALUE_RANGE_BEGIN,
                    tarako::TarakoConst::RANDOM_VALUE_RANGE_END
                );
                // std::cout << "[INFO] add random value to garbage box: " << random_value << std::endl;
                tarako::GarbageBoxStatus status =  tarako::TarakoUtil::GetGarbageBoxStatus(
                    &node_data->sensor, 
                    random_value,
                    tarako::TarakoConst::GARBAGE_BOX_VOLUME
                );
                gl_payload.node_infos.push_back({node_data->ble_network_addr, status});
                Ptr<Packet> new_packet = Create<Packet>((uint8_t *)&gl_payload, sizeof(gl_payload));
                node_data->lora_net_device->Send(new_packet);
                node_data->sent_packets_by_lora.push_back(new_packet);

                NS_LOG_INFO("----  DataIndication.SendAggregatePayload()  ----");
                NS_LOG_INFO("(lora network addr) : " << node_data->lora_network_addr);
                NS_LOG_INFO("(data rate)         : " << node_data->lora_net_device->GetMac()->GetObject<EndDeviceLorawanMac>()->GetDataRate());
                NS_LOG_INFO("(data size)         : " << sizeof(gl_payload));
                NS_LOG_INFO("-------------------------------------------------");

                // Judge Next Leader
                auto next_leader_id = tarako::TarakoUtil::GetNextGroupLeader(node_energy_consumptions);
                if (next_leader_id != node_data->leader_node_addr) 
                {
                    tarako::TarakoGroupDownlink downlink = tarako::TarakoGroupDownlink{next_leader_id}; 
                    for (auto& node: node_data->group_node_addrs)
                    {
                        Ptr<Packet> new_packet = Create<Packet>((uint8_t *)&downlink, sizeof(downlink));
                        McpsDataRequestParams params;
                        params.m_dstPanId    = 0;
                        params.m_srcAddrMode = SHORT_ADDR;
                        params.m_dstAddrMode = SHORT_ADDR;
                        params.m_dstAddr     = Mac16Address (std::get<1>(node).c_str());
                        node_data->lr_wpan_net_device->GetMac()->McpsDataRequest(params, new_packet);
                        NS_LOG_INFO("----  DataIndication.JudgeNextLeader()  ----");
                        NS_LOG_INFO("(i am)        : " << node_data->ble_network_addr);
                        NS_LOG_INFO("(next leader) : " <<  next_leader_id);
                        NS_LOG_INFO("(notify to)   : " <<  std::get<1>(node).c_str());
                        NS_LOG_INFO("--------------------------------------------");
                    }
                    // Update My Node Status -> Leader to Member
                    node_data->current_status   = tarako::TarakoNodeStatus::group_member;
                    node_data->leader_node_addr = next_leader_id;
                }
                node_data->received_packets_by_ble.push_back(packet);
                break;
            }
            else
            {
                node_data->buffered_packets.push_back({src_addr.str(), packet});

                Ptr<Packet> received_packet = packet->Copy ();
                uint8_t *received_packet_buffer = new uint8_t[received_packet->GetSize()];
                received_packet->CopyData(received_packet_buffer, received_packet->GetSize());
                tarako::TarakoGroupMemberPayload received_gm_payload;
                memcpy(&received_gm_payload, received_packet_buffer, sizeof(received_gm_payload));

                NS_LOG_INFO("----  DataIndication.PushToBufferdPackets()  ----");
                NS_LOG_INFO("(i am)           : " << node_data->ble_network_addr);
                NS_LOG_INFO("(payload.status) : " << received_gm_payload.status);
                NS_LOG_INFO("(flow)           : " << params.m_srcAddr << " -> " << params.m_dstAddr);
                NS_LOG_INFO("-------------------------------------------------");
                node_data->received_packets_by_ble.push_back(packet);
                break;
            }
        }
        case tarako::TarakoNodeStatus::group_member:
        {
            Ptr<Packet> received_packet = packet->Copy ();
            uint8_t *received_packet_buffer = new uint8_t[received_packet->GetSize()];
            received_packet->CopyData(received_packet_buffer, received_packet->GetSize());
            tarako::TarakoGroupDownlink received_group_leader;
            memcpy(&received_group_leader, received_packet_buffer, sizeof(received_group_leader));
            tarako::TarakoNodeStatus previous_status = node_data->current_status;
            if (node_data->ble_network_addr == received_group_leader.next_leader_id)
            {
                node_data->current_status = tarako::TarakoNodeStatus::group_leader;
            }
            node_data->leader_node_addr = received_group_leader.next_leader_id;
            NS_LOG_INFO("----  DataIndication.ReceivedFromGL()  ----");
            NS_LOG_INFO("(i am)            : " << node_data->ble_network_addr);
            NS_LOG_INFO("(flow)            : " << params.m_srcAddr << " -> " << params.m_dstAddr);
            NS_LOG_INFO("(next leader)     : " << received_group_leader.next_leader_id);
            NS_LOG_INFO("(previous status) : " << previous_status);
            NS_LOG_INFO("(current status)  : " << node_data->current_status);
            NS_LOG_INFO("--------------------------------------------");
            node_data->received_packets_by_ble.push_back(packet);
            break;
        }
        default:
        {}
    }
}

static void DataConfirm (McpsDataConfirmParams params)
{
    // std::cout << "    ----  DataConfirm  ---- " <<  std::endl;
    // std::cout << "(connection status)        : " << params.m_status <<  std::endl;
    // std::cout << "    -------------------------------------------------    " <<  std::endl;
}

int main (int argc, char *argv[])
{
    // --- Logging --- //
    LogComponentEnable ("HeterogeneousWirelessNetworkModel", LOG_LEVEL_ALL);
    LogComponentEnable ("TarakoTracer", LOG_LEVEL_ALL);
    LogComponentEnableAll (LOG_PREFIX_FUNC);
    LogComponentEnableAll (LOG_PREFIX_NODE);
    LogComponentEnableAll (LOG_PREFIX_TIME);
    // --- Set the EDs to require Data Rate control from the NS --- //
    Config::SetDefault ("ns3::EndDeviceLorawanMac::DRControl", BooleanValue (true));
    PacketMetadata::Enable();
    // lr_wpan_helper.EnableLogComponents();
    // [CSV READ] Garbage Box
    const auto garbage_boxes = tarako::TarakoUtil::GetGarbageBox(GARBAGE_BOX_MAP_FILE);  
    // [ADD] garbage boxes geolocation in allocator 
    for (const auto& g_box: garbage_boxes) {
        std::vector<std::tuple<int, bool>> available_connection;
        if (g_box.burnable) {
            ns3::Vector3D pos = Vector (g_box.latitude, g_box.longitude,1);
            ed_allocator->Add (pos);
            
            tarako::TarakoNodeData node;
            node.id = cnt_node;
            node.position = pos;
            tarako_nodes.push_back(node);
            available_connection.push_back({cnt_node, false});
            cnt_node++;
        }
        if (g_box.incombustible) {
            ns3::Vector3D pos = Vector (g_box.latitude + 0.000001, g_box.longitude,1);
            ed_allocator->Add(pos);

            tarako::TarakoNodeData node;
            node.id = cnt_node;
            node.position = pos;
            tarako_nodes.push_back(node);
            available_connection.push_back({cnt_node, false});
            cnt_node++;
        }
        if (g_box.resource) {
            ns3::Vector3D pos = Vector (g_box.latitude + 0.000002, g_box.longitude,1);
            ed_allocator->Add(pos);

            tarako::TarakoNodeData node;
            node.id = cnt_node;
            node.position = pos;
            tarako_nodes.push_back(node);
            available_connection.push_back({cnt_node, false});
            cnt_node++;
        }
        // select leader node
        int available_cnt = available_connection.size();
        int leader_node = tarako::TarakoUtil::CreateRandomInt(0, available_cnt-1);
        std::get<1>(available_connection.at(leader_node)) = true;
        // add connection
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
    if (tarako::TarakoConst::EnableGrouping) 
    {
        NS_LOG_INFO("(EnableGrouping) : true");
        ble_net_devices = ble_helper.Install(end_devices);
        for (uint32_t i = 0; i < end_devices.GetN(); i++)
        {
            std::stringstream stream;
            stream << std::hex << i+1;
            std::string s( stream.str());
            while (s.size() < 4)
                s.insert(0,1,'0');
            s.insert(2,1,':');
            char const * buffer = s.c_str();
            tarako_nodes[i].ble_network_addr = buffer;
        }
        // Setter Lr-Wpan Channel
        Ptr<LogDistancePropagationLossModel> lr_wpan_prop_model = CreateObject<LogDistancePropagationLossModel> ();
        Ptr<ConstantSpeedPropagationDelayModel> lr_wpan_delay_model = CreateObject<ConstantSpeedPropagationDelayModel> ();
        lr_wpan_channel->AddPropagationLossModel (lr_wpan_prop_model);
        lr_wpan_channel->SetPropagationDelayModel (lr_wpan_delay_model); 
    }

    // --- [INIT] trace_node_data_map && trace_garbage_box_sensor_map --- //
    NS_LOG_INFO("[INIT] init trace_node_data_map && trace_garbage_box_sensor_map");
    for (int i=0; i < (int)ed_net_devices.GetN(); i++) {
        // [Init] object initial value {TarakoNodeData}
        tarako::TarakoNodeData node_data = tarako_nodes[i];
        node_data.conn_interval = Minutes(10);
        node_data.total_energy_consumption = 0;
        node_data.lora_energy_consumption = 0;
        node_data.ble_energy_consumption = 0;
        // Common Setup
        // [Install] net devices {LoraNetDevice, LrWpanNetDevice}
        // --- Setup LoraNetDevice ---
        node_data.lora_net_device = ed_net_devices.Get(i)->GetObject<LoraNetDevice>();
        node_data.lora_network_addr = node_data.lora_net_device->GetMac()->GetObject<EndDeviceLorawanMac>()->GetDeviceAddress().GetNwkAddr();
        // [Function] Judge it is multiple nodes and target node vector index
        auto result_is_multiple_node = tarako::TarakoUtil::IsMultipleNode(available_connections, i);
        bool is_multiple_node = std::get<0>(result_is_multiple_node);
        int target_index = std::get<1>(result_is_multiple_node);
        if (tarako::TarakoConst::EnableGrouping)
        {
            // --- Setup LrWpanNetDevice ---
            Ptr<LrWpanNetDevice> lr_wpan_net_device = CreateObject<LrWpanNetDevice> ();
            lr_wpan_net_device->SetAddress(Mac16Address(tarako_nodes[i].ble_network_addr.c_str()));
            lr_wpan_net_device->SetChannel(lr_wpan_channel);
            end_devices.Get(i)->AddDevice(lr_wpan_net_device);
            node_data.lr_wpan_net_device = lr_wpan_net_device;
            node_data.lr_wpan_net_device->GetMac()->SetMcpsDataConfirmCallback(MakeCallback(&DataConfirm));
            if (is_multiple_node) 
            {  
                NS_LOG_INFO("(available_connections) size  : " << available_connections.size());
                NS_LOG_INFO("(target index)                : " << target_index);
                auto conn = available_connections.at(target_index);
                node_data.activate_time = Seconds(tarako::TarakoConst::ACTIVATION_INTERVAL * target_index);
                for (auto& conn_info: conn)
                {
                    int node_id = std::get<0>(conn_info);
                    bool this_is_leader = std::get<1>(conn_info);
                    if (this_is_leader) node_data.leader_node_addr = tarako_nodes[node_id].ble_network_addr;
                    // [Function] Register nodes {lora net addr, ble net addr} without me
                    int me = i;
                    if (node_id != me)
                    {
                        auto lora_nd = ed_net_devices.Get(node_id)->GetObject<LoraNetDevice>();
                        auto ble_net_addr = tarako_nodes[node_id].ble_network_addr;
                        auto lora_net_addr = lora_nd->GetMac()->GetObject<EndDeviceLorawanMac>()->GetDeviceAddress().GetNwkAddr();
                        node_data.group_node_addrs.push_back({lora_net_addr,ble_net_addr});
                    }
                    else 
                    {
                        // [Function] Judge Am I Which Nodes
                        if (this_is_leader) node_data.current_status = tarako::TarakoNodeStatus::group_leader;
                        else  node_data.current_status = tarako::TarakoNodeStatus::group_member;
                    }
                }
            }
            // [Function] Register Role {Group Leader, Group Member}
            else node_data.current_status = tarako::TarakoNodeStatus::only_lorawan;
        }
        else
        {
            if (is_multiple_node) 
            {  
                auto conn = available_connections.at(target_index);
                node_data.activate_time = Seconds(tarako::TarakoConst::ACTIVATION_INTERVAL * target_index);
            }
            node_data.current_status = tarako::TarakoNodeStatus::only_lorawan;
        }
        // [Init] Garbage Box Sensor
        tarako::GarbageBoxSensor garbage_box_sensor;
        garbage_box_sensor.current_volume = 0;
        trace_garbage_box_sensor_map[node_data.lora_network_addr] = garbage_box_sensor; 
        // Push node_data to hash map
        node_data.sensor = garbage_box_sensor;
        trace_node_data_map[node_data.lora_network_addr] = node_data;
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
        // --- Energy Consumption --- //
        uint index = itr->second.id;
        device_energy_models.Get(index) -> TraceConnectWithoutContext(
            "TotalEnergyConsumption", 
            MakeBoundCallback(
                &tarako::OnLoRaWANEnergyConsumptionChangeForGroup, 
                &trace_node_data_map.at(itr->second.lora_network_addr)
            )
        );
        if (tarako::TarakoConst::EnableGrouping)
        {
            itr->second.lr_wpan_net_device->GetMac()->SetMcpsDataIndicationCallback(MakeBoundCallback(&DataIndication, &itr->second));
        }
        Simulator::Schedule(
            itr->second.activate_time, 
            &tarako::OnActivateNodeForGroup, 
            &itr->second
        );
    }
    // [Simulation]
    Time simulationTime = Hours(tarako::TarakoConst::SIMURATION_TIME);
    Simulator::Stop (simulationTime);
    Simulator::Run ();
    Simulator::Destroy ();
    // --- Write Log --- //
    const std::string log_file_path = "./scratch/heterogeneous_wireless/log.csv"; 
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> log_stream = ascii.CreateFileStream(log_file_path); 
    // Write Header
    *log_stream->GetStream() << "id,";
    *log_stream->GetStream() << "pos_x,pos_y,pos_z,";
    *log_stream->GetStream() << "activation_time,connection_interval,";
    *log_stream->GetStream() << "total_energy_consumption,lora_energy_consumption,ble_energy_consumption";
    *log_stream->GetStream() << std::endl;
    double ble_rx = 0;
    double ble_tx = 0;
    double lora_all = 0;
    for (auto itr = trace_node_data_map.begin(); itr != trace_node_data_map.end(); ++itr)
    {
        *log_stream->GetStream() << itr->second.id << ",";
        *log_stream->GetStream() << itr->second.position.x << ",";
        *log_stream->GetStream() << itr->second.position.y << ",";
        *log_stream->GetStream() << itr->second.position.z << ",";
        *log_stream->GetStream() << std::fixed << itr->second.activate_time.GetSeconds() << ",";
        *log_stream->GetStream() << std::fixed << itr->second.conn_interval.GetSeconds() << ",";
        if (tarako::TarakoConst::EnableGrouping)
        {
            ble_rx = itr->second.received_packets_by_ble.size() * BLE_RX_POWER;
            ble_tx = itr->second.sent_packets_by_ble.size() * BLE_TX_POWER;
            itr->second.ble_energy_consumption = ble_tx + ble_rx;
        }
        lora_all = itr->second.lora_energy_consumption;
        itr->second.total_energy_consumption = ble_rx + ble_tx + lora_all;
        // Continue
        *log_stream->GetStream() << itr->second.total_energy_consumption << ",";
        *log_stream->GetStream() << itr->second.lora_energy_consumption << ",";
        *log_stream->GetStream() << itr->second.ble_energy_consumption << std::endl;
        std::cout << itr->second.ToString() << std::endl;
    }
    return 0;
}