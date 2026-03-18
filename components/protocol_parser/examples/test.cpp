#include "packet_generate.hpp"

void test(){
    PacketGenerate packetGenerate;
    packetGenerate.packet_generate_1_byte(dt_cmd_reboot, 0x01);
    packetGenerate.codec.Printf_format();
    
}