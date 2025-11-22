#include "../include/RTSerial.hpp"
#include "../include/CRC8.hpp"


struct __attribute__((packed)) Packet{

    uint8_t header;       // 0xA5
    uint8_t target_id;    // 0xEA
    uint8_t length;       // 0x1A (26)
    uint8_t cmd_id;       // 0x35
    uint8_t head_chk;     // 0xA6
    uint32_t timestamp;   // 0x7A100000 (Little Endian) or ID
    float q0;             // w
    float q1;             // x
    float q2;             // y
    float q3;             // z
    uint8_t checksum;     // 校验和
} ;

auto start = std::chrono::steady_clock::now();
static int num =0;

int main() {

    std::cout<<sizeof(Packet)<<std::endl;

    RTSerial<Packet> ser(10);

    std::function<bool(const Packet&)> check_fuc = CRC8::Check<Packet>;
    ser.setCheckfuc(check_fuc);
    int ret = ser.openDevice("/dev/ttyACM0", 460800);
    
    if(ret == 1)
    {
        std::cout<<"serial open ok"<<std::endl;
    }
    std::cout<<ser.isDeviceOpen()<<std::endl;
    
    ser.startReceive(500);

    while (true) {
        // Packet test;
        std::chrono::steady_clock::time_point time ;
        Packet test;
        bool ret = ser.readPacket(test, time);

        if(!ret)
        {
            // std::cout<<"dont have data"<<std::endl;
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        num++;
        if(num>1000)
        {
            std::cout<<num/((std::chrono::steady_clock::now()-start).count()*1e-9)<<std::endl;
            num = 0;
            start = std::chrono::steady_clock::now();
        }
        std::cout<<test.q0<<" "<<test.q1<<" "<<test.q2<<" " <<test.q3<<std::endl;
    }
    return 0;
}

