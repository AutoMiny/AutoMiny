#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
typedef int16_t speed_MMpS_t;
#define FLAWLESS_PROTOCOL_PACKET_CHAR_BEGINNING 0xf0
#define FLAWLESS_PROTOCOL_PACKET_CHAR_END       0x0f
#define FLAWLESS_PROTOCOL_PACKET_CHAR_ESCAPE    0x3c
#define RPC_SUB_PROTOCOL_IDENTIFIER 1U
typedef uint8_t flawLessBaseProtocol_token_t;
typedef uint8_t flawLessBaseProtocol_subProtocolID_t;
typedef uint16_t flawLessBaseProtocol_payloadLen_t;
typedef uint8_t flawLessBaseProtocol_checkSum_t;

void my_sleep(unsigned long milliseconds) {
    usleep(milliseconds*1000); // 100 ms
}
int run(int argc, char **argv)
{
  string port("/dev/ttySAC2");
  unsigned long baud = 921600;
  // port, baudrate, timeout in milliseconds
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;

  // Get the Test string
  int count = 0;
  //string test_string;
  const flawLessBaseProtocol_payloadLen_t outPacketLen = 4;
  uint8_t *rpcOutPacket = new uint8_t[outPacketLen];
      rpcOutPacket[0] = 8;
      *((uint8_t*)&(rpcOutPacket[1])) = 0U;
      int16_t speed=0XFFFF;
      const void *buffer;
      buffer=&speed;
      memcpy(&(rpcOutPacket[2]), buffer, 2);
      //rpcOutPacket[16]=100000;
      //test_string=(string*)&;
      //std::string test_string(rpcOutPacket,rpcOutPacket+sizeof(rpcOutPacket));
      printf("%d,%d,%d,%d \n",rpcOutPacket[0],rpcOutPacket[1],rpcOutPacket[2],rpcOutPacket[3]);

      const uint8_t *data = (const uint8_t*)rpcOutPacket;
      uint8_t i = 0U;
      /* send packet start */
      const uint8_t startByte = FLAWLESS_PROTOCOL_PACKET_CHAR_BEGINNING;
      const uint8_t stopByte = FLAWLESS_PROTOCOL_PACKET_CHAR_END;
      flawLessBaseProtocol_checkSum_t checksum = 0U;
      size_t bytes_wrote =0;
      std::string result="";
      while (true) {

      checksum=0;
      bytes_wrote =my_serial.write(&startByte, sizeof(startByte));
       result = my_serial.read(bytes_wrote);
      printf("%d \n",result[0]);
      //bytes_wrote =my_serial.write("a");


      const uint8_t i_byte=RPC_SUB_PROTOCOL_IDENTIFIER;
      bytes_wrote = my_serial.write(&i_byte, sizeof(i_byte));
      checksum+=i_byte;
       result = my_serial.read(bytes_wrote);
      printf("%X \n",result[0]);
      for (i = 0U; i < outPacketLen; ++i)
      {
          bytes_wrote =my_serial.write(&data[i], sizeof(data[i]));
           result = my_serial.read(bytes_wrote);
          printf("%X \n",static_cast<unsigned int>(result[0]));
          checksum+=data[i];
      }
       flawLessBaseProtocol_checkSum_t calculatedCheckSum = ~checksum  + 1U;
       bytes_wrote =my_serial.write(&calculatedCheckSum, sizeof(calculatedCheckSum));
       result = my_serial.read(bytes_wrote);
       printf("%X \n",result[0]);

      bytes_wrote = my_serial.write(&stopByte, sizeof(stopByte));
      my_sleep(1000);
      result = my_serial.read(bytes_wrote);
      printf("%X \n",result[0]);

      cout << result.length() << ", String read: " << result << endl;
      my_sleep(1000);

       }
//while(true)
//{
//  //cout<<test_string<<endl;
//  const uint8_t *data = (const uint8_t*)rpcOutPacket;
//  size_t bytes_wrote = my_serial.write(data,4);
//cout << bytes_wrote << ", Bytes read: ";

////  bytes_wrote = my_serial.write("0U");
////  bytes_wrote = my_serial.write("255");
//}
  //cout << "Iteration: " << count << ", Bytes written: ";
  //cout << bytes_wrote << ", Bytes read: ";
  //cout << result.length() << ", String read: " << result << endl;
  return 0;
}
int main(int argc, char **argv) {
    return run(argc, argv);
}
