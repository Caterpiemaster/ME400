#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device1, const std::string &serial_device2, int32_t baud_rate, int32_t timeout_ms)
  {  
    std::cout << "try connection" << std::endl;
    timeout_ms_ = timeout_ms;
    serial_conn_1.Open(serial_device1);
    serial_conn_1.SetBaudRate(convert_baud_rate(baud_rate));
    serial_conn_2.Open(serial_device2);
    serial_conn_2.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    std::cout << "try disconnection" << std::endl;
    serial_conn_1.Close();
    serial_conn_2.Close();
  }

  bool connected() const
  {
    if (serial_conn_1.IsOpen() && serial_conn_2.IsOpen()){
      return true;
    }
    else{
      return false;
    }
  }


  std::string send_msg_1(const std::string &msg_to_send, bool print_output = true)
  {
    serial_conn_1.FlushIOBuffers(); // Just in case
    serial_conn_1.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_1.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }

  std::string send_msg_2(const std::string &msg_to_send, bool print_output = true)
  {
    serial_conn_2.FlushIOBuffers(); // Just in case
    serial_conn_2.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_2.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }

  void send_empty_msg()
  {
    std::cout << "send empty msg" << std::endl;

    std::string response1 = send_msg_1("\r");
    std::string response2 = send_msg_2("\r");
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    std::cout << "read encoder values" << std::endl;

    std::string response1 = send_msg_1("e\r");
    std::string response2 = send_msg_1("e\r");

    std::string delimiter = " ";
    size_t del_pos1 = response1.find(delimiter);
    std::string token_1 = response1.substr(0, del_pos1);

    size_t del_pos2 = response2.find(delimiter);
    std::string token_2 = response2.substr(0, del_pos2);
    // std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
  }
  void set_motor_values(int val_1, int val_2)
  {
    std::cout << "Send Motor Values" << std::endl;
    std::stringstream ss1;
    std::stringstream ss2;
    // ss << "m " << val_1 << " " << val_2 << "\r";
    ss1 << "m " << val_1 << "\r";
    send_msg_1(ss1.str());

    ss2 << "m " << val_2 << "\r";
    send_msg_2(ss2.str()); 
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  { 
    std::cout << "set pid values" << std::endl;
    std::stringstream ss1;
    std::stringstream ss2;
    ss1 << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg_1(ss1.str());
  
    ss2 << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg_2(ss2.str());
  }

private:
    LibSerial::SerialPort serial_conn_1;
    LibSerial::SerialPort serial_conn_2;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP