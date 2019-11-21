#pragma once

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <string>
#include <vector>
#include <iostream>
#include<ros/ros.h>
#include<std_msgs/UInt8.h>
using namespace boost::asio;
typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

#define SERIAL_PORT_READ_BUF_SIZE 17

class SerialPort
{
protected:

	boost::asio::io_service io_service_;
	serial_port_ptr port_;
	boost::mutex mutex_;
	
   unsigned	char read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE];
   unsigned char* buf_;
   std::vector<u_char> buffer_;
	std::string read_buf_str_;
   boost::asio::streambuf sbuf_;
	char end_of_line_char_;
    unsigned int size_;
    std::string port_name_;

private:
	SerialPort(const SerialPort &p);
	SerialPort &operator=(const SerialPort &p); 
    void run();
public:

	SerialPort(void);
	virtual ~SerialPort(void);
    bool get_data(std::vector<u_char> *data);
	char end_of_line_char() const;
	void end_of_line_char(const char &c);

	virtual bool start(const char *com_port_name, int baud_rate=9600);
	virtual void stop();
	virtual void reset();

	int write_some(const std::string &buf);
	int write_some(const char *buf, const int &size);

	static int get_port_number();
	static std::string get_port_name(const unsigned int &idx);
	static std::vector<std::string> get_port_names();
	static void print_devices();

protected:
	virtual void async_read_some_();
	virtual void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred);

};
