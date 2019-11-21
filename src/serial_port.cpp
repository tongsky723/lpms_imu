

#include "lpms_imu/serial_port.h"

SerialPort::SerialPort(void) : end_of_line_char_('\n'),buf_(NULL)
{

}

SerialPort::~SerialPort(void)
{
	stop();
}

char SerialPort::end_of_line_char() const
{
	return this->end_of_line_char_;
}

void SerialPort::end_of_line_char(const char &c)
{
	this->end_of_line_char_ = c;
}
std::vector<std::string> SerialPort::get_port_names()
{
	std::vector<std::string> names;
	return names;
}

int SerialPort::get_port_number()
{
	std::vector<std::string> names = get_port_names();
	return names.size();
}

std::string SerialPort::get_port_name(const unsigned int &idx)
{
	std::vector<std::string> names = get_port_names();
	if (idx >= names.size()) return std::string();
	return names[idx];
}

void SerialPort::print_devices()
{
	std::cout << "SerialPort::print_devices()" << std::endl;
	int n = SerialPort::get_port_number();
	for (int i = 0; i < n; ++i) {
		std::string name = SerialPort::get_port_name(i);
		std::cout << "\t" << name.c_str() << std::endl;
	}
}

bool SerialPort::start(const char *com_port_name, int baud_rate)
{
    port_name_=com_port_name;
	boost::system::error_code ec;

	if (port_) {
		std::cout << "error : port is already opened..." << std::endl;
		return false;
	}

	port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
	port_->open(com_port_name, ec);
	if (ec) {
    ROS_ERROR("error : port_->open() failed...com_port_name=%s,e=%s",com_port_name,ec.message().c_str());
//		std::cout << "error : port_->open() failed...com_port_name="
//			<< com_port_name << ", e=" << ec.message().c_str() << std::endl;
		return false;
	}

	// option settings...
	port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	port_->set_option(boost::asio::serial_port_base::character_size(8));
	port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

//	boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));



    boost::thread t(boost::bind(&SerialPort::run, this));
      async_read_some_();
      return true;
    }





void SerialPort::run()
{
    while(ros::ok())
    {
        io_service_.run();
        io_service_.reset();
    }
}

void SerialPort::reset()
{
    if (port_) {
        port_->close();
        port_.reset();
    }
    io_service_.reset();
}

void SerialPort::stop()
{
	boost::mutex::scoped_lock look(mutex_);

	if (port_) {
		port_->cancel();
		port_->close();
		port_.reset();
	}
	io_service_.stop();
	io_service_.reset();
}

int SerialPort::write_some(const std::string &buf)
{
	return write_some(buf.c_str(), buf.size());
}

int SerialPort::write_some(const char *buf, const int &size)
{
	boost::system::error_code ec;

	if (!port_) return -1;
	if (size == 0) return 0;

	return port_->write_some(boost::asio::buffer(buf, size), ec);
}

void SerialPort::async_read_some_()
{
	if (port_.get() == NULL || !port_->is_open()) return;
    //读满才返回
    async_read(*port_,boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
               boost::bind(
                   &SerialPort::on_receive_,
                   this, boost::asio::placeholders::error,
                   boost::asio::placeholders::bytes_transferred));
      //有数据就返回
//    port_->async_read_some(
//        boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
//        boost::bind(
//            &SerialPort::on_receive_,
//            this, boost::asio::placeholders::error,
//            boost::asio::placeholders::bytes_transferred));

//    boost::asio::async_read_until(*port_,sbuf_, '3a',
//                                  boost::bind(&SerialPort::on_receive_,this, boost::asio::placeholders::error,
//                                  boost::asio::placeholders::bytes_transferred));

}

void SerialPort::on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)
{

    if (port_.get() == NULL || !port_->is_open()) return;
    if (ec) {
        async_read_some_();
        return;
    }
    boost::mutex::scoped_lock look(mutex_);
    for (unsigned int i = 0; i < bytes_transferred; ++i)
    {
    buffer_.push_back(read_buf_raw_[i]);
//    printf("i=%d,%02x\r\n",i,read_buf_raw_[i]);
    }
	async_read_some_();
}


bool SerialPort::get_data(std::vector<u_char>*data)
{
    if(buffer_.size()>=SERIAL_PORT_READ_BUF_SIZE*2)
    {
        boost::mutex::scoped_lock look(mutex_);
        data->assign(buffer_.begin(),buffer_.end());
        buffer_.erase(buffer_.begin(),buffer_.begin()+SERIAL_PORT_READ_BUF_SIZE);//删除上一个数据
        return true;
    }
    else {
        return false;
    }

}

