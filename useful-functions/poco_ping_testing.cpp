/// ping function fot windows10


#if 0

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <string.h>

int main(int argc,char *argv[])
{
    // ping /?

    if (system( "ping 192.168.2.113 -n 1 -w 800") )
    {
        printf("internet connx failed \n");
    }
    else {
        printf("internet connx OK ! :) \n");
    }
}

#endif

/// Determine if Linux or Windows in C++

#if 1

#include <iostream>
#include <string>

#include <chrono>
#include <thread>

#include <Poco/Net/ICMPClient.h>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
static const std::string slash="\\";
#else
static const std::string slash="/";
#endif

struct Timer
{
    std::chrono::time_point<std::chrono::steady_clock> start,end;
    std::chrono::duration<float> duration;

    Timer()
    {
        start = std::chrono::high_resolution_clock::now();
    }

    ~Timer()
    {
        end = std::chrono::high_resolution_clock::now();
        duration = end - start;

        float ms = duration.count() * 1000.0f;
        std::cout << "Timer took " << ms << "ms" << std::endl;
    }
};


bool IsArmControlBoxPowerOn()
{
    Poco::Net::AddressFamily family;

    Poco::Net::ICMPClient icmpClient(family.IPv4);

    auto result = icmpClient.ping("192.168.7.29",3);

    if(result == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

int main()
{
    Poco::Net::AddressFamily family;

    Poco::Net::ICMPClient icmpClient(family.IPv4);


    while (1)
    {
        Timer timer;

        auto result = IsArmControlBoxPowerOn();

//        auto result = icmpClient.ping("192.168.7.29",3);

//        std::cout << "Received Number: " << result << std::endl;

    }


    //std::cout << "slash is: " << slash << std::endl;

    return 1;
}

#endif


# if 0

#include "Poco/Util/Application.h"
#include "Poco/Util/Option.h"
#include "Poco/Util/OptionSet.h"
#include "Poco/Util/HelpFormatter.h"
#include "Poco/Util/AbstractConfiguration.h"
#include "Poco/Net/ICMPSocket.h"
#include "Poco/Net/ICMPClient.h"
#include "Poco/Net/IPAddress.h"
#include "Poco/Net/ICMPEventArgs.h"
#include "Poco/AutoPtr.h"
#include "Poco/NumberParser.h"
#include "Poco/Delegate.h"
#include <iostream>
#include <sstream>


using Poco::Util::Application;
using Poco::Util::Option;
using Poco::Util::OptionSet;
using Poco::Util::HelpFormatter;
using Poco::Util::AbstractConfiguration;
using Poco::Net::ICMPSocket;
using Poco::Net::ICMPClient;
using Poco::Net::IPAddress;
using Poco::Net::ICMPEventArgs;
using Poco::AutoPtr;
using Poco::NumberParser;
using Poco::Delegate;


class Ping: public Application
        /// This sample demonstrates the Poco::Net::ICMPClient in conjunction with
        /// Poco Foundation C#-like events functionality.
        ///
        /// Try Ping --help (on Unix platforms) or Ping /help (elsewhere) for
        /// more information.
        {
        public:
            Ping():
            _helpRequested(false),
            _icmpClient(IPAddress::IPv4),
            _repetitions(4),
            _target("localhost")
            {
            }

        protected:
            void initialize(Application& self)
            {
                loadConfiguration(); // load default configuration files, if present
                Application::initialize(self);

                _icmpClient.pingBegin += delegate(this, &Ping::onBegin);
                _icmpClient.pingReply += delegate(this, &Ping::onReply);
                _icmpClient.pingError += delegate(this, &Ping::onError);
                _icmpClient.pingEnd   += delegate(this, &Ping::onEnd);
            }

            void uninitialize()
            {
                _icmpClient.pingBegin -= delegate(this, &Ping::onBegin);
                _icmpClient.pingReply -= delegate(this, &Ping::onReply);
                _icmpClient.pingError -= delegate(this, &Ping::onError);
                _icmpClient.pingEnd   -= delegate(this, &Ping::onEnd);

                Application::uninitialize();
            }

            void defineOptions(OptionSet& options)
            {
                Application::defineOptions(options);

                options.addOption(
                        Option("help", "h", "display help information on command line arguments")
                        .required(false)
                        .repeatable(false));

                options.addOption(
                        Option("repetitions", "r", "define the number of repetitions")
                        .required(false)
                        .repeatable(false)
                        .argument("repetitions"));

                options.addOption(
                        Option("target", "t", "define the target address")
                        .required(false)
                        .repeatable(false)
                        .argument("target"));
            }

            void handleOption(const std::string& name, const std::string& value)
            {
                Application::handleOption(name, value);

                if (name == "help")
                    _helpRequested = true;
                else if (name == "repetitions")
                    _repetitions = NumberParser::parse(value);
                else if (name == "target")
                    _target = value;
            }

            void displayHelp()
            {
                HelpFormatter helpFormatter(options());
                helpFormatter.setCommand(commandName());
                helpFormatter.setUsage("OPTIONS");
                helpFormatter.setHeader(
                        "A sample application that demonstrates the functionality of the "
                        "Poco::Net::ICMPClient class in conjunction with Poco::Events package functionality.");
                helpFormatter.format(std::cout);
            }


            int main(const std::vector<std::string>& args)
            {
                if (_helpRequested)
                    displayHelp();
                else
                    _icmpClient.ping(_target, _repetitions);

                return Application::EXIT_OK;
            }


            void onBegin(const void* pSender, ICMPEventArgs& args)
            {
                std::ostringstream os;
                os << "Pinging " << args.hostName() << " [" << args.hostAddress() << "] with " << args.dataSize() << " bytes of data:"
                << std::endl << "---------------------------------------------" << std::endl;
                logger().information(os.str());
            }

            void onReply(const void* pSender, ICMPEventArgs& args)
            {
                std::ostringstream os;
                os << "Reply from " << args.hostAddress()
                << " bytes=" << args.dataSize()
                << " time=" << args.replyTime() << "ms"
                << " TTL=" << args.ttl();
                logger().information(os.str());
            }

            void onError(const void* pSender, ICMPEventArgs& args)
            {
                std::ostringstream os;
                os << args.error();
                logger().information(os.str());
            }

            void onEnd(const void* pSender, ICMPEventArgs& args)
            {
                std::ostringstream os;
                os << std::endl << "--- Ping statistics for " << args.hostName() << " ---"
                << std::endl << "Packets: Sent=" << args.sent() << ", Received=" << args.received()
                << " Lost=" << args.repetitions() - args.received() << " (" << 100.0 - args.percent() << "% loss),"
                << std::endl << "Approximate round trip times in milliseconds: " << std::endl
                << "Minimum=" << args.minRTT() << "ms, Maximum=" << args.maxRTT()
                << "ms, Average=" << args.avgRTT() << "ms"
                << std::endl << "------------------------------------------";
                logger().information(os.str());
            }

        private:
            bool        _helpRequested;
            ICMPClient  _icmpClient;
            int         _repetitions;
            std::string _target;
        };


int main(int argc, char** argv)
{
    AutoPtr<Ping> pApp = new Ping;
    try
    {
        pApp->init(argc, argv);
    }
    catch (Poco::Exception& exc)
    {
        pApp->logger().log(exc);
        return Application::EXIT_CONFIG;
    }
    return pApp->run();
}

#endif