// gz_zoom_publisher - lines of text on stdin to a gz.msgs.Double topic.
//
// The emulated SCF4 lens controller is a Python program with no Gazebo of its
// own (modules/sim/scf4_emulator.py). It writes the field of view its lens is
// at, one radian value per line, and this puts each one on the topic the video
// streamer follows. Two single-purpose programs joined by a pipe, rather than
// Gazebo bindings inside the lens.
//
// It stays alive for as long as the lens does, which is the point: a publisher
// that advertises, publishes once and exits can be gone before gz-transport
// has finished telling the subscribers it exists, and the message goes
// nowhere. That is why the entry point's one-shot `gz topic -p` needs a sleep
// in front of it and why this does not.
//
//   scf4_emulator.py ... | gz_zoom_publisher --topic /uas11/camera/zoom
//
// Copyright (c) 2026. BSD 3-Clause, as the streamer beside it.

#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char **argv) {
  std::string topic;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--help" || a == "-h") {
      std::cout << "gz_zoom_publisher --topic <gz topic>\n"
                << "  Reads one number per line from stdin and publishes each\n"
                << "  one as a gz.msgs.Double on <topic>.\n";
      return 0;
    }
    if (a == "--topic" && i + 1 < argc) { topic = argv[++i]; continue; }
    std::cerr << "unknown option " << a << std::endl;
    return 2;
  }
  if (topic.empty()) {
    std::cerr << "gz_zoom_publisher needs --topic" << std::endl;
    return 2;
  }

  gz::transport::Node node;
  auto publisher = node.Advertise<gz::msgs::Double>(topic);
  if (!publisher) {
    std::cerr << "cannot advertise " << topic << std::endl;
    return 1;
  }
  // Discovery is not instant, and the first value is the one that says what
  // framing the lens came up at. Give the subscribers a moment to appear
  // before it goes out, rather than losing it.
  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::cerr << "gz_zoom_publisher: publishing to " << topic << std::endl;

  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      gz::msgs::Double message;
      message.set_data(std::stod(line));
      publisher.Publish(message);
    } catch (const std::exception &) {
      std::cerr << "gz_zoom_publisher: cannot read a number from " << line
                << std::endl;
    }
  }
  return 0;
}
