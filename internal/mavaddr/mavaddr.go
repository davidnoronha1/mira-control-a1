// Package mavaddr converts pymavlink style connection strings into gomavlib
// endpoints so the node accepts the same pixhawk_address values as the
// Python master (e.g. "/dev/Pixhawk" or "tcp:127.0.0.1:5760" for SITL).
package mavaddr

import (
	"fmt"
	"strconv"
	"strings"

	"github.com/bluenviron/gomavlib/v3"
)

// Parse converts address into an endpoint. Serial devices use baud unless the
// address carries its own rate ("/dev/ttyACM0:115200" or "/dev/ttyACM0,115200").
//
// Supported forms, as in pymavlink's mavlink_connection:
//
//	tcp:host:port      connect to a TCP server (ArduPilot SITL)
//	tcpin:host:port    listen for a TCP client
//	udp:host:port      listen for UDP packets (same as udpin)
//	udpin:host:port    listen for UDP packets
//	udpout:host:port   send UDP packets to host:port
//	<device path>      serial port
func Parse(address string, baud int) (gomavlib.EndpointConf, error) {
	address = strings.TrimSpace(address)
	if address == "" {
		return nil, fmt.Errorf("empty pixhawk address")
	}

	if scheme, hostPort, ok := strings.Cut(address, ":"); ok {
		switch strings.ToLower(scheme) {
		case "tcp":
			return gomavlib.EndpointTCPClient{Address: hostPort}, checkHostPort(hostPort)
		case "tcpin":
			return gomavlib.EndpointTCPServer{Address: hostPort}, checkHostPort(hostPort)
		case "udp", "udpin":
			return gomavlib.EndpointUDPServer{Address: hostPort}, checkHostPort(hostPort)
		case "udpout":
			return gomavlib.EndpointUDPClient{Address: hostPort}, checkHostPort(hostPort)
		}
	}

	device := address
	if i := strings.LastIndexAny(address, ":,"); i > 0 {
		if b, err := strconv.Atoi(address[i+1:]); err == nil {
			device, baud = address[:i], b
		}
	}
	if baud <= 0 {
		return nil, fmt.Errorf("invalid baud rate %d", baud)
	}
	return gomavlib.EndpointSerial{Device: device, Baud: baud}, nil
}

func checkHostPort(hostPort string) error {
	i := strings.LastIndex(hostPort, ":")
	if i < 0 {
		return fmt.Errorf("address %q is missing a port", hostPort)
	}
	if _, err := strconv.Atoi(hostPort[i+1:]); err != nil {
		return fmt.Errorf("address %q has an invalid port", hostPort)
	}
	return nil
}
