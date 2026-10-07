package mavaddr

import (
	"reflect"
	"testing"

	"github.com/bluenviron/gomavlib/v3"
)

func TestParse(t *testing.T) {
	for _, tc := range []struct {
		addr string
		want gomavlib.EndpointConf
	}{
		{"/dev/Pixhawk", gomavlib.EndpointSerial{Device: "/dev/Pixhawk", Baud: 57600}},
		{"/dev/ttyACM0:115200", gomavlib.EndpointSerial{Device: "/dev/ttyACM0", Baud: 115200}},
		{"/dev/ttyACM0,921600", gomavlib.EndpointSerial{Device: "/dev/ttyACM0", Baud: 921600}},
		{"tcp:127.0.0.1:5760", gomavlib.EndpointTCPClient{Address: "127.0.0.1:5760"}},
		{"tcpin:0.0.0.0:5760", gomavlib.EndpointTCPServer{Address: "0.0.0.0:5760"}},
		{"udp:0.0.0.0:14550", gomavlib.EndpointUDPServer{Address: "0.0.0.0:14550"}},
		{"udpin:0.0.0.0:14550", gomavlib.EndpointUDPServer{Address: "0.0.0.0:14550"}},
		{"udpout:192.168.2.1:14550", gomavlib.EndpointUDPClient{Address: "192.168.2.1:14550"}},
	} {
		got, err := Parse(tc.addr, 57600)
		if err != nil {
			t.Errorf("Parse(%q): %v", tc.addr, err)
			continue
		}
		if !reflect.DeepEqual(got, tc.want) {
			t.Errorf("Parse(%q) = %#v, want %#v", tc.addr, got, tc.want)
		}
	}
}

func TestParseErrors(t *testing.T) {
	for _, addr := range []string{"", "tcp:127.0.0.1", "udp:host:port"} {
		if _, err := Parse(addr, 57600); err == nil {
			t.Errorf("Parse(%q) succeeded", addr)
		}
	}
}
