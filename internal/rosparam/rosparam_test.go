package rosparam

import (
	"os"
	"path/filepath"
	"testing"
)

func TestParseCommandLine(t *testing.T) {
	args := []string{
		"-m", "MANUAL",
		"--ros-args", "-p", "pixhawk_address:=tcp:127.0.0.1:5760", "--param", "baud:=115200",
		"-p", "initial_mode:='ALT_HOLD'", "--",
		"-p", "ignored:=1",
	}
	p, err := Parse(args, "pymav_master")
	if err != nil {
		t.Fatal(err)
	}
	if got := p.String("pixhawk_address", ""); got != "tcp:127.0.0.1:5760" {
		t.Errorf("pixhawk_address = %q", got)
	}
	if got, _ := p.Int("baud", 0); got != 115200 {
		t.Errorf("baud = %d", got)
	}
	if got := p.String("initial_mode", ""); got != "ALT_HOLD" {
		t.Errorf("initial_mode = %q", got)
	}
	if _, ok := p["ignored"]; ok {
		t.Error("parameter outside --ros-args was parsed")
	}
	if got := p.String("missing", "def"); got != "def" {
		t.Errorf("default = %q", got)
	}
}

func TestParseParamsFile(t *testing.T) {
	// The layout ros2 launch writes for <param> tags.
	file := filepath.Join(t.TempDir(), "params.yaml")
	err := os.WriteFile(file, []byte(`
/**:
  ros__parameters:
    pixhawk_address: /dev/ttyACM0
    baud: 57600
/pymavlink_commands:
  ros__parameters:
    baud: 115200
/other_node:
  ros__parameters:
    initial_mode: MANUAL
`), 0o600)
	if err != nil {
		t.Fatal(err)
	}

	args := []string{"--ros-args", "-r", "__node:=pymavlink_commands", "--params-file", file}
	p, err := Parse(args, "pymav_master")
	if err != nil {
		t.Fatal(err)
	}
	if got := p.String("pixhawk_address", ""); got != "/dev/ttyACM0" {
		t.Errorf("pixhawk_address = %q", got)
	}
	if got, _ := p.Int("baud", 0); got != 115200 {
		t.Errorf("node specific baud not applied, got %d", got)
	}
	if _, ok := p["initial_mode"]; ok {
		t.Error("picked up another node's parameter")
	}
}

func TestParseErrors(t *testing.T) {
	for _, args := range [][]string{
		{"--ros-args", "-p"},
		{"--ros-args", "-p", "novalue"},
		{"--ros-args", "--params-file", "/does/not/exist.yaml"},
	} {
		if _, err := Parse(args, "n"); err == nil {
			t.Errorf("Parse(%q) succeeded", args)
		}
	}
}
