// Command master is a drop-in Go replacement for mira2_control_master's
// alt_master.py: it bridges /master/commands to a Pixhawk running ArduSub and
// publishes the Pixhawk's telemetry on /master/telemetry.
package main

//go:generate ./scripts/generate.sh

import (
	"context"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"syscall"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/tiiuae/rclgo/pkg/rclgo"

	"github.com/davidnoronha1/mira-control-a1/internal/mavaddr"
	"github.com/davidnoronha1/mira-control-a1/internal/rosparam"
)

const nodeName = "pymav_master"

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, "master:", err)
		os.Exit(1)
	}
}

func run() error {
	rclArgs, rest, err := rclgo.ParseArgs(os.Args[1:])
	if err != nil {
		return fmt.Errorf("parsing ROS arguments: %w", err)
	}
	cfg, err := loadConfig(os.Args[1:], rest)
	if err != nil {
		return err
	}

	if err := rclgo.Init(rclArgs); err != nil {
		return fmt.Errorf("initialising rclgo: %w", err)
	}
	defer rclgo.Uninit()

	node, err := rclgo.NewNode(nodeName, "")
	if err != nil {
		return fmt.Errorf("creating node: %w", err)
	}
	defer node.Close()
	log := node.Logger()
	log.Infof("pixhawk_address='%s', initial_mode='%s', baud=%d, source_system=%d",
		cfg.PixhawkAddress, cfg.InitialMode, cfg.Baud, cfg.SourceSystem)

	endpoint, err := mavaddr.Parse(cfg.PixhawkAddress, cfg.Baud)
	if err != nil {
		return err
	}
	mav, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints:  []gomavlib.EndpointConf{endpoint},
		Dialect:    ardupilotmega.Dialect,
		OutVersion: gomavlib.V2,
		// ArduPilot only accepts RC_CHANNELS_OVERRIDE from the system id in
		// its SYSID_MYGCS parameter (255 by default), which is also
		// pymavlink's default source system.
		OutSystemID:    byte(cfg.SourceSystem),
		OutComponentID: byte(cfg.SourceComponent),
		// pymavlink does not send heartbeats either.
		HeartbeatDisable: true,
	})
	if err != nil {
		return fmt.Errorf("connecting to the Pixhawk at %s: %w", cfg.PixhawkAddress, err)
	}
	defer mav.Close()

	master, err := NewMaster(cfg, node, mav)
	if err != nil {
		return fmt.Errorf("creating ROS interfaces: %w", err)
	}
	log.Info("PixhawkMaster node initialized")

	sigCtx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()
	// Separate from sigCtx so the MAVLink reader keeps running (and sees the
	// ack) while we disarm on shutdown.
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	spinErr := make(chan error, 1)
	go func() { spinErr <- node.Spin(ctx) }()

	done := make(chan struct{})
	go func() {
		master.Run(ctx)
		close(done)
	}()

	select {
	case <-sigCtx.Done():
	case err = <-spinErr:
		log.Errorf("ROS spin stopped: %v", err)
	}
	master.shutdown()
	cancel()
	<-done
	return nil
}

// loadConfig reads ROS parameters (`--ros-args -p name:=value` or a launch
// file's params file) and, for compatibility with master.py, the -p/--port
// and -m/--mode flags.
func loadConfig(allArgs, nonROSArgs []string) (Config, error) {
	params, err := rosparam.Parse(allArgs, nodeName)
	if err != nil {
		return Config{}, err
	}

	cfg := Config{
		PixhawkAddress: params.String("pixhawk_address", "/dev/Pixhawk"),
		InitialMode:    params.String("initial_mode", "STABILIZE"),
	}
	ints := []struct {
		dst  *int
		name string
		def  int
	}{
		{&cfg.Baud, "baud", 57600},
		{&cfg.SourceSystem, "source_system", 255},
		{&cfg.SourceComponent, "source_component", 0},
	}
	for _, p := range ints {
		if *p.dst, err = params.Int(p.name, p.def); err != nil {
			return Config{}, err
		}
	}
	floats := []struct {
		dst  *float64
		name string
		def  float64
	}{
		{&cfg.RCRateHz, "rc_rate_hz", 25},
		{&cfg.StreamRateHz, "stream_rate_hz", 100},
		{&cfg.TelemetryRateHz, "telemetry_rate_hz", 50},
	}
	for _, p := range floats {
		if *p.dst, err = params.Float(p.name, p.def); err != nil {
			return Config{}, err
		}
		if *p.dst <= 0 {
			return Config{}, fmt.Errorf("parameter %s must be positive", p.name)
		}
	}
	if cfg.SourceSystem < 1 || cfg.SourceSystem > 255 || cfg.SourceComponent < 0 || cfg.SourceComponent > 255 {
		return Config{}, fmt.Errorf("source_system must be 1-255 and source_component 0-255")
	}

	fs := flag.NewFlagSet("master", flag.ContinueOnError)
	port := fs.String("port", "", "Pixhawk address (same as the pixhawk_address parameter)")
	fs.StringVar(port, "p", "", "shorthand for -port")
	mode := fs.String("mode", "", "initial mode (same as the initial_mode parameter)")
	fs.StringVar(mode, "m", "", "shorthand for -mode")
	if err := fs.Parse(nonROSArgs); err != nil {
		return Config{}, err
	}
	if *port != "" {
		cfg.PixhawkAddress = *port
	}
	if *mode != "" {
		cfg.InitialMode = *mode
	}
	return cfg, nil
}
