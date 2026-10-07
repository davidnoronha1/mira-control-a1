package main

import (
	"context"
	"errors"
	"strings"
	"sync"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/message"
	"github.com/tiiuae/rclgo/pkg/rclgo"

	custom_msgs "github.com/davidnoronha1/mira-control-a1/msgs/custom_msgs/msg"
	sensor_msgs "github.com/davidnoronha1/mira-control-a1/msgs/sensor_msgs/msg"
	std_srvs "github.com/davidnoronha1/mira-control-a1/msgs/std_srvs/srv"
)

const (
	heartbeatLostAfter = 5 * time.Second
	ackTimeout         = 1500 * time.Millisecond
	// After the Pixhawk rejects an arm/disarm we retry on a later command
	// message, but no more often than this.
	armRetryCooldown = 2 * time.Second
	neutralPWM       = 1500
)

// ArduSub flight modes, the same table pymavlink's mode_mapping() returns
// for a submarine.
var subModes = map[string]ardupilotmega.SUB_MODE{
	"STABILIZE": ardupilotmega.SUB_MODE_STABILIZE,
	"ACRO":      ardupilotmega.SUB_MODE_ACRO,
	"ALT_HOLD":  ardupilotmega.SUB_MODE_ALT_HOLD,
	"AUTO":      ardupilotmega.SUB_MODE_AUTO,
	"GUIDED":    ardupilotmega.SUB_MODE_GUIDED,
	"CIRCLE":    ardupilotmega.SUB_MODE_CIRCLE,
	"SURFACE":   ardupilotmega.SUB_MODE_SURFACE,
	"POSHOLD":   ardupilotmega.SUB_MODE_POSHOLD,
	"MANUAL":    ardupilotmega.SUB_MODE_MANUAL,
}

// Telemetry streams requested from the Pixhawk (alt_master.py's set, plus
// SCALED_IMU2 which feeds the IMU fields and /master/imu_ned).
var telemetryStreams = []struct {
	name string
	msg  message.Message
}{
	{"HEARTBEAT", &ardupilotmega.MessageHeartbeat{}},
	{"SYS_STATUS", &ardupilotmega.MessageSysStatus{}},
	{"ATTITUDE_QUATERNION", &ardupilotmega.MessageAttitudeQuaternion{}},
	{"SCALED_PRESSURE2", &ardupilotmega.MessageScaledPressure2{}},
	{"VFR_HUD", &ardupilotmega.MessageVfrHud{}},
	{"SERVO_OUTPUT_RAW", &ardupilotmega.MessageServoOutputRaw{}},
	{"AHRS2", &ardupilotmega.MessageAhrs2{}},
	{"SCALED_IMU2", &ardupilotmega.MessageScaledImu2{}},
}

type Config struct {
	PixhawkAddress  string
	Baud            int
	InitialMode     string
	SourceSystem    int
	SourceComponent int
	RCRateHz        float64
	StreamRateHz    float64
	TelemetryRateHz float64
}

// actionKind is a MAVLink command the worker goroutine sends to the Pixhawk.
type actionKind int

const (
	actionArm actionKind = iota
	actionDisarm
	actionSetMode
)

type action struct {
	kind actionKind
	mode string
}

// Master is the Go port of mira2_control_master's alt_master.py.
type Master struct {
	cfg  Config
	log  *rclgo.Logger
	node *rclgo.Node
	mav  *gomavlib.Node
	link *vehicleLink

	telemetryPub *custom_msgs.TelemetryPublisher
	depthPub     *custom_msgs.DepthPublisher
	headingPub   *custom_msgs.HeadingPublisher
	imuPub       *sensor_msgs.ImuPublisher

	// Arm/disarm/mode changes are sent from one worker goroutine, in the order
	// they were requested, so ROS callbacks never block on the Pixhawk.
	actions chan action

	mu              sync.Mutex
	armState        bool // what we last commanded, reported in telemetry like the Python node
	emergencyLocked bool
	mode            string
	channels        [8]uint16
	lastArmFailure  time.Time
	warnedAt        map[string]time.Time

	telem telemetryState
}

// telemetryState holds the latest copy of every MAVLink message telemetry is
// built from.
type telemetryState struct {
	sysStatus *ardupilotmega.MessageSysStatus
	attitude  *ardupilotmega.MessageAttitudeQuaternion
	vfrHud    *ardupilotmega.MessageVfrHud
	pressure  *ardupilotmega.MessageScaledPressure2
	servo     *ardupilotmega.MessageServoOutputRaw
	ahrs2     *ardupilotmega.MessageAhrs2
	imu       *ardupilotmega.MessageScaledImu2
}

func NewMaster(cfg Config, node *rclgo.Node, mav *gomavlib.Node) (*Master, error) {
	m := &Master{
		cfg:      cfg,
		log:      node.Logger(),
		node:     node,
		mav:      mav,
		link:     newVehicleLink(mav),
		actions:  make(chan action, 16),
		mode:     cfg.InitialMode,
		warnedAt: map[string]time.Time{},
	}
	for i := range m.channels {
		m.channels[i] = neutralPWM
	}

	var err error
	if m.telemetryPub, err = custom_msgs.NewTelemetryPublisher(node, "/master/telemetry", nil); err != nil {
		return nil, err
	}
	if m.depthPub, err = custom_msgs.NewDepthPublisher(node, "/master/depth", nil); err != nil {
		return nil, err
	}
	if m.headingPub, err = custom_msgs.NewHeadingPublisher(node, "/master/heading", nil); err != nil {
		return nil, err
	}
	if m.imuPub, err = sensor_msgs.NewImuPublisher(node, "/master/imu_ned", nil); err != nil {
		return nil, err
	}

	onCommand := func(msg *custom_msgs.Commands, _ *rclgo.MessageInfo, err error) {
		if err != nil {
			m.log.Errorf("Failed to take Commands message: %v", err)
			return
		}
		m.onCommand(msg)
	}
	// Every publisher in the workspace (mira2_rov, behaviour trees, path
	// planning) writes /master/commands. /rov/commands is also accepted, as
	// mira2_control_master/master.py did; the last message received wins.
	if _, err = custom_msgs.NewCommandsSubscription(node, "/master/commands", nil, onCommand); err != nil {
		return nil, err
	}
	if _, err = custom_msgs.NewCommandsSubscription(node, "/rov/commands", nil, onCommand); err != nil {
		return nil, err
	}
	if _, err = std_srvs.NewTriggerService(node, "/toggle_emergency", nil, m.onToggleEmergency); err != nil {
		return nil, err
	}
	return m, nil
}

// Run blocks until ctx is cancelled.
func (m *Master) Run(ctx context.Context) {
	var wg sync.WaitGroup
	for _, f := range []func(context.Context){
		m.readMavlink,
		m.actionWorker,
		m.actuateLoop,
		m.telemetryLoop,
	} {
		wg.Add(1)
		go func(f func(context.Context)) {
			defer wg.Done()
			f(ctx)
		}(f)
	}
	wg.Wait()
}

// ---------------------------------------------------------------------------
// ROS callbacks
// ---------------------------------------------------------------------------

// onCommand mirrors alt_master.py's rov_callback.
func (m *Master) onCommand(msg *custom_msgs.Commands) {
	m.mu.Lock()
	defer m.mu.Unlock()

	if msg.Arm && !m.armState {
		switch {
		case m.emergencyLocked:
			m.warnThrottled("arm-locked", "Arm blocked: emergency lock is engaged")
		case time.Since(m.lastArmFailure) < armRetryCooldown:
			// The Pixhawk just refused; don't hammer it at the command rate.
		default:
			m.armState = true
			m.enqueue(action{kind: actionArm})
		}
	} else if !msg.Arm && m.armState {
		m.armState = false
		m.enqueue(action{kind: actionDisarm})
	}

	m.channels = [8]uint16{
		pwm(msg.Pitch),
		pwm(msg.Roll),
		pwm(msg.Thrust),
		pwm(msg.Yaw),
		pwm(msg.Forward),
		pwm(msg.Lateral),
		pwm(msg.Servo1),
		pwm(msg.Servo2),
	}

	if m.mode != msg.Mode {
		if !m.armState {
			m.mode = msg.Mode
			m.enqueue(action{kind: actionSetMode, mode: msg.Mode})
		} else {
			m.warnThrottled("mode-armed", "Disarm Pixhawk to change modes.")
		}
	}
}

func (m *Master) onToggleEmergency(_ *rclgo.ServiceInfo, _ *std_srvs.Trigger_Request, sender std_srvs.TriggerServiceResponseSender) {
	resp := std_srvs.NewTrigger_Response()
	resp.Success = true

	m.mu.Lock()
	m.emergencyLocked = !m.emergencyLocked
	if m.emergencyLocked {
		if m.armState {
			m.armState = false
			m.enqueue(action{kind: actionDisarm})
		}
		resp.Message = "Emergency lock engaged, disarming"
		m.log.Warn(resp.Message)
	} else {
		resp.Message = "Emergency lock cleared"
		m.log.Info(resp.Message)
	}
	m.mu.Unlock()

	if err := sender.SendResponse(resp); err != nil {
		m.log.Errorf("Failed to answer /toggle_emergency: %v", err)
	}
}

// enqueue must be called with m.mu held.
func (m *Master) enqueue(a action) {
	select {
	case m.actions <- a:
	default:
		m.log.Error("Command queue full, dropping arm/mode request")
	}
}

// warnThrottled logs at most once a second per key. Must be called with m.mu
// held.
func (m *Master) warnThrottled(key, msg string) {
	if time.Since(m.warnedAt[key]) < time.Second {
		return
	}
	m.warnedAt[key] = time.Now()
	m.log.Warn(msg)
}

func pwm(v int16) uint16 {
	if v < 0 {
		return 0
	}
	return uint16(v)
}

// ---------------------------------------------------------------------------
// Pixhawk commands
// ---------------------------------------------------------------------------

func (m *Master) actionWorker(ctx context.Context) {
	for {
		select {
		case <-ctx.Done():
			return
		case a := <-m.actions:
			switch a.kind {
			case actionArm:
				m.sendArm(ctx, true)
			case actionDisarm:
				m.sendArm(ctx, false)
			case actionSetMode:
				m.sendMode(ctx, a.mode)
			}
		}
	}
}

func (m *Master) sendArm(ctx context.Context, arm bool) {
	what := map[bool]string{true: "Arm", false: "Disarm"}[arm]

	// Like the Python node, make sure the Pixhawk is alive before arming.
	if err := m.link.waitHeartbeat(ctx, 3*time.Second); err != nil {
		m.log.Warnf("%s: %v, sending anyway", what, err)
	}
	param1 := float32(0)
	if arm {
		param1 = 1
	}
	result, err := m.link.commandLong(ctx, common.MAV_CMD_COMPONENT_ARM_DISARM, ackTimeout, param1)
	switch {
	case errors.Is(err, context.Canceled):
		return
	case err != nil:
		m.log.Warnf("%s command sent to Pixhawk, but %v", what, err)
	case result == common.MAV_RESULT_ACCEPTED || result == common.MAV_RESULT_IN_PROGRESS:
		m.log.Infof("%s command sent to Pixhawk (accepted)", what)
	default:
		m.log.Errorf("%s command rejected by Pixhawk: %s (check the Pixhawk's STATUSTEXT messages above)", what, result)
		m.mu.Lock()
		// Undo the state change so the next command message retries, unless
		// a newer command has already changed it.
		if m.armState == arm {
			m.armState = !arm
			m.lastArmFailure = time.Now()
		}
		m.mu.Unlock()
	}
}

func (m *Master) sendMode(ctx context.Context, mode string) {
	id, ok := subModes[strings.ToUpper(mode)]
	if !ok {
		// alt_master.py exits here. Keeping the node (and the thrusters'
		// neutral override) alive is safer; the next valid mode is applied.
		m.log.Errorf("Unknown mode: %q. Try: STABILIZE, ACRO, ALT_HOLD, AUTO, GUIDED, CIRCLE, SURFACE, POSHOLD, MANUAL", mode)
		return
	}
	result, err := m.link.setMode(ctx, uint32(id), ackTimeout)
	switch {
	case errors.Is(err, context.Canceled):
		return
	case err != nil:
		m.log.Warnf("Mode change to %s sent, but %v", mode, err)
	case result == common.MAV_RESULT_ACCEPTED:
		m.log.Infof("Mode changed to: %s", mode)
	default:
		m.log.Errorf("Mode change to %s rejected by Pixhawk: %s", mode, result)
	}
}

// requestStreams mirrors request_message_interval for every telemetry stream.
func (m *Master) requestStreams(ctx context.Context) {
	interval := float32(1e6 / m.cfg.StreamRateHz)
	for _, stream := range telemetryStreams {
		name, msg := stream.name, stream.msg
		var (
			result common.MAV_RESULT
			err    error
		)
		for attempt := 0; attempt < 3; attempt++ {
			result, err = m.link.commandLong(ctx, common.MAV_CMD_SET_MESSAGE_INTERVAL, ackTimeout, float32(msg.GetID()), interval)
			if ctx.Err() != nil {
				return
			}
			if err == nil {
				break
			}
		}
		if err == nil && result == common.MAV_RESULT_ACCEPTED {
			m.log.Infof("Command Accepted for %s", name)
		} else if err != nil {
			m.log.Errorf("Command Failed for %s: %v", name, err)
		} else {
			m.log.Errorf("Command Failed for %s: %s", name, result)
		}
	}
}

// actuateLoop replaces the Python main loop's actuate() call. It sends the
// latest PWM values at a fixed rate: often enough to keep ArduSub's RC
// override from timing out, slowly enough to leave room on the serial link
// for arm and mode commands.
func (m *Master) actuateLoop(ctx context.Context) {
	t := time.NewTicker(time.Duration(float64(time.Second) / m.cfg.RCRateHz))
	defer t.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-t.C:
		}
		if _, _, ok := m.link.target(); !ok {
			continue
		}
		m.mu.Lock()
		ch := m.channels
		m.mu.Unlock()
		if err := m.link.rcOverride(ch); err != nil {
			m.log.Errorf("Failed to send RC override: %v", err)
		}
	}
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

func (m *Master) readMavlink(ctx context.Context) {
	waitingLogged := false
	for {
		select {
		case <-ctx.Done():
			return
		case evt, ok := <-m.mav.Events():
			if !ok {
				return
			}
			switch e := evt.(type) {
			case *gomavlib.EventChannelOpen:
				m.log.Infof("MAVLink connection established (%v)", e.Channel)
				if !waitingLogged {
					m.log.Info("Waiting for heartbeat from Pixhawk...")
					waitingLogged = true
				}
			case *gomavlib.EventChannelClose:
				m.log.Warnf("MAVLink connection closed (%v)", e.Channel)
			case *gomavlib.EventParseError:
				m.log.Debugf("MAVLink parse error: %v", e.Error)
			case *gomavlib.EventFrame:
				m.handleFrame(ctx, e)
			}
		}
	}
}

func (m *Master) handleFrame(ctx context.Context, frm *gomavlib.EventFrame) {
	switch msg := frm.Message().(type) {
	case *ardupilotmega.MessageHeartbeat:
		ev, ok := m.link.handleHeartbeat(frm, msg)
		if !ok {
			return
		}
		if ev.first {
			m.log.Infof("Heartbeat from Pixhawk (system %d component %d)", ev.system, ev.component)
			go m.requestStreams(ctx)
		}
		if ev.armedChanged {
			m.log.Infof("Pixhawk reports it is now %s", map[bool]string{true: "ARMED", false: "DISARMED"}[ev.armed])
		}
		return
	case *ardupilotmega.MessageCommandAck:
		m.link.handleAck(msg)
		return
	case *ardupilotmega.MessageStatustext:
		m.log.Infof("[PIXHAWK] %s", msg.Text)
		return
	}

	// Only telemetry from the autopilot we command.
	if sys, comp, ok := m.link.target(); !ok || frm.SystemID() != sys || frm.ComponentID() != comp {
		return
	}

	m.mu.Lock()
	defer m.mu.Unlock()
	switch msg := frm.Message().(type) {
	case *ardupilotmega.MessageSysStatus:
		m.telem.sysStatus = msg
	case *ardupilotmega.MessageAttitudeQuaternion:
		m.telem.attitude = msg
	case *ardupilotmega.MessageVfrHud:
		m.telem.vfrHud = msg
	case *ardupilotmega.MessageScaledPressure2:
		m.telem.pressure = msg
	case *ardupilotmega.MessageServoOutputRaw:
		m.telem.servo = msg
	case *ardupilotmega.MessageAhrs2:
		m.telem.ahrs2 = msg
	case *ardupilotmega.MessageScaledImu2:
		m.telem.imu = msg
	}
}

func (m *Master) telemetryLoop(ctx context.Context) {
	t := time.NewTicker(time.Duration(float64(time.Second) / m.cfg.TelemetryRateHz))
	defer t.Stop()
	start := time.Now()
	var lastIMU *ardupilotmega.MessageScaledImu2
	var lastPressure *ardupilotmega.MessageScaledPressure2
	var lastVfr *ardupilotmega.MessageVfrHud
	warned := map[string]bool{}

	for {
		select {
		case <-ctx.Done():
			return
		case <-t.C:
		}

		now := time.Now()
		stamp := float64(now.UnixNano()) / 1e9

		m.mu.Lock()
		ts := m.telem
		armState := m.armState
		m.mu.Unlock()

		// alt_master.py does not publish until it has seen every message once.
		if now.Sub(start) > 10*time.Second {
			for _, name := range ts.missing() {
				if !warned[name] {
					warned[name] = true
					m.log.Warnf("Still waiting for %s from the Pixhawk", name)
					if name == "SCALED_PRESSURE2" {
						m.log.Warn("!!! Is depth sensor connected ? Connect it to I2C port on the pixhawk !!!")
					}
				}
			}
		}

		if !ts.ready() {
			continue
		}

		telem := buildTelemetry(&ts, armState, stamp)
		if err := m.telemetryPub.Publish(telem); err != nil {
			m.log.Errorf("Failed to publish telemetry: %v", err)
		}

		if ts.vfrHud != lastVfr {
			lastVfr = ts.vfrHud
			heading := custom_msgs.NewHeading()
			heading.Timestamp = stamp
			heading.Heading = int32(ts.vfrHud.Heading)
			_ = m.headingPub.Publish(heading)
		}
		if ts.pressure != nil && ts.pressure != lastPressure {
			lastPressure = ts.pressure
			depth := custom_msgs.NewDepth()
			depth.Timestamp = stamp
			depth.ExternalPressure = ts.pressure.PressAbs
			_ = m.depthPub.Publish(depth)
		}
		if ts.imu != nil && ts.imu != lastIMU {
			lastIMU = ts.imu
			_ = m.imuPub.Publish(buildIMU(ts.imu, now))
		}
	}
}

func (t *telemetryState) missing() []string {
	var out []string
	for name, have := range map[string]bool{
		"SYS_STATUS":          t.sysStatus != nil,
		"ATTITUDE_QUATERNION": t.attitude != nil,
		"VFR_HUD":             t.vfrHud != nil,
		"SCALED_PRESSURE2":    t.pressure != nil,
		"SERVO_OUTPUT_RAW":    t.servo != nil,
		"AHRS2":               t.ahrs2 != nil,
	} {
		if !have {
			out = append(out, name)
		}
	}
	return out
}

// ready reports whether enough has been received to publish telemetry. The
// depth sensor is optional: without it external_pressure is -1, as in
// alt_master.py.
func (t *telemetryState) ready() bool {
	return t.sysStatus != nil && t.attitude != nil && t.vfrHud != nil && t.servo != nil && t.ahrs2 != nil
}

// buildTelemetry mirrors alt_master.py's telem_publish_func.
func buildTelemetry(ts *telemetryState, armState bool, stamp float64) *custom_msgs.Telemetry {
	t := custom_msgs.NewTelemetry()
	t.Arm = armState
	t.BatteryVoltage = float32(ts.sysStatus.VoltageBattery) / 1000
	t.Timestamp = stamp

	t.InternalPressure = ts.vfrHud.Alt
	t.ExternalPressure = -1
	if ts.pressure != nil {
		t.ExternalPressure = ts.pressure.PressAbs
	}
	t.Heading = int32(ts.vfrHud.Heading)

	if ts.imu != nil {
		t.ImuXacc = int32(ts.imu.Xacc)
		t.ImuYacc = int32(ts.imu.Yacc)
		t.ImuZacc = int32(ts.imu.Zacc)
		t.ImuGyroX = int32(ts.imu.Xgyro)
		t.ImuGyroY = int32(ts.imu.Ygyro)
		t.ImuGyroZ = int32(ts.imu.Zgyro)
		t.ImuGyroCompassX = int32(ts.imu.Xmag)
		t.ImuGyroCompassY = int32(ts.imu.Ymag)
		t.ImuGyroCompassZ = int32(ts.imu.Zmag)
	}

	t.Q1 = ts.attitude.Q1
	t.Q2 = ts.attitude.Q2
	t.Q3 = ts.attitude.Q3
	t.Q4 = ts.attitude.Q4
	t.Rollspeed = ts.attitude.Rollspeed
	t.Pitchspeed = ts.attitude.Pitchspeed
	t.Yawspeed = ts.attitude.Yawspeed

	t.Roll = ts.ahrs2.Roll
	t.Pitch = ts.ahrs2.Pitch
	t.Yaw = ts.ahrs2.Yaw

	s := ts.servo
	t.ThrusterPwms = [8]float32{
		float32(s.Servo1Raw), float32(s.Servo2Raw), float32(s.Servo3Raw), float32(s.Servo4Raw),
		float32(s.Servo5Raw), float32(s.Servo6Raw), float32(s.Servo7Raw), float32(s.Servo8Raw),
	}
	return t
}

// buildIMU mirrors alt_master.py's publish_imu.
func buildIMU(imu *ardupilotmega.MessageScaledImu2, now time.Time) *sensor_msgs.Imu {
	const gToMs2 = 9.80665
	msg := sensor_msgs.NewImu()
	msg.Header.Stamp.Sec = int32(now.Unix())
	msg.Header.Stamp.Nanosec = uint32(now.Nanosecond())
	// mG -> m/s^2
	msg.LinearAcceleration.X = float64(imu.Xacc) * gToMs2 / 1000
	msg.LinearAcceleration.Y = float64(imu.Yacc) * gToMs2 / 1000
	msg.LinearAcceleration.Z = float64(imu.Zacc) * gToMs2 / 1000
	// mrad/s -> rad/s
	msg.AngularVelocity.X = float64(imu.Xgyro) / 1000
	msg.AngularVelocity.Y = float64(imu.Ygyro) / 1000
	msg.AngularVelocity.Z = float64(imu.Zgyro) / 1000
	// No orientation estimate in this message (REP 145).
	msg.OrientationCovariance[0] = -1
	return msg
}

// shutdown disarms the vehicle if we armed it, like the Python SIGINT handler.
func (m *Master) shutdown() {
	m.mu.Lock()
	armed := m.armState
	m.armState = false
	m.mu.Unlock()
	if !armed {
		return
	}
	m.log.Info("Signal received, disarming and shutting down...")
	result, err := m.link.commandLong(context.Background(), common.MAV_CMD_COMPONENT_ARM_DISARM, ackTimeout, 0)
	switch {
	case err != nil:
		m.log.Warnf("Disarm command sent to Pixhawk, but %v", err)
	case result != common.MAV_RESULT_ACCEPTED:
		m.log.Errorf("Disarm command rejected by Pixhawk: %s", result)
	default:
		m.log.Info("Disarm command sent to Pixhawk (accepted)")
	}
}
