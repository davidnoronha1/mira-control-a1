package main

//go:generate ./scripts/generate.sh

import (
	"bufio"
	"context"
	"errors"
	"flag"
	"os"
	"os/signal"
	"reflect"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/message"
	"github.com/charmbracelet/log"
	"github.com/tiiuae/rclgo/pkg/rclgo"

	"github.com/davidnoronha1/mira-control-a1/internal/mavaddr"
	"github.com/davidnoronha1/mira-control-a1/internal/rosparam"
	custom_msgs "github.com/davidnoronha1/mira-control-a1/msgs/custom_msgs/msg"
	sensor_msgs "github.com/davidnoronha1/mira-control-a1/msgs/sensor_msgs/msg"
	std_srvs "github.com/davidnoronha1/mira-control-a1/msgs/std_srvs/srv"
)

const (
	NODE_NAME = "pymav_master"
	// Lost heartbeat after this long => re-request telemetry streams when it returns
	HEARTBEAT_TIMEOUT = 5 * time.Second
	ACK_TIMEOUT       = 1500 * time.Millisecond
	// After the PixHawk refuses an ARM / DISARM, wait this long before retrying
	ARM_RETRY_COOLDOWN = 2 * time.Second
	NEUTRAL_PWM        = 1500
)

type Config struct {
	pixhawk_address   string
	baud              int
	initial_mode      string
	source_system     int
	source_component  int
	rc_rate_hz        float64
	stream_rate_hz    float64
	telemetry_rate_hz float64
}

// Work for the PixHawk command goroutine, every COMMAND_LONG goes through it
// so there is only ever one command waiting on an ack.
//
// ARM / DISARM / emergency disarm go on priority_ch and are always handled
// before anything on command_ch. ARM is on the priority queue as well so that a
// DISARM can never overtake an earlier ARM (which would leave the vehicle armed).
type CommandKind int

const (
	CMD_ARM CommandKind = iota
	CMD_DISARM
	CMD_EMERGENCY_DISARM
	CMD_SET_MODE
	CMD_MESSAGE_INTERVAL
)

func (kind CommandKind) isPriority() bool {
	return kind == CMD_ARM || kind == CMD_DISARM || kind == CMD_EMERGENCY_DISARM
}

// Returned by sendCommandLong when a priority command arrived while a normal
// command was waiting for its ack
var errPreempted = errors.New("preempted by a priority command")

// MAV_CMD_COMPONENT_ARM_DISARM param2 value that makes ArduPilot disarm even
// if it would normally refuse, only used for the emergency kill
const FORCE_DISARM_MAGIC = 21196

type PixhawkCommand struct {
	kind    CommandKind
	mode    string
	msg_id  uint32
	freq_hz float64
	done    chan struct{} // (optional) closed once the command has been handled
}

type Master struct {
	// Connections
	pixhawk *gomavlib.Node
	ros     *rclgo.Node
	config  Config
	// Publishers & Subscribers
	command_sub   *custom_msgs.CommandsSubscription
	thruster_sub  *custom_msgs.CommandsSubscription
	toggle_srv    *std_srvs.TriggerService
	kill_srv      *std_srvs.TriggerService
	clear_srv     *std_srvs.TriggerService
	telemetry_pub *custom_msgs.TelemetryPublisher
	depth_pub     *custom_msgs.DepthPublisher
	heading_pub   *custom_msgs.HeadingPublisher
	imu_pub       *sensor_msgs.ImuPublisher
	// Channels
	heartbeat_ch chan struct{}
	ack_ch       chan common.MessageCommandAck
	telemetry_ch chan message.Message
	command_ch   chan PixhawkCommand
	priority_ch  chan PixhawkCommand
	// Signalled (non blocking) whenever something is put on priority_ch, so a
	// normal command waiting on its ack can give way immediately
	priority_signal chan struct{}
	// State, shared between goroutines so guarded by mu
	mu               sync.Mutex
	target_system    uint8
	target_component uint8
	have_target      bool
	last_heartbeat   time.Time
	pixhawk_armed    bool // what the PixHawk reports in its heartbeat
	channel_arr      [8]int
	armed            bool // what we last commanded, reported in telemetry
	mode             string
	emergency_locked bool
	last_arm_failure time.Time
	last_warn        map[string]time.Time
	// Misc
	log_misc bool
}

func NewApplication(config Config, _pixhawk *gomavlib.Node, _ros *rclgo.Node) *Master {
	log.Info("[*] Initializing new application")
	app := &Master{
		pixhawk: _pixhawk,
		ros:     _ros,
		config:  config,

		// All buffered: the MAVLink listener must never block on a consumer
		heartbeat_ch: make(chan struct{}, 1),
		ack_ch:       make(chan common.MessageCommandAck, 8),
		telemetry_ch: make(chan message.Message, 64),
		command_ch:   make(chan PixhawkCommand, 32),
		priority_ch:  make(chan PixhawkCommand, 16),

		priority_signal: make(chan struct{}, 1),

		channel_arr: [8]int{NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM},
		mode:        config.initial_mode,
		last_warn:   map[string]time.Time{},

		log_misc: false,
	}
	n := app.ros
	var err error

	// -- INITIALIZE SUBSCRIBERS AND PUBLISHERS --
	// Every publisher in the workspace (joystick, behaviour trees, path planning)
	// writes /master/commands. /rov/commands is accepted too, last message wins.
	app.command_sub, err = custom_msgs.NewCommandsSubscription(n, "/master/commands", nil, app.onCommandMessage)
	if err != nil {
		log.Fatal("[ROS] Failed to register /master/commands subscriber", "err", err)
	}

	app.thruster_sub, err = custom_msgs.NewCommandsSubscription(n, "/rov/commands", nil, app.onROVMessage)
	if err != nil {
		log.Fatal("[ROS] Failed to register /rov/commands subscriber", "err", err)
	}

	// Used by mira2_control_master's killswitch node
	app.toggle_srv, err = std_srvs.NewTriggerService(n, "/toggle_emergency", nil, app.onToggleEmergency)
	if err != nil {
		log.Fatal("[ROS] Failed to register /toggle_emergency service", "err", err)
	}

	app.kill_srv, err = std_srvs.NewTriggerService(n, "/emergency_kill", nil, app.onEmergencyKill)
	if err != nil {
		log.Fatal("[ROS] Failed to register /emergency_kill service", "err", err)
	}

	app.clear_srv, err = std_srvs.NewTriggerService(n, "/clear_emergency", nil, app.onClearEmergency)
	if err != nil {
		log.Fatal("[ROS] Failed to register /clear_emergency service", "err", err)
	}

	app.telemetry_pub, err = custom_msgs.NewTelemetryPublisher(n, "/master/telemetry", nil)
	if err != nil {
		log.Fatal("[ROS] Failed to register /master/telemetry publisher", "err", err)
	}

	app.depth_pub, err = custom_msgs.NewDepthPublisher(n, "/master/depth", nil)
	if err != nil {
		log.Fatal("[ROS] Failed to register /master/depth publisher", "err", err)
	}

	app.heading_pub, err = custom_msgs.NewHeadingPublisher(n, "/master/heading", nil)
	if err != nil {
		log.Fatal("[ROS] Failed to register /master/heading publisher", "err", err)
	}

	app.imu_pub, err = sensor_msgs.NewImuPublisher(n, "/master/imu_ned", nil)
	if err != nil {
		log.Fatal("[ROS] Failed to register /master/imu_ned publisher", "err", err)
	}

	return app
}

// Disarm before shutting down, needs listenForEvents and handleCommands to still be running
func (app *Master) cleanup() {
	log.Info("[*] Cleaning up")
	app.mu.Lock()
	was_armed := app.armed
	app.armed = false
	app.mu.Unlock()

	if !was_armed {
		return
	}

	done := make(chan struct{})
	app.queueCommand(PixhawkCommand{kind: CMD_DISARM, done: done})
	select {
	case <-done:
	case <-time.After(5 * time.Second):
		log.Error("[PIXHAWK] Timed out disarming on shutdown")
	}
}

// -- PIXHAWK COMMANDS (only ever called from handleCommands) --

func (app *Master) handleCommands(ctx context.Context) {
	for {
		// Anything on the priority queue goes first
		select {
		case cmd := <-app.priority_ch:
			app.runCommand(ctx, cmd)
			continue
		default:
		}

		select {
		case <-ctx.Done():
			return
		case cmd := <-app.priority_ch:
			app.runCommand(ctx, cmd)
		case cmd := <-app.command_ch:
			// A normal command gives way as soon as a priority command arrives,
			// then gets sent again
			for app.runCommand(ctx, cmd) {
				app.runPriorityCommands(ctx)
			}
		}
	}
}

func (app *Master) runPriorityCommands(ctx context.Context) {
	for {
		select {
		case cmd := <-app.priority_ch:
			app.runCommand(ctx, cmd)
		default:
			return
		}
	}
}

// Returns true if the command was preempted and has to be run again
func (app *Master) runCommand(ctx context.Context, cmd PixhawkCommand) (preempted bool) {
	switch cmd.kind {
	case CMD_ARM:
		app.ArmOrDisarm(ctx, true)
	case CMD_DISARM:
		app.ArmOrDisarm(ctx, false)
	case CMD_EMERGENCY_DISARM:
		app.EmergencyDisarm(ctx)
	case CMD_SET_MODE:
		preempted = app.SwitchMode(ctx, cmd.mode)
	case CMD_MESSAGE_INTERVAL:
		preempted = app.requestMessageAtInterval(ctx, cmd.msg_id, cmd.freq_hz)
	}
	if !preempted && cmd.done != nil {
		close(cmd.done)
	}
	return preempted
}

// Hand a command to handleCommands without blocking the caller
func (app *Master) queueCommand(cmd PixhawkCommand) {
	queue := app.command_ch
	if cmd.kind.isPriority() {
		queue = app.priority_ch
	}
	select {
	case queue <- cmd:
		if cmd.kind.isPriority() {
			select {
			case app.priority_signal <- struct{}{}:
			default: // already signalled
			}
		}
	default:
		log.Error("[PIXHAWK] Command queue full, dropping command", "kind", cmd.kind)
		if cmd.done != nil {
			close(cmd.done)
		}
	}
}

// Sends a COMMAND_LONG and waits for its COMMAND_ACK. Only one command is in
// flight at a time, so any ack on ack_ch for this command is ours.
//
// A preemptible command stops waiting and returns errPreempted as soon as a
// priority command is queued.
func (app *Master) sendCommandLong(ctx context.Context, preemptible bool, command common.MAV_CMD, params ...float32) (common.MAV_RESULT, error) {
	app.mu.Lock()
	target_system, target_component, have_target := app.target_system, app.target_component, app.have_target
	app.mu.Unlock()
	if !have_target {
		return 0, errors.New("pixhawk not discovered yet")
	}

	var p [7]float32
	copy(p[:], params)

	var preempt <-chan struct{} // nil channel: never fires
	if preemptible {
		// Clear a stale signal (its command may already have been handled),
		// then check the queue itself
		select {
		case <-app.priority_signal:
		default:
		}
		if len(app.priority_ch) > 0 {
			return 0, errPreempted
		}
		preempt = app.priority_signal
	}

	// Throw away stale acks (eg. a late ack for a command that timed out)
drain:
	for {
		select {
		case <-app.ack_ch:
		default:
			break drain
		}
	}

	err := app.pixhawk.WriteMessageAll(&ardupilotmega.MessageCommandLong{
		TargetSystem:    target_system,
		TargetComponent: target_component,
		Command:         command,
		Confirmation:    0,
		Param1:          p[0],
		Param2:          p[1],
		Param3:          p[2],
		Param4:          p[3],
		Param5:          p[4],
		Param6:          p[5],
		Param7:          p[6],
	})
	if err != nil {
		return 0, err
	}

	timeout := time.After(ACK_TIMEOUT)
	for {
		select {
		case ack := <-app.ack_ch:
			if ack.Command == command {
				return ack.Result, nil
			}
			log.Debug("[PIXHAWK] Got ack for another command", "ack", ack)
		case <-preempt:
			return 0, errPreempted
		case <-timeout:
			return 0, errors.New("no ack")
		case <-ctx.Done():
			return 0, ctx.Err()
		}
	}
}

func (app *Master) ArmOrDisarm(ctx context.Context, arm bool) {
	if arm {
		// Same as pymavlink's wait_heartbeat() before arming, but give up waiting
		// as soon as something else lands on the priority queue
		select {
		case <-app.priority_signal: // stale, most likely from queueing this ARM
		default:
		}
		if len(app.priority_ch) == 0 {
			if err := app.waitForHeartbeat(ctx, 3*time.Second, app.priority_signal); err != nil {
				log.Warn("[PIXHAWK] No heartbeat before ARM, sending anyway", "err", err)
			}
		}
		// A DISARM or emergency kill may have come in while we waited
		app.mu.Lock()
		still_wanted := app.armed && !app.emergency_locked
		app.mu.Unlock()
		if !still_wanted {
			log.Warn("[PIXHAWK] ARM cancelled, superseded by a DISARM / emergency kill")
			return
		}
	}

	var arm_v float32 = 0.0
	if arm {
		arm_v = 1.0
	}

	what := "DISARM"
	if arm {
		what = "ARM"
	}

	ack_result, err := app.sendCommandLong(ctx, false, common.MAV_CMD_COMPONENT_ARM_DISARM, arm_v)
	switch {
	case err != nil:
		log.Warn("[PIXHAWK] "+what+" Command Sent, but", "err", err)
	case ack_result == common.MAV_RESULT_ACCEPTED || ack_result == common.MAV_RESULT_IN_PROGRESS:
		log.Warn("[PIXHAWK] "+what+" Command Sent", "result", ack_result)
	default:
		log.Error("[PIXHAWK] "+what+" Command Rejected (see STATUSTEXT for why)", "result", ack_result)
		// Undo the state change so that the next command message retries,
		// unless a newer command has already changed it
		app.mu.Lock()
		if app.armed == arm {
			app.armed = !arm
			app.last_arm_failure = time.Now()
		}
		app.mu.Unlock()
	}
}

// Force disarm straight away (no waiting for a heartbeat), retried until the
// PixHawk acknowledges it
func (app *Master) EmergencyDisarm(ctx context.Context) {
	for attempt := 1; attempt <= 3; attempt++ {
		ack_result, err := app.sendCommandLong(ctx, false, common.MAV_CMD_COMPONENT_ARM_DISARM, 0, FORCE_DISARM_MAGIC)
		if err == nil && ack_result == common.MAV_RESULT_ACCEPTED {
			log.Warn("[PIXHAWK] EMERGENCY DISARM accepted")
			return
		}
		if ctx.Err() != nil {
			return
		}
		if err != nil {
			log.Error("[PIXHAWK] EMERGENCY DISARM not confirmed, retrying", "attempt", attempt, "err", err)
		} else {
			log.Error("[PIXHAWK] EMERGENCY DISARM rejected, retrying", "attempt", attempt, "result", ack_result)
		}
	}
	log.Error("[PIXHAWK] EMERGENCY DISARM never confirmed by the PixHawk!")
}

// Returns true if it was preempted by a priority command and has to be re-run
func (app *Master) SwitchMode(ctx context.Context, mode string) bool {

	var values_SUB_MODE = map[string]ardupilotmega.SUB_MODE{
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

	mode = strings.ToUpper(mode)

	mode_val, ok := values_SUB_MODE[mode]

	if !ok {
		log.Error("Invalid Mode, try STABILIZE, ACRO, ALT_HOLD, AUTO, GUIDED, CIRCLE, SURFACE, POSHOLD or MANUAL", "mode", mode)
		return false
	}

	// MAV_CMD_DO_SET_MODE goes through the same code in ArduPilot as the
	// SET_MODE message but, unlike SET_MODE, it gets acknowledged
	ack_result, err := app.sendCommandLong(ctx, true, common.MAV_CMD_DO_SET_MODE,
		float32(ardupilotmega.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), float32(mode_val))

	if errors.Is(err, errPreempted) {
		return true
	}
	if err != nil {
		log.Warn("[PIXHAWK] No ack for mode change, sending SET_MODE as well", "mode", mode, "err", err)
		app.mu.Lock()
		target_system := app.target_system
		app.mu.Unlock()
		err = app.pixhawk.WriteMessageAll(&ardupilotmega.MessageSetMode{
			TargetSystem: target_system,
			BaseMode:     ardupilotmega.MAV_MODE(ardupilotmega.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
			CustomMode:   uint32(mode_val),
		})
		if err != nil {
			log.Error("Failed to set pixhawk mode", "error", err)
		}
		return false
	}

	if ack_result == common.MAV_RESULT_ACCEPTED {
		log.Info("[PIXHAWK] Mode changed", "mode", mode)
	} else {
		log.Error("[PIXHAWK] Mode change rejected", "mode", mode, "result", ack_result)
	}
	return false
}

// Returns true if it was preempted by a priority command and has to be re-run
func (app *Master) requestMessageAtInterval(ctx context.Context, msg_id uint32, freq_hz float64) bool {
	log.Info("Request message at interval", "msgid", msg_id, "freq", freq_hz)

	var ack_result common.MAV_RESULT
	var err error
	for attempt := 0; attempt < 3; attempt++ {
		ack_result, err = app.sendCommandLong(ctx, true, common.MAV_CMD_SET_MESSAGE_INTERVAL, float32(msg_id), float32(1e6/freq_hz))
		if errors.Is(err, errPreempted) {
			return true
		}
		if err == nil || ctx.Err() != nil {
			break
		}
	}

	switch {
	case err != nil:
		log.Error("[PIXHAWK] Failed request to modify interval of message", "id", msg_id, "err", err)
	case ack_result != common.MAV_RESULT_ACCEPTED:
		log.Error("[PIXHAWK] Failed request to modify interval of message", "id", msg_id, "result", ack_result)
	default:
		log.Info("[PIXHAWK] Succeded request to modify interval of message", "id", msg_id)
	}
	return false
}

func (app *Master) requestTelemetryStreams() {
	for _, msg := range []message.Message{
		&ardupilotmega.MessageHeartbeat{},
		&ardupilotmega.MessageSysStatus{},
		&ardupilotmega.MessageAttitudeQuaternion{},
		&ardupilotmega.MessageScaledPressure2{},
		&ardupilotmega.MessageVfrHud{},
		&ardupilotmega.MessageServoOutputRaw{},
		&ardupilotmega.MessageAhrs2{},
		&ardupilotmega.MessageScaledImu2{},
	} {
		app.queueCommand(PixhawkCommand{kind: CMD_MESSAGE_INTERVAL, msg_id: msg.GetID(), freq_hz: app.config.stream_rate_hz})
	}
}

// -- ROS CALLBACKS (must never block) --

func (app *Master) onCommandMessage(msg *custom_msgs.Commands, _ *rclgo.MessageInfo, err error) {
	if err != nil {
		log.Error("[ROS] Failed to read /master/commands", "err", err)
		return
	}
	app.evaluateCommand(msg)
}

func (app *Master) onROVMessage(msg *custom_msgs.Commands, _ *rclgo.MessageInfo, err error) {
	if err != nil {
		log.Error("[ROS] Failed to read /rov/commands", "err", err)
		return
	}
	app.evaluateCommand(msg)
}

func (app *Master) evaluateCommand(msg *custom_msgs.Commands) {
	app.mu.Lock()
	defer app.mu.Unlock()

	app.setArmed(msg.Arm)

	app.channel_arr = [8]int{
		int(msg.Pitch),
		int(msg.Roll),
		int(msg.Thrust),
		int(msg.Yaw),
		int(msg.Forward),
		int(msg.Lateral),
		int(msg.Servo1),
		int(msg.Servo2),
	}

	if app.mode != msg.Mode {
		if app.armed {
			app.warnThrottled("mode", "Disarm Pixhawk to change modes")
		} else {
			app.mode = msg.Mode
			app.queueCommand(PixhawkCommand{kind: CMD_SET_MODE, mode: msg.Mode})
		}
	}
}

// Must be called with app.mu held
func (app *Master) setArmed(arm bool) {
	if arm == app.armed {
		return
	}
	if arm {
		if app.emergency_locked {
			app.warnThrottled("locked", "Arm blocked: emergency lock is engaged")
			return
		}
		if time.Since(app.last_arm_failure) < ARM_RETRY_COOLDOWN {
			return // PixHawk just refused, don't hammer it at the command rate
		}
		app.armed = true
		app.queueCommand(PixhawkCommand{kind: CMD_ARM})
	} else {
		app.armed = false
		app.queueCommand(PixhawkCommand{kind: CMD_DISARM})
	}
}

// Must be called with app.mu held
func (app *Master) engageEmergency() {
	app.emergency_locked = true
	app.armed = false
	app.channel_arr = [8]int{NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM}
	// Sent even if we think we are disarmed, the vehicle may have been armed from elsewhere (eg. QGC)
	app.queueCommand(PixhawkCommand{kind: CMD_EMERGENCY_DISARM})
}

// Engages the emergency lock and force disarms. Arming stays blocked until
// /clear_emergency is called. Responds immediately, the disarm happens on the
// priority queue.
func (app *Master) onEmergencyKill(_ *rclgo.ServiceInfo, _ *std_srvs.Trigger_Request, sender std_srvs.TriggerServiceResponseSender) {
	app.mu.Lock()
	app.engageEmergency()
	app.mu.Unlock()

	resp := std_srvs.NewTrigger_Response()
	resp.Success = true
	resp.Message = "Emergency kill engaged, force disarming. Call /clear_emergency to allow arming again"
	log.Warn("[*] " + resp.Message)
	if err := sender.SendResponse(resp); err != nil {
		log.Error("[ROS] Failed to respond to /emergency_kill", "err", err)
	}
}

func (app *Master) onClearEmergency(_ *rclgo.ServiceInfo, _ *std_srvs.Trigger_Request, sender std_srvs.TriggerServiceResponseSender) {
	app.mu.Lock()
	was_locked := app.emergency_locked
	app.emergency_locked = false
	app.mu.Unlock()

	resp := std_srvs.NewTrigger_Response()
	resp.Success = true
	if was_locked {
		resp.Message = "Emergency lock cleared"
	} else {
		resp.Message = "No emergency was active"
	}
	log.Info("[*] " + resp.Message)
	if err := sender.SendResponse(resp); err != nil {
		log.Error("[ROS] Failed to respond to /clear_emergency", "err", err)
	}
}

// Kept for mira2_control_master's killswitch node, which toggles on magnet removed / attached
func (app *Master) onToggleEmergency(_ *rclgo.ServiceInfo, _ *std_srvs.Trigger_Request, sender std_srvs.TriggerServiceResponseSender) {
	resp := std_srvs.NewTrigger_Response()
	resp.Success = true

	app.mu.Lock()
	if !app.emergency_locked {
		app.engageEmergency()
		resp.Message = "Emergency lock engaged, disarming"
		log.Warn("[*] " + resp.Message)
	} else {
		app.emergency_locked = false
		resp.Message = "Emergency lock cleared"
		log.Info("[*] " + resp.Message)
	}
	app.mu.Unlock()

	if err := sender.SendResponse(resp); err != nil {
		log.Error("[ROS] Failed to respond to /toggle_emergency", "err", err)
	}
}

// Must be called with app.mu held
func (app *Master) warnThrottled(key string, msg string) {
	if time.Since(app.last_warn[key]) < time.Second {
		return
	}
	app.last_warn[key] = time.Now()
	log.Warn(msg)
}

// -- TELEMETRY --

func (app *Master) handleTelemetry(ctx context.Context) {
	var (
		sys_status *ardupilotmega.MessageSysStatus
		attitude   *ardupilotmega.MessageAttitudeQuaternion
		vfr_hud    *ardupilotmega.MessageVfrHud
		pressure   *ardupilotmega.MessageScaledPressure2
		servo      *ardupilotmega.MessageServoOutputRaw
		ahrs2      *ardupilotmega.MessageAhrs2
		imu        *ardupilotmega.MessageScaledImu2
	)
	start := time.Now()
	warned := map[string]bool{}
	var last_battery_warning time.Time

	// Publish /master/telemetry at a fixed rate instead of on every MAVLink
	// message, which at the requested stream rates would be several hundred Hz
	ticker := time.NewTicker(time.Duration(float64(time.Second) / app.config.telemetry_rate_hz))
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return

		case data := <-app.telemetry_ch:
			now := time.Now()
			timestamp := float64(now.UnixNano()) / 1e9
			switch msg := data.(type) {
			case *ardupilotmega.MessageSysStatus:
				sys_status = msg
				if voltage := float32(msg.VoltageBattery) / 1000; voltage < 15 && time.Since(last_battery_warning) > 10*time.Second {
					last_battery_warning = time.Now()
					log.Warn("Battery Critically Low ", "battery level", voltage)
				}
			case *ardupilotmega.MessageAttitudeQuaternion:
				attitude = msg
			case *ardupilotmega.MessageVfrHud:
				vfr_hud = msg
				heading_msg := custom_msgs.NewHeading()
				heading_msg.Timestamp = timestamp
				heading_msg.Heading = int32(msg.Heading)
				app.heading_pub.Publish(heading_msg)
			case *ardupilotmega.MessageScaledPressure2:
				pressure = msg
				depth_msg := custom_msgs.NewDepth()
				depth_msg.Timestamp = timestamp
				depth_msg.ExternalPressure = msg.PressAbs
				app.depth_pub.Publish(depth_msg)
			case *ardupilotmega.MessageServoOutputRaw:
				servo = msg
			case *ardupilotmega.MessageAhrs2:
				ahrs2 = msg
			case *ardupilotmega.MessageScaledImu2:
				imu = msg
				app.imu_pub.Publish(imuMessage(msg, now))
			default:
				log.Warn("Unhandled telemetry message recieved ", "msg", reflect.TypeOf(data).String())
			}

		case <-ticker.C:
			// Same as alt_master.py: nothing is published until every message has
			// been seen once, except the depth sensor which may not be connected
			if sys_status == nil || attitude == nil || vfr_hud == nil || servo == nil || ahrs2 == nil {
				if time.Since(start) > 10*time.Second {
					for name, have := range map[string]bool{
						"SYS_STATUS": sys_status != nil, "ATTITUDE_QUATERNION": attitude != nil,
						"VFR_HUD": vfr_hud != nil, "SERVO_OUTPUT_RAW": servo != nil, "AHRS2": ahrs2 != nil,
					} {
						if !have && !warned[name] {
							warned[name] = true
							log.Warn("[PIXHAWK] Still waiting for telemetry message", "msg", name)
						}
					}
				}
				continue
			}
			if pressure == nil && !warned["SCALED_PRESSURE2"] && time.Since(start) > 10*time.Second {
				warned["SCALED_PRESSURE2"] = true
				log.Warn("!!! Is depth sensor connected ? Connect it to I2C port on the pixhawk !!!")
			}

			app.mu.Lock()
			armed := app.armed
			app.mu.Unlock()

			telemetry_msg := custom_msgs.NewTelemetry()
			telemetry_msg.Arm = armed
			telemetry_msg.Timestamp = float64(time.Now().UnixNano()) / 1e9
			telemetry_msg.BatteryVoltage = float32(sys_status.VoltageBattery) / 1000
			telemetry_msg.InternalPressure = vfr_hud.Alt
			telemetry_msg.Heading = int32(vfr_hud.Heading)
			telemetry_msg.ExternalPressure = -1
			if pressure != nil {
				telemetry_msg.ExternalPressure = pressure.PressAbs
			}
			if imu != nil {
				telemetry_msg.ImuXacc = int32(imu.Xacc)
				telemetry_msg.ImuYacc = int32(imu.Yacc)
				telemetry_msg.ImuZacc = int32(imu.Zacc)
				telemetry_msg.ImuGyroX = int32(imu.Xgyro)
				telemetry_msg.ImuGyroY = int32(imu.Ygyro)
				telemetry_msg.ImuGyroZ = int32(imu.Zgyro)
				telemetry_msg.ImuGyroCompassX = int32(imu.Xmag)
				telemetry_msg.ImuGyroCompassY = int32(imu.Ymag)
				telemetry_msg.ImuGyroCompassZ = int32(imu.Zmag)
			}
			telemetry_msg.Q1 = attitude.Q1
			telemetry_msg.Q2 = attitude.Q2
			telemetry_msg.Q3 = attitude.Q3
			telemetry_msg.Q4 = attitude.Q4
			telemetry_msg.Rollspeed = attitude.Rollspeed
			telemetry_msg.Pitchspeed = attitude.Pitchspeed
			telemetry_msg.Yawspeed = attitude.Yawspeed
			telemetry_msg.Roll = ahrs2.Roll
			telemetry_msg.Pitch = ahrs2.Pitch
			telemetry_msg.Yaw = ahrs2.Yaw
			telemetry_msg.ThrusterPwms = [8]float32{
				float32(servo.Servo1Raw), float32(servo.Servo2Raw), float32(servo.Servo3Raw), float32(servo.Servo4Raw),
				float32(servo.Servo5Raw), float32(servo.Servo6Raw), float32(servo.Servo7Raw), float32(servo.Servo8Raw),
			}
			if err := app.telemetry_pub.Publish(telemetry_msg); err != nil {
				log.Error("[ROS] Failed to publish telemetry", "err", err)
			}
		}
	}
}

// Same conversion as alt_master.py's publish_imu
func imuMessage(imu *ardupilotmega.MessageScaledImu2, now time.Time) *sensor_msgs.Imu {
	const G_TO_MS2 = 9.80665
	imu_msg := sensor_msgs.NewImu()
	imu_msg.Header.Stamp.Sec = int32(now.Unix())
	imu_msg.Header.Stamp.Nanosec = uint32(now.Nanosecond())
	// mG -> m/s^2
	imu_msg.LinearAcceleration.X = float64(imu.Xacc) * G_TO_MS2 / 1000
	imu_msg.LinearAcceleration.Y = float64(imu.Yacc) * G_TO_MS2 / 1000
	imu_msg.LinearAcceleration.Z = float64(imu.Zacc) * G_TO_MS2 / 1000
	// mrad/s -> rad/s
	imu_msg.AngularVelocity.X = float64(imu.Xgyro) / 1000
	imu_msg.AngularVelocity.Y = float64(imu.Ygyro) / 1000
	imu_msg.AngularVelocity.Z = float64(imu.Zgyro) / 1000
	// No orientation in this message
	imu_msg.OrientationCovariance[0] = -1
	return imu_msg
}

// -- RC CHANNELS --

// Modify ALL 8 RC channel PWMs with a single RC_CHANNELS_OVERRIDE
func (app *Master) SetRCChannelsPWM(channels [8]int) {
	app.mu.Lock()
	target_system, target_component, have_target := app.target_system, app.target_component, app.have_target
	app.mu.Unlock()
	if !have_target {
		return
	}

	var rc_channels [8]uint16
	for i, pwm := range channels {
		if pwm < 0 {
			pwm = 0
		}
		rc_channels[i] = uint16(pwm)
	}

	// Channels 9-18 are left at 0 which means "ignore" for them
	err := app.pixhawk.WriteMessageAll(&ardupilotmega.MessageRcChannelsOverride{
		TargetSystem:    target_system,
		TargetComponent: target_component,
		Chan1Raw:        rc_channels[0],
		Chan2Raw:        rc_channels[1],
		Chan3Raw:        rc_channels[2],
		Chan4Raw:        rc_channels[3],
		Chan5Raw:        rc_channels[4],
		Chan6Raw:        rc_channels[5],
		Chan7Raw:        rc_channels[6],
		Chan8Raw:        rc_channels[7],
	})
	if err != nil {
		log.Error("[PIXHAWK] Failed to send RC override", "err", err)
	}
}

func (app *Master) Actuate() {
	app.mu.Lock()
	channels := app.channel_arr
	app.mu.Unlock()
	app.SetRCChannelsPWM(channels)
}

// Rate limited: gomavlib silently drops messages once its write queue is
// full, so an unthrottled loop here starves ARM / SET_MODE commands
func (app *Master) updateRCChannels(ctx context.Context) {
	ticker := time.NewTicker(time.Duration(float64(time.Second) / app.config.rc_rate_hz))
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			app.Actuate()
		}
	}
}

// -- MAVLINK EVENTS --

// Runs on the listenForEvents goroutine, every send here is non blocking
func (app *Master) handleEventFrame(frm *gomavlib.EventFrame) {
	switch msg := frm.Message().(type) {
	case *ardupilotmega.MessageHeartbeat:
		// Only the autopilot's heartbeat decides who we send commands to
		if msg.Autopilot == ardupilotmega.MAV_AUTOPILOT_INVALID || msg.Type == ardupilotmega.MAV_TYPE_GCS {
			return
		}
		pixhawk_armed := msg.BaseMode&ardupilotmega.MAV_MODE_FLAG_SAFETY_ARMED != 0

		app.mu.Lock()
		reconnected := !app.have_target || time.Since(app.last_heartbeat) > HEARTBEAT_TIMEOUT
		armed_changed := app.have_target && pixhawk_armed != app.pixhawk_armed
		app.target_system = frm.SystemID()
		app.target_component = frm.ComponentID()
		app.have_target = true
		app.last_heartbeat = time.Now()
		app.pixhawk_armed = pixhawk_armed
		app.mu.Unlock()

		if reconnected {
			log.Info("[PIXHAWK] Recieved heartbeat", "system", frm.SystemID(), "component", frm.ComponentID())
			app.requestTelemetryStreams()
		}
		if armed_changed {
			log.Info("[PIXHAWK] Armed state changed", "armed", pixhawk_armed)
		}
		select {
		case app.heartbeat_ch <- struct{}{}:
		default: // nobody waiting
		}
	case *ardupilotmega.MessageCommandAck:
		if app.log_misc {
			log.Info("[PIXHAWK] Recieved Command Acknowledgement", "ack", msg)
		}
		select {
		case app.ack_ch <- *msg:
		default: // nobody waiting
		}
	case *ardupilotmega.MessageStatustext:
		log.Info("[PIXHAWK]", "status_text", msg.Text)
	case *ardupilotmega.MessageAhrs2, *ardupilotmega.MessageScaledImu2, *ardupilotmega.MessageVfrHud, *ardupilotmega.MessageAttitudeQuaternion, *ardupilotmega.MessageScaledPressure2, *ardupilotmega.MessageSysStatus, *ardupilotmega.MessageServoOutputRaw:
		// Only telemetry from the autopilot we command
		app.mu.Lock()
		from_target := app.have_target && frm.SystemID() == app.target_system && frm.ComponentID() == app.target_component
		app.mu.Unlock()
		if !from_target {
			return
		}
		if app.log_misc {
			log.Info("[PIXHAWK] Recieved Telemetry Message", "message", reflect.TypeOf(msg).String())
		}
		select {
		case app.telemetry_ch <- msg:
		default:
			log.Warn("[PIXHAWK] Telemetry queue full, dropping message", "message", reflect.TypeOf(msg).String())
		}
	default:
		if app.log_misc {
			log.Warn("[PIXHAWK] Unhandeled Message Type ", "msg_type", reflect.TypeOf(msg).String(), "msg", msg)
		}
	}
}

func (app *Master) listenForEvents(ctx context.Context) {
	for {
		select {
		case <-ctx.Done():
			return
		case evt, ok := <-app.pixhawk.Events():
			if !ok {
				return
			}
			switch v := evt.(type) {
			case *gomavlib.EventChannelClose:
				log.Warn("[PIXHAWK] Event channel closed", "channel", v.Channel)
			case *gomavlib.EventChannelOpen:
				log.Info("[PIXHAWK] Event channel opened", "channel", v.Channel)
			case *gomavlib.EventParseError:
				log.Debug("[PIXHAWK] Event parse error ", "err", v.Error)
			case *gomavlib.EventFrame:
				app.handleEventFrame(v)
			}
		}
	}
}

// Waits for the next heartbeat, like pymavlink's wait_heartbeat(). Stops early
// if interrupt fires (pass nil to wait for the full timeout).
func (app *Master) waitForHeartbeat(ctx context.Context, timeout time.Duration, interrupt <-chan struct{}) error {
	// Discard a heartbeat that arrived while nobody was waiting
	select {
	case <-app.heartbeat_ch:
	default:
	}
	select {
	case <-app.heartbeat_ch:
		return nil
	case <-interrupt:
		return errors.New("interrupted by a priority command")
	case <-time.After(timeout):
		return errors.New("timed out waiting for heartbeat")
	case <-ctx.Done():
		return ctx.Err()
	}
}

func main() {
	// -- PARSE ARGUMENTS --
	ros_args, rest_args, err := rclgo.ParseArgs(os.Args[1:])
	if err != nil {
		log.Fatal("[ROS] Failed to parse ROS arguments", "err", err)
	}
	config, err := loadConfig(os.Args[1:], rest_args)
	if err != nil {
		log.Fatal("[*] Invalid configuration", "err", err)
	}

	// -- INIT NODE --
	log.Info("[ROS] Initializing ROS 2")
	if err := rclgo.Init(ros_args); err != nil {
		log.Fatal("[ROS] Failed to initialize rclgo", "err", err)
	}
	defer rclgo.Uninit()

	n, err := rclgo.NewNode(NODE_NAME, "")
	if err != nil {
		log.Fatal("[ROS] Failed to create node", "err", err)
	}
	defer n.Close()

	// MAVLINK
	log.Info("[PIXHAWK] Connecting to PixHawk", "address", config.pixhawk_address, "baud", config.baud)
	endpoint, err := mavaddr.Parse(config.pixhawk_address, config.baud)
	if err != nil {
		log.Fatal("[PIXHAWK] Invalid pixhawk_address", "err", err)
	}
	mav_node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints:  []gomavlib.EndpointConf{endpoint},
		Dialect:    ardupilotmega.Dialect,
		OutVersion: gomavlib.V2,
		// ArduPilot ignores RC_CHANNELS_OVERRIDE unless it comes from the
		// system id in SYSID_MYGCS (255 by default, same as pymavlink)
		OutSystemID:    byte(config.source_system),
		OutComponentID: byte(config.source_component),
		// pymavlink does not send heartbeats either
		HeartbeatDisable: true,
	})

	if err != nil {
		log.Fatal("[PIXHAWK] Failed to connect to Pixhawk!", "err", err)
	}
	defer mav_node.Close()

	// Setup State
	app := NewApplication(config, mav_node, n)

	sig_ctx, stop_signals := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop_signals()
	// Separate from sig_ctx so we can still disarm after a signal
	ctx, cancel := context.WithCancel(context.Background())

	var wg sync.WaitGroup

	wg.Add(1)
	go func() {
		app.listenForEvents(ctx)
		log.Warn("[*] Finished listening for events")
		wg.Done()
	}()

	log.Info("[PIXHAWK] Waiting for heartbeat")
	for app.waitForHeartbeat(sig_ctx, 5*time.Second, nil) != nil {
		if sig_ctx.Err() != nil {
			cancel()
			wg.Wait()
			return
		}
		log.Warn("[PIXHAWK] Still waiting for heartbeat, is the PixHawk connected?")
	}

	wg.Add(4)
	go func() {
		app.handleCommands(ctx)
		log.Warn("[*] Finished handling commands")
		wg.Done()
	}()

	go func() {
		app.handleTelemetry(ctx)
		log.Warn("[*] Finished handling telemetry")
		wg.Done()
	}()

	go func() {
		app.updateRCChannels(ctx)
		log.Warn("[*] Finished updating RC channels")
		wg.Done()
	}()

	go func() {
		if err := n.Spin(ctx); err != nil && ctx.Err() == nil {
			log.Error("[ROS] Spin stopped", "err", err)
		}
		log.Warn("[*] Finished spinning ROS node")
		wg.Done()
	}()

	// Start listening for user input to arm or disarm the vehicle
	go func() {
		scanner := bufio.NewScanner(os.Stdin)
		for scanner.Scan() { // stops on EOF, eg. under ros2 launch
			input := scanner.Text()

			app.mu.Lock()
			switch input {
			case "arm":
				app.setArmed(true)
			case "disarm":
				app.setArmed(false)
			default:
				log.Warn("Invalid input, please enter 'arm' or 'disarm'.")
			}
			app.mu.Unlock()
		}
	}()

	log.Info("[*] Init Complete")
	<-sig_ctx.Done()

	app.cleanup()
	cancel()
	wg.Wait()
}

// ROS parameters (`--ros-args -p name:=value` or a launch file's params
// file) plus master.py's -p/--port and -m/--mode flags
func loadConfig(all_args []string, non_ros_args []string) (Config, error) {
	params, err := rosparam.Parse(all_args, NODE_NAME)
	if err != nil {
		return Config{}, err
	}

	config := Config{
		pixhawk_address: params.String("pixhawk_address", "/dev/Pixhawk"),
		initial_mode:    params.String("initial_mode", "STABILIZE"),
	}
	for _, p := range []struct {
		dst  *int
		name string
		def  int
	}{
		{&config.baud, "baud", 57600},
		{&config.source_system, "source_system", 255},
		{&config.source_component, "source_component", 0},
	} {
		if *p.dst, err = params.Int(p.name, p.def); err != nil {
			return Config{}, err
		}
	}
	for _, p := range []struct {
		dst  *float64
		name string
		def  float64
	}{
		{&config.rc_rate_hz, "rc_rate_hz", 25},
		{&config.stream_rate_hz, "stream_rate_hz", 100},
		{&config.telemetry_rate_hz, "telemetry_rate_hz", 50},
	} {
		if *p.dst, err = params.Float(p.name, p.def); err != nil {
			return Config{}, err
		}
		if *p.dst <= 0 {
			return Config{}, errors.New(p.name + " must be positive")
		}
	}
	if config.source_system < 1 || config.source_system > 255 || config.source_component < 0 || config.source_component > 255 {
		return Config{}, errors.New("source_system must be 1-255 and source_component 0-255")
	}

	fs := flag.NewFlagSet("master", flag.ContinueOnError)
	port := fs.String("port", "", "Pixhawk address (same as the pixhawk_address parameter)")
	fs.StringVar(port, "p", "", "shorthand for -port")
	mode := fs.String("mode", "", "initial mode (same as the initial_mode parameter)")
	fs.StringVar(mode, "m", "", "shorthand for -mode")
	if err := fs.Parse(non_ros_args); err != nil {
		return Config{}, err
	}
	if *port != "" {
		config.pixhawk_address = *port
	}
	if *mode != "" {
		config.initial_mode = *mode
	}
	return config, nil
}
