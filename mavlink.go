package main

import (
	"context"
	"errors"
	"sync"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/message"
)

var errNoAck = errors.New("no COMMAND_ACK received")

// vehicleLink wraps the gomavlib node with what pymavlink's mavfile gives the
// Python master for free: tracking of the autopilot's system/component id
// from its heartbeat, wait_heartbeat(), and matching COMMAND_ACKs to the
// command that caused them.
type vehicleLink struct {
	node *gomavlib.Node

	mu            sync.Mutex
	targetSystem  uint8
	targetComp    uint8
	haveTarget    bool
	lastHeartbeat time.Time
	armed         bool
	// closed and replaced on every autopilot heartbeat
	heartbeatSignal chan struct{}
	ackWaiters      map[common.MAV_CMD][]chan *common.MessageCommandAck
}

func newVehicleLink(node *gomavlib.Node) *vehicleLink {
	return &vehicleLink{
		node:            node,
		heartbeatSignal: make(chan struct{}),
		ackWaiters:      map[common.MAV_CMD][]chan *common.MessageCommandAck{},
	}
}

// heartbeatEvent describes an autopilot heartbeat handled by the link.
type heartbeatEvent struct {
	first        bool // first heartbeat, or first after the link was lost
	armedChanged bool
	armed        bool
	system       uint8
	component    uint8
}

// handleHeartbeat records an autopilot heartbeat. Heartbeats from GCSes,
// gimbals, cameras etc. are ignored so that they never become the target of
// our commands.
func (l *vehicleLink) handleHeartbeat(frm *gomavlib.EventFrame, hb *ardupilotmega.MessageHeartbeat) (heartbeatEvent, bool) {
	if hb.Autopilot == ardupilotmega.MAV_AUTOPILOT_INVALID || hb.Type == ardupilotmega.MAV_TYPE_GCS {
		return heartbeatEvent{}, false
	}

	l.mu.Lock()
	defer l.mu.Unlock()

	now := time.Now()
	armed := hb.BaseMode&ardupilotmega.MAV_MODE_FLAG_SAFETY_ARMED != 0
	ev := heartbeatEvent{
		first:        !l.haveTarget || now.Sub(l.lastHeartbeat) > heartbeatLostAfter,
		armedChanged: l.haveTarget && armed != l.armed,
		armed:        armed,
		system:       frm.SystemID(),
		component:    frm.ComponentID(),
	}
	l.targetSystem = frm.SystemID()
	l.targetComp = frm.ComponentID()
	l.haveTarget = true
	l.lastHeartbeat = now
	l.armed = armed

	close(l.heartbeatSignal)
	l.heartbeatSignal = make(chan struct{})
	return ev, true
}

// handleAck hands a COMMAND_ACK to everybody waiting on that command.
func (l *vehicleLink) handleAck(ack *common.MessageCommandAck) {
	l.mu.Lock()
	waiters := l.ackWaiters[ack.Command]
	delete(l.ackWaiters, ack.Command)
	l.mu.Unlock()

	for _, ch := range waiters {
		ch <- ack // buffered, never blocks
	}
}

func (l *vehicleLink) target() (system, component uint8, ok bool) {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.targetSystem, l.targetComp, l.haveTarget
}

// waitHeartbeat blocks until the next autopilot heartbeat, like pymavlink's
// wait_heartbeat().
func (l *vehicleLink) waitHeartbeat(ctx context.Context, timeout time.Duration) error {
	l.mu.Lock()
	signal := l.heartbeatSignal
	l.mu.Unlock()

	select {
	case <-signal:
		return nil
	case <-time.After(timeout):
		return errors.New("timed out waiting for heartbeat")
	case <-ctx.Done():
		return ctx.Err()
	}
}

// write sends a message. gomavlib silently drops messages when its write
// queue is full, so callers must not flood the link.
func (l *vehicleLink) write(msg message.Message) error {
	return l.node.WriteMessageAll(msg)
}

// commandLong sends a COMMAND_LONG to the autopilot and waits for its ack.
func (l *vehicleLink) commandLong(ctx context.Context, cmd common.MAV_CMD, timeout time.Duration, params ...float32) (common.MAV_RESULT, error) {
	sys, comp, ok := l.target()
	if !ok {
		return 0, errors.New("autopilot not discovered yet")
	}
	var p [7]float32
	copy(p[:], params)

	return l.sendAndAwaitAck(ctx, cmd, timeout, &ardupilotmega.MessageCommandLong{
		TargetSystem:    sys,
		TargetComponent: comp,
		Command:         cmd,
		Confirmation:    0,
		Param1:          p[0],
		Param2:          p[1],
		Param3:          p[2],
		Param4:          p[3],
		Param5:          p[4],
		Param6:          p[5],
		Param7:          p[6],
	})
}

// setMode changes the flight mode with MAV_CMD_DO_SET_MODE, which ArduPilot
// handles exactly like the SET_MODE message pymavlink's set_mode_send uses,
// but also acknowledges. Without an ack it falls back to SET_MODE.
func (l *vehicleLink) setMode(ctx context.Context, customMode uint32, timeout time.Duration) (common.MAV_RESULT, error) {
	result, err := l.commandLong(ctx, common.MAV_CMD_DO_SET_MODE, timeout,
		float32(ardupilotmega.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), float32(customMode))
	if !errors.Is(err, errNoAck) {
		return result, err
	}
	sys, _, _ := l.target()
	if werr := l.write(&ardupilotmega.MessageSetMode{
		TargetSystem: sys,
		BaseMode:     ardupilotmega.MAV_MODE(ardupilotmega.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
		CustomMode:   customMode,
	}); werr != nil {
		return 0, werr
	}
	return 0, err
}

func (l *vehicleLink) sendAndAwaitAck(ctx context.Context, cmd common.MAV_CMD, timeout time.Duration, msg message.Message) (common.MAV_RESULT, error) {
	ch := make(chan *common.MessageCommandAck, 1)
	l.mu.Lock()
	l.ackWaiters[cmd] = append(l.ackWaiters[cmd], ch)
	l.mu.Unlock()
	defer l.cancelAckWaiter(cmd, ch)

	if err := l.write(msg); err != nil {
		return 0, err
	}

	select {
	case ack := <-ch:
		return ack.Result, nil
	case <-time.After(timeout):
		return 0, errNoAck
	case <-ctx.Done():
		return 0, ctx.Err()
	}
}

func (l *vehicleLink) cancelAckWaiter(cmd common.MAV_CMD, ch chan *common.MessageCommandAck) {
	l.mu.Lock()
	defer l.mu.Unlock()
	waiters := l.ackWaiters[cmd]
	for i, w := range waiters {
		if w == ch {
			l.ackWaiters[cmd] = append(waiters[:i], waiters[i+1:]...)
			break
		}
	}
	if len(l.ackWaiters[cmd]) == 0 {
		delete(l.ackWaiters, cmd)
	}
}

// rcOverride sends all eight RC channels in one RC_CHANNELS_OVERRIDE.
// UINT16_MAX means "leave this channel alone" for channels 1-8 and 0 means
// the same for channels 9-18.
func (l *vehicleLink) rcOverride(ch [8]uint16) error {
	sys, comp, ok := l.target()
	if !ok {
		return errors.New("autopilot not discovered yet")
	}
	return l.write(&ardupilotmega.MessageRcChannelsOverride{
		TargetSystem:    sys,
		TargetComponent: comp,
		Chan1Raw:        ch[0],
		Chan2Raw:        ch[1],
		Chan3Raw:        ch[2],
		Chan4Raw:        ch[3],
		Chan5Raw:        ch[4],
		Chan6Raw:        ch[5],
		Chan7Raw:        ch[6],
		Chan8Raw:        ch[7],
	})
}
