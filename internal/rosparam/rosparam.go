// Package rosparam extracts ROS 2 parameter overrides from the command line.
//
// rclgo does not implement ROS 2 parameters, but launch files and `ros2 run`
// still pass them as `--ros-args -p name:=value` or `--params-file <yaml>`.
// This package reads both forms so the node accepts the same configuration
// as the rclpy implementation.
package rosparam

import (
	"fmt"
	"os"
	"strconv"
	"strings"

	"gopkg.in/yaml.v3"
)

// Params holds parameter values keyed by parameter name.
type Params map[string]string

// Parse extracts the parameters meant for nodeName from args (normally
// os.Args[1:]). Parameters from later arguments override earlier ones, which
// matches rcl's precedence rules. A `-r __node:=name` remap changes the node
// name that params-file entries are matched against.
func Parse(args []string, nodeName string) (Params, error) {
	type item struct {
		file  string
		name  string
		value string
	}
	var items []item

	inROS := false
	for i := 0; i < len(args); i++ {
		arg := args[i]
		if arg == "--ros-args" {
			inROS = true
			continue
		}
		if !inROS {
			continue
		}
		if arg == "--" {
			inROS = false
			continue
		}

		next := func() (string, bool) {
			if i+1 >= len(args) {
				return "", false
			}
			i++
			return args[i], true
		}

		switch arg {
		case "-p", "--param":
			v, ok := next()
			if !ok {
				return nil, fmt.Errorf("%s requires an argument", arg)
			}
			name, value, found := strings.Cut(v, ":=")
			if !found {
				return nil, fmt.Errorf("invalid parameter override %q, expected name:=value", v)
			}
			items = append(items, item{name: name, value: value})
		case "--params-file":
			v, ok := next()
			if !ok {
				return nil, fmt.Errorf("%s requires an argument", arg)
			}
			items = append(items, item{file: v})
		case "-r", "--remap":
			v, ok := next()
			if !ok {
				return nil, fmt.Errorf("%s requires an argument", arg)
			}
			if from, to, found := strings.Cut(v, ":="); found && from == "__node" {
				nodeName = to
			}
		}
	}

	params := Params{}
	for _, it := range items {
		if it.file == "" {
			params[it.name] = unquote(it.value)
			continue
		}
		fromFile, err := readParamsFile(it.file, nodeName)
		if err != nil {
			return nil, err
		}
		for k, v := range fromFile {
			params[k] = v
		}
	}
	return params, nil
}

// unquote decodes a command line parameter value the way rcl does: as a YAML
// scalar, so that `'/dev/ttyACM0'` and `/dev/ttyACM0` are equivalent.
func unquote(v string) string {
	var out any
	if err := yaml.Unmarshal([]byte(v), &out); err != nil {
		return v
	}
	if s, ok := out.(string); ok {
		return s
	}
	return v
}

func readParamsFile(path, nodeName string) (Params, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("reading params file: %w", err)
	}
	var doc map[string]struct {
		Params map[string]yaml.Node `yaml:"ros__parameters"`
	}
	if err := yaml.Unmarshal(data, &doc); err != nil {
		return nil, fmt.Errorf("parsing params file %s: %w", path, err)
	}

	params := Params{}
	// Wildcard entries first so node-specific entries take precedence.
	for _, key := range []string{"/**", "**", "/*", "*", nodeName, "/" + nodeName} {
		entry, ok := doc[key]
		if !ok {
			continue
		}
		for name, node := range entry.Params {
			if node.Kind == yaml.ScalarNode {
				params[name] = node.Value
			}
		}
	}
	return params, nil
}

// String returns the parameter name or def if it was not set.
func (p Params) String(name, def string) string {
	if v, ok := p[name]; ok {
		return v
	}
	return def
}

// Int returns the integer parameter name or def if it was not set.
func (p Params) Int(name string, def int) (int, error) {
	v, ok := p[name]
	if !ok {
		return def, nil
	}
	i, err := strconv.Atoi(v)
	if err != nil {
		return 0, fmt.Errorf("parameter %s: %w", name, err)
	}
	return i, nil
}

// Float returns the floating point parameter name or def if it was not set.
func (p Params) Float(name string, def float64) (float64, error) {
	v, ok := p[name]
	if !ok {
		return def, nil
	}
	f, err := strconv.ParseFloat(v, 64)
	if err != nil {
		return 0, fmt.Errorf("parameter %s: %w", name, err)
	}
	return f, nil
}
