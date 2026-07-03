// Package main is a TRIVIAL viam Go module to empirically test bug #73:
// a valid->invalid->valid config cycle on a modular resource.
//
// The model is a generic component. Its Validate returns an error iff the
// config attribute `bad` is true. The constructor and Close log LOUDLY so we
// can see, from the viam-server log, exactly which lifecycle calls happen
// during the cycle (in particular: is Close ever called on the orphan?).
package main

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

var myModel = resource.NewModel("repro73", "demo", "thing")

// Config is the native config. `bad=true` makes Validate fail.
type Config struct {
	Bad bool `json:"bad"`
}

// Validate fails iff bad==true. No dependencies.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Bad {
		return nil, nil, fmt.Errorf("repro73: config is INVALID (bad=true) for %q", path)
	}
	return []string{}, nil, nil
}

func main() {
	resource.RegisterComponent(generic.API, myModel, resource.Registration[resource.Resource, *Config]{
		Constructor: newThing,
	})
	module.ModularMain(resource.APIModel{API: generic.API, Model: myModel})
}

func newThing(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (resource.Resource, error) {
	logger.Infof("REPRO73_CONSTRUCT: constructing instance for resource %q (ptr will follow in Close)", conf.ResourceName().Name)
	t := &thing{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
	}
	logger.Infof("REPRO73_CONSTRUCT_DONE: instance %p is now LIVE for %q", t, conf.ResourceName().Name)
	return t, nil
}

type thing struct {
	resource.Named
	logger logging.Logger
}

func (t *thing) DoCommand(ctx context.Context, req map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{"ok": true}, nil
}

// Close logs LOUDLY. If bug #73 holds, the ORPHANED instance's Close is NEVER
// called during the valid->invalid->valid brick loop.
func (t *thing) Close(ctx context.Context) error {
	t.logger.Infof("REPRO73_CLOSE: Close() CALLED on instance %p (%q)", t, t.Name().Name)
	return nil
}
