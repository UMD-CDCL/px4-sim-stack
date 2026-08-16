# px4-sim-stack - the front door.
# Run `make` for the target list.

SHELL := /bin/bash
DC    := docker compose
ROOT  := $(shell pwd)

# Every target loads .env, so `make SCENE=forest up` works as an override.
ifneq (,$(wildcard ./.env))
include .env
export
endif

# Which DeepStream this machine can run, from the driver. These override the
# .env values above, which is the point: the release that starts here is a
# property of this machine, not of the file. See scripts/ds-select.sh.
# One $(shell) call resolves all three values, so nvidia-smi runs once, not
# three times. The .env pins go to the script on its command line, because
# GNU Make below 4.4 does not export variables to $(shell).
DS_SELECT  := $(shell DS_IMAGE="$(DS_IMAGE)" DS_VERSION="$(DS_VERSION)" DS_FLAVOUR="$(DS_FLAVOUR)" ./scripts/ds-select.sh)
DS_VERSION := $(patsubst DS_VERSION=%,%,$(filter DS_VERSION=%,$(DS_SELECT)))
DS_IMAGE   := $(patsubst DS_IMAGE=%,%,$(filter DS_IMAGE=%,$(DS_SELECT)))
DS_TAG     := $(patsubst DS_TAG=%,%,$(filter DS_TAG=%,$(DS_SELECT)))

# Home and fiducial ride inside a generated scenario, so SCENARIO alone
# carries them. Empty output, such as for a hand-written scenario, leaves
# the .env values alone. Same one-$(shell) pattern as ds-select above.
SCENARIO_ENV := $(shell ./scripts/scenario-env.sh modules/sim/scenes/scenarios/$(SCENARIO).yaml 2>/dev/null)
ifneq ($(SCENARIO_ENV),)
HOME_LAT := $(patsubst HOME_LAT=%,%,$(filter HOME_LAT=%,$(SCENARIO_ENV)))
HOME_LON := $(patsubst HOME_LON=%,%,$(filter HOME_LON=%,$(SCENARIO_ENV)))
HOME_ALT := $(patsubst HOME_ALT=%,%,$(filter HOME_ALT=%,$(SCENARIO_ENV)))
FIDUCIAL_ENABLED := 1
FIDUCIAL_SURVEYED_LAT := $(patsubst FIDUCIAL_SURVEYED_LAT=%,%,$(filter FIDUCIAL_SURVEYED_LAT=%,$(SCENARIO_ENV)))
FIDUCIAL_SURVEYED_LON := $(patsubst FIDUCIAL_SURVEYED_LON=%,%,$(filter FIDUCIAL_SURVEYED_LON=%,$(SCENARIO_ENV)))
FIDUCIAL_SURVEYED_ALT := $(patsubst FIDUCIAL_SURVEYED_ALT=%,%,$(filter FIDUCIAL_SURVEYED_ALT=%,$(SCENARIO_ENV)))
endif

.DEFAULT_GOAL := help
.PHONY: help preflight bootstrap x11 build build-% up up-core down restart ps logs \
        sim ros qgc perception hub px4-console scenario scene reset genscene \
        clean clean-src lint-docs check endpoints

## ----------------------------------------------------------------- setup

help: ## Show this list
	@echo "px4-sim-stack"
	@echo ""
	@grep -hE '^[a-zA-Z_%-]+:.*?## .*$$' $(MAKEFILE_LIST) \
	  | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[1m%-16s\033[0m %s\n", $$1, $$2}'
	@echo ""
	@echo "  Current scene: $(SCENE)   vehicle: $(VEHICLE)   ros stack: $(ROS_STACK)"

preflight: ## Check the host: driver, docker, GPU runtime, X11, disk
	@./scripts/preflight.sh

bootstrap: ## Clone and pin PX4, QGroundControl and the ROS workspace into ./src
	@./scripts/bootstrap.sh

x11: ## Write the X11 cookie the containers need
	@./scripts/x11-allow.sh

## ----------------------------------------------------------------- build

build: x11 ## Build every image
	$(DC) build

build-%: ## Build one image, for example `make build-sim`
	$(DC) build $*

## ----------------------------------------------------------------- run

up: x11 ## Start the stack, following COMPOSE_PROFILES in .env
	$(DC) up -d
	@echo ""
	@$(MAKE) --no-print-directory endpoints

up-core: x11 ## Start only sim, transport and QGC
	COMPOSE_PROFILES= $(DC) up -d
	@$(MAKE) --no-print-directory endpoints

down: ## Stop the stack and remove the containers
	COMPOSE_PROFILES=ros,perception,xrce,qgc-dev,scenegen $(DC) down --remove-orphans

restart: ## Restart one service, for example `make restart S=sim`
	$(DC) restart $(S)

ps: ## Show the running services
	$(DC) ps

logs: ## Follow the logs, optionally of one service: `make logs S=sim`
	$(DC) logs -f --tail=100 $(S)

endpoints: ## Print the module contracts
	@echo "  MAVLink   udp  mavlink-hub:14551   (ROS)     host: udp://localhost:14550"
	@echo "            tcp  localhost:5760      (MAVSDK, pymavlink)"
	@echo "  Video     rtsp://localhost:8554/gimbal"
	@echo "            rtsp://localhost:8554/nadir"
	@echo "            rtsp://localhost:8554/gimbal_annotated   (perception profile)"
	@echo "            http://localhost:8889/gimbal             (WebRTC, browser)"
	@echo "  Detections mqtt://localhost:1883   topic perception/detections"
	@echo "  Foxglove  ws://localhost:8765      (ros profile)"

## ----------------------------------------------------------------- shells

sim: ## Open a shell in the sim container
	$(DC) exec sim bash

ros: ## Open a ROS 2 shell with the stack overlay sourced
	$(DC) exec ros bash

qgc: ## Open a shell in the QGroundControl container
	$(DC) exec qgc bash

perception: ## Open a shell in the DeepStream container
	$(DC) exec perception bash

hub: ## Open a shell in the MAVLink hub
	$(DC) exec mavlink-hub sh

px4-console: ## Attach to the PX4 pxh> console. Detach with Ctrl-P Ctrl-Q.
	@echo "Attaching to pxh>. Detach with Ctrl-P Ctrl-Q (not Ctrl-C)."
	@docker attach $$($(DC) ps -q sim)

## ----------------------------------------------------------------- scenes

scene: ## Switch the world and restart the sim: `make scene SCENE=forest`
	$(DC) up -d --force-recreate sim
	@echo "sim restarted with SCENE=$(SCENE) VEHICLE=$(VEHICLE)"

scenario: ## Re-run the scenario spawner without a sim restart
	$(DC) exec sim /scenes/spawn_scenario.py \
	  --world $(SCENE) --scenario /scenes/scenarios/$(SCENARIO).yaml

genscene: ## Build a world from map data: `make genscene ARGS="create --name x ..."`
	COMPOSE_PROFILES=scenegen $(DC) run --rm --service-ports scenegen $(ARGS)

reset: ## Clear the scenario targets from the running world
	$(DC) exec sim /scenes/spawn_scenario.py --world $(SCENE) --clear

## ----------------------------------------------------------------- quality

lint-docs: ## Lint the docs with the STE writing linter
	@./scripts/lint-docs.sh

check: preflight ## Validate the compose file and lint the docs
	$(DC) config -q && echo "compose.yaml OK"
	@$(MAKE) --no-print-directory lint-docs

## ----------------------------------------------------------------- cleanup

clean: ## Remove containers, networks and named volumes
	COMPOSE_PROFILES=ros,perception,xrce,qgc-dev,scenegen $(DC) down -v --remove-orphans

clean-src: ## Delete the cloned upstream sources in ./src
	rm -rf ./src/PX4-Autopilot ./src/qgroundcontrol ./src/ros2_ws
