# px4-sim-stack - the make targets forward to ./px4sim.
#
# ./px4sim is the front door. It reads .env, picks the DeepStream release for
# this machine, and resolves the world origin from the scene and the scenario.
# Every target here is a name people already type; the work happens in the
# script. Run `./px4sim` or `make help` for the command list.

SHELL := /bin/bash
PX4SIM := ./px4sim

.DEFAULT_GOAL := help
.PHONY: help preflight bootstrap x11 build build-% up up-core down restart ps logs \
        sim ros qgc perception hub px4-console scenario scene origin reset genscene \
        clean clean-src lint-docs check endpoints

## ----------------------------------------------------------------- setup

help: ## Show the command list
	@$(PX4SIM) help

preflight: ## Check the host: driver, docker, GPU runtime, X11, disk
	@$(PX4SIM) doctor

bootstrap: ## Clone and pin PX4, QGroundControl and the ROS workspace into ./src
	@$(PX4SIM) setup

x11: ## Write the X11 cookie the containers need
	@$(PX4SIM) x11

## ----------------------------------------------------------------- build

build: ## Build every image
	@$(PX4SIM) build

build-%: ## Build one image, for example `make build-sim`
	@$(PX4SIM) build $*

## ----------------------------------------------------------------- run

up: ## Start the stack, following COMPOSE_PROFILES in .env
	@$(PX4SIM) start

up-core: ## Start only sim, transport and QGC
	@$(PX4SIM) core

down: ## Stop the stack and remove the containers
	@$(PX4SIM) stop

restart: ## Restart one service, for example `make restart S=sim`
	@$(PX4SIM) restart $(S)

ps: ## Show the running services
	@$(PX4SIM) status

logs: ## Follow the logs, optionally of one service: `make logs S=sim`
	@$(PX4SIM) logs $(S)

endpoints: ## Print the module contracts
	@$(PX4SIM) endpoints

## ----------------------------------------------------------------- shells

sim: ## Open a shell in the sim container
	@$(PX4SIM) sim

ros: ## Open a ROS 2 shell with the stack overlay sourced
	@$(PX4SIM) ros

qgc: ## Open a shell in the QGroundControl container
	@$(PX4SIM) qgc

perception: ## Open a shell in the DeepStream container
	@$(PX4SIM) perception

hub: ## Open a shell in the MAVLink hub
	@$(PX4SIM) hub

px4-console: ## Attach to the PX4 pxh> console. Detach with Ctrl-P Ctrl-Q.
	@$(PX4SIM) console

## ----------------------------------------------------------------- scenes

scene: ## Switch the world and restart the sim: `make scene SCENE=forest`
	@$(PX4SIM) scene $(SCENE)

scenario: ## Re-run the scenario spawner without a sim restart
	@$(PX4SIM) scenario $(SCENARIO)

origin: ## Print the coordinates this scene and scenario fly at
	@$(PX4SIM) origin

genscene: ## Build a world from map data: `make genscene ARGS="create --name x ..."`
	@$(PX4SIM) genscene $(ARGS)

reset: ## Clear the scenario targets from the running world
	@$(PX4SIM) reset

## ----------------------------------------------------------------- quality

lint-docs: ## Lint the docs with the STE writing linter
	@./scripts/lint-docs.sh

check: ## Validate the compose file and lint the docs
	@$(PX4SIM) check

## ----------------------------------------------------------------- cleanup

clean: ## Remove containers, networks and named volumes
	@$(PX4SIM) clean

clean-src: ## Delete the cloned upstream sources in ./src
	@$(PX4SIM) clean-src
