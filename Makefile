COMPOSE := docker compose -f compose.yaml
ifeq ($(shell uname -s),Darwin)
COMPOSE += -f compose.mac.yaml
else ifeq ($(filter web-sim,$(MAKECMDGOALS)):$(G1_SIM_VIEWER),web-sim:1)
COMPOSE += -f compose.x11.yaml
endif

.PHONY: build up down shell

build:
	$(COMPOSE) build web-sim web-mock

up:
ifeq ($(words $(filter web-sim web-mock,$(MAKECMDGOALS))),1)
	$(COMPOSE) up $(filter web-sim web-mock,$(MAKECMDGOALS))
else
	@echo "Usage: make up web-sim | make up web-mock" >&2; exit 2
endif

down:
	$(COMPOSE) --profile web-mock down

shell:
	$(COMPOSE) exec web-sim bash

ifneq ($(filter up,$(MAKECMDGOALS)),)
.PHONY: web-sim web-mock
web-sim web-mock: up
	@:
endif
