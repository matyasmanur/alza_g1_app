COMPOSE := docker compose -f compose.yaml
ifeq ($(shell uname -s),Darwin)
COMPOSE += -f compose.mac.yaml
else ifeq ($(filter web-sim,$(MAKECMDGOALS)):$(G1_SIM_VIEWER),web-sim:1)
COMPOSE += -f compose.x11.yaml
endif

WEB_SERVICES := web-sim web-mock web-hw

.PHONY: build up down shell

build:
	$(COMPOSE) build web-sim web-mock

up:
ifeq ($(words $(filter $(WEB_SERVICES),$(MAKECMDGOALS))),1)
	$(COMPOSE) up $(filter $(WEB_SERVICES),$(MAKECMDGOALS))
else
	@echo "Usage: make up web-sim | make up web-mock | make up web-hw" >&2; exit 2
endif

down:
	$(COMPOSE) --profile web-mock --profile web-hw down

shell:
	$(COMPOSE) exec web-sim bash

ifneq ($(filter up,$(MAKECMDGOALS)),)
.PHONY: $(WEB_SERVICES)
$(WEB_SERVICES): up
	@:
endif
