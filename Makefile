# Makefile for make commands to be run OUTSIDE the docker container

IMAGE=idscfrazz/dt-ros-noetic-vnc
TAG?=daffy-$(ARCH)

FULL_IMAGE=$(IMAGE):$(TAG)
CONTAINER_NAME=main-workspace
# Directory INSIDE docker container for exercise repos and project code
CONTAINER_CODE_ROOT=/code/catkin_ws/src/user_code

# Use nvidia docker runtime or not
GPU_DOCKER=false
ifeq ($(shell docker info --format '{{json .Runtimes}}' | grep -q 'nvidia' && echo true || echo false), true)
  # if detected as available, nvidia runtime will be used
  GPU_DOCKER=true
endif
# Set GPU_DOCKER to [true | false] to FORCE en/disable docker GPU access
# GPU_DOCKER=false

ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

REPO_VERSION=$(shell cat $(ROOT_DIR)/assets/VERSION)

# Detect the architecture of the machine
ARCH_TMP := $(shell uname -m)
ifeq ($(ARCH_TMP),x86_64)
  ARCH := amd64
else ifeq ($(ARCH_TMP),aarch64)
  ARCH := arm64v8
else ifeq ($(ARCH_TMP),arm64)
  ARCH := arm64v8
else
  $(error "Unsupported architecture $(ARCH_TMP)")
endif

ifeq ($(ARCH),arm64v8)
# Assume arm64 (jetson nano, jetson tx2, mac with M-chips) as jetson
  ON_JETSON=true
else
  ON_JETSON=false
endif

.PHONY: help pull build stop rm-container chown run exec rerun rerun-with-pull rerun-with-build show-params show-versioning

#: Show this help message
help:
	@echo "List of available make commands for vnc-docker:\n"
	@grep -B1 -E "^[a-zA-Z0-9_-]+\:([^\=]|$$)" Makefile \
	| grep -v -- -- \
	| sed 'N;s/\n/###/' \
	| sed -n 's/^#: \(.*\)###\(.*\):.*/\2###\1/p' \
	| column -t  -s '###'
	@echo ""

#: Pull the docker image from docker hub
pull:
	@docker pull $(FULL_IMAGE)

#: Build the image locally
build:
	@DOCKER_BUILDKIT=1 docker build -t $(FULL_IMAGE) --progress auto \
	--build-arg ON_JETSON=$(ON_JETSON) --build-arg ARCH=$(ARCH) \
	--build-arg DT_IMAGE_VERSION=$(REPO_VERSION) .

#: Stop the running main container
stop:
	@docker stop $(CONTAINER_NAME)

#: Remove the stopped main container
rm-container:
	@docker rm $(CONTAINER_NAME)

#: Change the ownership of the files in the root directory
chown:
	@sudo chown -R $$(id -u):$$(id -g) $(ROOT_DIR)


# ----- Launching vnc-docker container -----

# Set the volume options for the container
# Ref: https://github.com/duckietown/duckietown-shell-commands/blob/daffy/stack/stacks/duckietown/duckiebot.yaml
VOLUME_OPTIONS= -v $(ROOT_DIR)/user_code_mount_dir:$(CONTAINER_CODE_ROOT)
ifeq ($(ON_JETSON),true)
VOLUME_OPTIONS+= -v /data:/data -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket
endif

# Set the display options for the container
ifeq ($(ON_JETSON),false)
XSOCK:=/tmp/.X11-unix
XAUTH:=/tmp/.docker.xauth
DISPLAY_ENV:=$(DISPLAY)
ifndef DISPLAY_ENV
DISPLAY_ENV:=:0
$(info $(shell tput setaf 3)WARNING: DISPLAY environment variable not set in this terminal, using $(DISPLAY_ENV). \
This could lead to GPU not being detected or display issues.$(shell tput sgr0))
endif
DISPLAY_OPTIONS=-e XAUTHORITY=${XAUTH} \
	  -e DISPLAY=${DISPLAY_ENV} \
 	  -e QT_GRAPHICSSYSTEM=native \
 	  -e XDG_RUNTIME_DIR=/tmp \
	  -e TERM=${TERM} \
 	  --privileged \
 	  -v ${XSOCK}:${XSOCK}:rw \
 	  -v ${XAUTH}:${XAUTH}:rw
else
DISPLAY_OPTIONS=-e DISPLAY=:0
endif

# Set the GPU flag for the container
ifeq ($(GPU_DOCKER),true)
GPU_FLAG=--gpus all
else ifneq ($(GPU_DOCKER),false)
$(error GPU_DOCKER '$(GPU_DOCKER)' must be set to [true | false])
endif

# Set the options to run the container
DOCKER_RUN_OPTIONS= -it -d --net host --name $(CONTAINER_NAME) $(VOLUME_OPTIONS) $(DISPLAY_OPTIONS) $(GPU_FLAG) --restart unless-stopped

# Set the command to run in the container
ifeq ($(ON_JETSON),false)
DOCKER_CMD:=bash -c "while :; do sleep 10000; done;"
endif

IMAGE_VERSION=$(shell docker inspect -f '{{ index .Config.Labels "dt_image_version" }}' $(CONTAINER_NAME) 2>/dev/null || echo "? (container not found or still creating)")

define warn-mismatch
	@if [ "$(IMAGE_VERSION)" = " ? (container not found or still creating)" ]; then \
		.; \
	elif [ "$(REPO_VERSION)" != "$(IMAGE_VERSION)" ]; then \
		echo "#$(shell tput setaf 3) [WARNING] Version mismatch between vnc-docker repository and docker image. Consider either:$(shell tput sgr0)"; \
		echo "#$(shell tput setaf 3)           1) updating the image: rebuild (->make build) or pull (->make pull) the image (and then ->make rerun), $(shell tput sgr0)"; \
		echo "#$(shell tput setaf 3)        or 2) updating the repo : pull the vnc-docker repository (->git pull).$(shell tput sgr0)"; \
		echo "#"; \
	fi
endef

#: Show params used in this Makefile
show-params:
	#
	# ---------- Makefile params (BEGINNING) ----------
	#
	#  REPO_VERSION:   $(REPO_VERSION)
	#  CONTAINER_NAME: $(CONTAINER_NAME)
	#  FULL_IMAGE:     $(FULL_IMAGE)
	#  IMAGE_VERSION:  $(IMAGE_VERSION)
	#  DOCKER_CMD:     $(DOCKER_CMD)
	#  ARCH:           $(ARCH)
	#  ON_JETSON:      $(ON_JETSON)
	#  GPU_DOCKER:     $(GPU_DOCKER)
	#  [Do not override below values manually]
	#  DOCKER_RUN_OPTIONS: $(DOCKER_RUN_OPTIONS)
	#
	# ---------- Makefile params (END) ----------
	#
	@$(call warn-mismatch)
	
#: Show versioning information
show-versioning:
	#
	# ---------- Versioning information ----------
	#
	#  vnc-docker:   $(REPO_VERSION)
	#  docker image: $(IMAGE_VERSION)
	#
	@$(call warn-mismatch)
	#  User code packages:
	@for pkg in $$(find $(ROOT_DIR)/user_code_mount_dir/ -name package.xml | sort); do \
		echo "#    $$(grep -oPm1 "(?<=<name>)[^<]+" $$pkg;)": "$$(grep -oPm1 "(?<=<version>)[^<]+" $$pkg;)"; \
	done
	#

#: Run the container
run: show-params
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --filter "status=exited" --format "{{.Names}}" | \
	grep -q $(CONTAINER_NAME) && echo "Starting existing container $(CONTAINER_NAME)" && \
	docker start $(CONTAINER_NAME) || \
	(echo "Running a new container $(CONTAINER_NAME)" && \
	docker run $(DOCKER_RUN_OPTIONS) $(FULL_IMAGE) $(DOCKER_CMD)) || \
	(docker ps -a --filter "name=$(CONTAINER_NAME)" --filter "status=running" --format "{{.Names}}" | \
	grep -q $(CONTAINER_NAME) && echo "Container $(CONTAINER_NAME) was already running. Stop it first with 'make stop', \
	and remove it with 'make rm-container' to be able to start a fresh new container (or run 'make rerun').")

#: Attach a shell to the running container
exec:
	@xhost +local: > /dev/null 2>&1 || true
	@docker exec -it $(CONTAINER_NAME) bash

#: Remove the running container and run a fresh container
rerun:
	@docker rm -f $(CONTAINER_NAME) || true
	@$(MAKE) run

#: Remove the running container, pull the latest image, and run a fresh container
rerun-with-pull:
	@docker rm -f $(CONTAINER_NAME)
	@$(MAKE) pull && $(MAKE) run

#: Remove the running container, rebuild the image, and run a fresh container
rerun-with-build:
	@docker rm -f $(CONTAINER_NAME)
	@$(MAKE) build && $(MAKE) run
