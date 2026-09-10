#!/usr/bin/env bash
# Check that the host can run the stack, then fill in the host-specific values
# in .env. This script changes nothing except .env.
set -uo pipefail

cd "$(dirname "$0")/.."

RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; OFF=$'\033[0m'
fail=0
warn=0

ok()   { printf '  %sok%s    %s\n'   "$GRN" "$OFF" "$1"; }
bad()  { printf '  %sfail%s  %s\n'   "$RED" "$OFF" "$1"; fail=$((fail+1)); }
note() { printf '  %swarn%s  %s\n'   "$YEL" "$OFF" "$1"; warn=$((warn+1)); }

echo "px4-sim-stack preflight"
echo ""

# What ./px4sim doctor selected, with the fleet profiles added. The checks
# below ask which world that is.
selected=",${COMPOSE_PROFILES:-},"
sim_selected()  { case "$selected" in *,sim,*) return 0 ;; esac; return 1; }
# The fleet numbers and whether they are the simulated ones. fleet.sh holds
# that rule for ./px4sim, and this reads the same answer rather than a second
# copy of the arithmetic.
# shellcheck disable=SC1091
. ./scripts/fleet.sh

# ---------------------------------------------------------------- NVIDIA driver
# The GPU and its driver decide which DeepStream release this machine can run,
# and a DeepStream release brings its Ubuntu and so its ROS 2 distribution with
# it. scripts/ds-select.sh holds that table; this only reports what it said.
#
# Neither bound degrades. A driver below the release fails to initialize CUDA
# and the container stops at its first call. A GPU newer than the release's
# TensorRT is worse: the pipeline runs, the streams decode, no engine is ever
# built and nothing is ever detected.
if command -v nvidia-smi >/dev/null 2>&1; then
	drv=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)
	gpu=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)
	vram=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader | head -1)
	cap=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1)
	ok "GPU: $gpu ($vram), driver $drv, compute capability ${cap:-unknown}"
else
	bad "nvidia-smi not found. Install the NVIDIA driver."
fi

# ------------------------------------------------------------- video engines
# The compute cores and the video engines are separate silicon, and the
# laptop parts (the T500, the MX class) ship with NVENC and NVDEC fused off
# while CUDA works untouched. Nothing fails outright on such a machine: every
# pipeline in the stack probes at run time and falls to software -- the sim
# encodes with x265enc, the companion decodes with avdec and previews with
# jpegenc, and TensorRT inference never used the engines at all. But each
# fallback announces itself one container log at a time, so say here, once,
# which way this machine will go.
#
# The nvcodec plugin asks the driver which engines exist and registers one
# element per codec it finds, so a fresh registry is the hardware answering.
# The session registry is not consulted: it can predate a driver change.
if command -v nvidia-smi >/dev/null 2>&1; then
	if command -v gst-inspect-1.0 >/dev/null 2>&1 &&
	   gst-inspect-1.0 nvcodec >/dev/null 2>&1; then
		registry=$(mktemp)
		nvcodec=$(GST_REGISTRY="$registry" gst-inspect-1.0 nvcodec 2>/dev/null)
		rm -f "$registry"
		enc=no; dec=no
		printf '%s' "$nvcodec" | grep -qE 'nv[a-z0-9]*h26[45]enc' && enc=yes
		printf '%s' "$nvcodec" | grep -qE 'nv[a-z0-9]*h26[45]dec' && dec=yes
		if [ "$enc" = yes ] && [ "$dec" = yes ]; then
			ok "GPU video engines: NVENC and NVDEC present"
		else
			note "GPU video engines: NVENC $enc, NVDEC $dec. Video falls back to
        software where an engine is missing: the sim encodes with x265enc, the
        companion decodes on the CPU. Detection stays on the GPU either way."
		fi
	elif command -v ffmpeg >/dev/null 2>&1 &&
	     ffmpeg -hide_banner -encoders 2>/dev/null | grep -q h264_nvenc; then
		# No nvcodec plugin on this host, so ask NVENC itself by opening one
		# session. Nothing on the host answers for NVDEC; the containers
		# probe it at run time.
		if ffmpeg -hide_banner -v error -f lavfi -i testsrc=duration=0.1:size=320x240 \
		          -frames:v 1 -c:v h264_nvenc -f null - >/dev/null 2>&1; then
			ok "GPU video engines: NVENC present (NVDEC not checked here)"
		else
			note "GPU video engines: no NVENC. Video falls back to software where
        an engine is missing: the sim encodes with x265enc, the companion
        decodes on the CPU. Detection stays on the GPU either way."
		fi
	else
		note "GPU video engines: nothing here can check (no gstreamer nvcodec
        plugin, no ffmpeg with nvenc). The containers probe at run time and
        fall back to software encoders and decoders where an engine is missing."
	fi
fi

# The release, and why. ds-select.sh writes its complaints to stderr, so a
# pinned release this machine cannot run is reported here as a failure rather
# than as a line nobody reads.
ds_complaint=$(./scripts/ds-select.sh --explain 2>&1 >/dev/null)
ds_choice=$(./scripts/ds-select.sh --explain 2>/dev/null)
if [ -z "$ds_choice" ]; then
	bad "no DeepStream release could be chosen"
	printf '%s\n' "$ds_complaint" | sed 's/^ds-select: //; s/^/        /'
elif [ -n "$ds_complaint" ]; then
	bad "$ds_choice"
	printf '%s\n' "$ds_complaint" | sed 's/^ds-select: //; s/^/        /'
else
	ok "$ds_choice"
fi

# ------------------------------------------------------------------- Docker
# Under `sudo ./px4sim doctor` this process is root, and root reaches the
# daemon whatever the operator's groups hold. The aircraft's boot unit runs as
# the operator (chimera-deploy remote/onboard.service), so it is that account
# that has to be in the group. Ask about it by name.
if [ -n "${SUDO_USER:-}" ] && ! id -nG "$SUDO_USER" 2>/dev/null | grep -qw docker; then
	note "docker works here under sudo only: $SUDO_USER is not in group docker.
        onboard.service runs as that user, so it cannot start this stack.
        The operator fixes it once, and then logs in again:
        sudo usermod -aG docker $SUDO_USER"
elif docker version >/dev/null 2>&1; then
	ok "docker $(docker version --format '{{.Server.Version}}') reachable without sudo"
elif id -nG | grep -qw docker; then
	bad "cannot talk to the docker daemon, and you are in group docker. Is it running?
        sudo systemctl start docker"
else
	bad "docker needs sudo here: $USER is not in group docker. Fix it once, then log in again:
        sudo usermod -aG docker $USER"
fi

if docker compose version >/dev/null 2>&1; then
	ok "docker compose $(docker compose version --short)"
else
	bad "docker compose v2 not found."
fi

# A build under sudo leaves root's files in this user's ~/.docker. The same
# build without sudo, which is what a machine with the operator in group
# docker runs, then cannot take the buildx lock. `docker compose build` prints
# the reason and still exits 0, so the build appears to succeed and the image
# is quietly the old one.
docker_root_owned=$(find "${HOME}/.docker" ! -user "$(id -un)" 2>/dev/null | head -3)
if [ -n "$docker_root_owned" ]; then
	bad "another user owns files in ${HOME}/.docker, so a build here cannot take
        the buildx lock. It prints one line and exits 0, and the image stays
        as it was. $(echo "$docker_root_owned" | tr '\n' ' ')
        sudo chown -R $(id -un):$(id -gn) ${HOME}/.docker"
else
	ok "${HOME}/.docker belongs to $(id -un), so a build can take the buildx lock"
fi

# ------------------------------------------------------- NVIDIA container runtime
if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
	ok "nvidia container runtime registered"
else
	bad "nvidia runtime missing. Install nvidia-container-toolkit, then run:
        sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker"
fi

# --------------------------------------------------------------------- disk
avail=$(df -BG --output=avail . | tail -1 | tr -dc '0-9')
if [ "${avail:-0}" -lt 80 ]; then
	note "only ${avail}G free here. A full build needs about 80G."
else
	ok "${avail}G free"
fi

# -------------------------------------------------------------------- memory
mem=$(free -g | awk '/^Mem:/{print $2}')
cpus=$(nproc)
if [ "$mem" -lt 16 ]; then
	note "${mem}G RAM. The PX4 build and Gazebo together want 16G or more."
else
	ok "${mem}G RAM, ${cpus} cores"
fi

# ---------------------------------------------------------------------- .env
if [ ! -f .env ]; then
	cp .env.example .env
	echo ""
	echo "  Created .env from .env.example."
fi

# The operator's ids, also when doctor runs under sudo to reach docker.
host_uid=${SUDO_UID:-$(id -u)}
host_gid=${SUDO_GID:-$(id -g)}
sed -i "s|^HOST_UID=.*|HOST_UID=$host_uid|" .env
sed -i "s|^HOST_GID=.*|HOST_GID=$host_gid|" .env
# One host value in .env. Append rather than edit where the line is absent,
# because an .env copied from an older example does not have it yet.
set_env_key() { # name value
	if grep -q "^$1=" .env; then
		sed -i "s|^$1=.*|$1=$2|" .env
	else
		printf '%s=%s\n' "$1" "$2" >> .env
	fi
}
# The group that owns /dev/input/event*. The qgc container joins it so
# QGroundControl can read a joystick.
input_gid=$(getent group input | cut -d: -f3 || true)
[ -n "${input_gid:-}" ] && set_env_key INPUT_GID "$input_gid"
# The group that owns /dev/dri/renderD128 on a Jetson. The aircraft container
# joins it beside video and dialout.
render_gid=$(getent group render | cut -d: -f3 || true)
[ -n "${render_gid:-}" ] && set_env_key RENDER_GID "$render_gid"
# DISPLAY stays out of .env. The containers take it from the session that
# starts them, because a value in the file goes stale on another machine.
sed -i "/^DISPLAY=/d" .env
[ -n "${SUDO_UID:-}" ] && chown "$host_uid:$host_gid" .env
ok ".env host values set (HOST_UID=$host_uid HOST_GID=$host_gid)"

# .env.example is the list of every line the stack reads. An .env copied from
# an older one is missing whatever was added since, and a missing line is not
# an empty line: the reader falls back to a default written into compose.yaml
# or an entrypoint, so the machine runs, and the knob that would have fixed it
# is one the operator cannot see. Name the names. Which of them this host
# wants is a decision, so this is a warning and not a failure.
env_names() { # file -- every name the file mentions, set or commented out
	sed -n 's/^[[:space:]]*#\?[[:space:]]*\([A-Za-z_][A-Za-z_0-9]*\)=.*/\1/p' "$1" |
		sort -u
}
# ONBOARD_LENS_DEVICE names the aircraft's zoom lens. A ground station and a
# simulator have no lens, so that name is not one their .env lacks.
wanted_names() { if aircraft_selected; then cat; else grep -v '^ONBOARD_LENS_DEVICE$'; fi; }
missing=$(comm -23 <(env_names .env.example | wanted_names) <(env_names .env) | paste -sd' ' -)
if [ -n "$missing" ]; then
	note ".env does not mention: $missing
        These were added to .env.example after this .env was made from it.
        Each one falls back to a built-in default, so nothing fails and
        nothing says so. Copy the lines you want across."
else
	ok ".env mentions every name .env.example does"
fi

# ---------------------------------------------------------------------- X11
# Only a service that mounts the X socket wants a display, and x11-allow.sh
# reads the compose file for that answer. A missing display is never a
# failure: Gazebo runs headless with GZ_GUI=0, the ground station starts
# without its USPI window, and the aircraft has no display at all.
if ./scripts/x11-allow.sh --needed; then
	if [ -n "${DISPLAY:-}" ]; then
		ok "DISPLAY=$DISPLAY"
	else
		note "DISPLAY is empty. A selected service mounts the X socket, so its windows cannot open."
	fi
	if command -v xauth >/dev/null 2>&1; then
		ok "xauth present"
	else
		note "xauth not found, so no X11 cookie can be written. Run: sudo apt install xauth"
	fi
	if [ "${XDG_SESSION_TYPE:-}" = "wayland" ]; then
		note "Wayland session. XWayland works, but GPU rendering can fall back to
        software. An X11 session is the tested path."
	fi
else
	ok "no selected service draws on X, so DISPLAY is not checked"
fi

# ----------------------------------------------------------------- the world
# UAS_BASE and COMPOSE_PROFILES have to describe one world, and the real
# profiles need their numbers. A wrong number here is a station that hears
# nothing and says nothing.
if real_profiles_selected && [ "$FLEET_IS_SIMULATED" = true ]; then
	bad "COMPOSE_PROFILES selects the real ground or the aircraft and UAS_BASE is $UAS_BASE. Set UAS_BASE=0 in .env."
elif ! real_profiles_selected && [ "$FLEET_IS_SIMULATED" = false ]; then
	bad "UAS_BASE=$UAS_BASE numbers the real fleet and COMPOSE_PROFILES selects no real profile. Set COMPOSE_PROFILES=ground or aircraft in .env."
else
	ok "UAS_BASE=$UAS_BASE and profiles '${COMPOSE_PROFILES:-}' describe one world"
fi
case "$selected" in
*,ground,*)
	[ "${GROUND_DOMAIN:-60}" = 60 ] \
		|| note "the fielded ground station is ROS domain 60 and GROUND_DOMAIN is ${GROUND_DOMAIN}. Native ROS on this machine is on 60."
	;;
*,aircraft,*)
	if [ -n "${UAS_NUM:-}" ]; then
		ok "UAS_NUM=$UAS_NUM from the login environment"
	else
		bad "UAS_NUM is not set. chimera-deploy/deploy.sh writes it into /etc/environment. Log in again."
	fi
	if [ -z "${ONBOARD_LENS_DEVICE:-}" ]; then
		note "ONBOARD_LENS_DEVICE is unset, so the container gets /dev/null as its lens. A v3 needs it: see .env.example."
	elif [ -e "${ONBOARD_LENS_DEVICE}" ]; then
		ok "SCF4 zoom lens at ${ONBOARD_LENS_DEVICE} -> $(readlink -f "${ONBOARD_LENS_DEVICE}")"
	else
		bad "ONBOARD_LENS_DEVICE=${ONBOARD_LENS_DEVICE} does not exist. ls /dev/serial/by-id/"
	fi
	;;
esac

# --------------------------------------------------------------- host ports
# Every published port is a claim on the host, and this stack is not the only
# thing that can hold one. This runs after the .env block above, because
# compose cannot resolve the file's variables before the file exists.
#
# A taken port is not a small failure. `docker compose up` stops at the
# container that wanted it and abandons the rest of the start, so the services
# after it never run and what the operator sees is whatever those were
# carrying, never the port. The container that lost the bind is worse: it keeps
# its place in the project, and the next start brings it up with no network
# endpoint at all. That one reads as healthy everywhere except in what it
# serves. `./px4sim status` names that state once it exists; this is where it
# is caught before it does.
# Where the stack can move a host port itself, name the variable that does it
# rather than leave the operator to go and find it. An overridable published
# port is written `${VAR:-default}` in compose.yaml, so the file already says
# which ports carry a knob and what each is called. Resolve each against the
# environment, because the number to match against is the one that variable
# publishes now, not the default it fell back from.
overrides=$(sed -n 's/^[[:space:]]*-[[:space:]]*"\${\([A-Za-z_][A-Za-z_0-9]*\):-\([0-9]\{1,5\}\)}:.*/\1 \2/p' compose.yaml |
	while read -r name default; do
		printf '%s %s\n' "${!name:-$default}" "$name"
	done)

config=$(docker compose config --format json 2>/dev/null)
# The name compose labels every container of this stack with. Both port checks
# below tell one of ours from a stranger's by it, so it is read once, from the
# rendered file rather than from a copy of the name.
project=""
if [ -n "$config" ] && command -v python3 >/dev/null 2>&1; then
	project=$(printf '%s' "$config" |
		python3 -c 'import json,sys; print(json.load(sys.stdin).get("name",""))')
fi
if ! command -v ss >/dev/null 2>&1 || ! command -v python3 >/dev/null 2>&1; then
	note "no ss or python3 here, so no host port was checked."
elif [ -z "$config" ]; then
	note "compose could not read its own file, so no host port was checked.
        Say why:  ./px4sim check"
else
	# What this stack already publishes is its own, not a conflict with itself.
	# docker prints a run of ports as one range, 14561-14569->14561-14569/udp,
	# so each mapping is spread back out. Read one number where a range is
	# meant and this stack reports its own ground station as a stranger.
	mine=$(docker ps --filter "label=com.docker.compose.project=$project" \
	                 --format '{{.Ports}}' 2>/dev/null |
		tr ',' '\n' |
		awk -F'->' 'NF == 2 {
			split($1, address, ":"); span = address[length(address)]
			split($2, target, "/"); protocol = target[2]
			low = span; high = span
			if (split(span, ends, "-") == 2) { low = ends[1]; high = ends[2] }
			for (port = low + 0; port <= high + 0; port++) print port, protocol
		}' | sort -u)

	# Every port the next start would publish, and the service that wants it.
	wanted=$(printf '%s' "$config" | python3 -c '
import json, sys

claims = {}
for service, spec in (json.load(sys.stdin).get("services") or {}).items():
    for port in spec.get("ports") or []:
        published = port.get("published")
        if published:
            claims.setdefault((int(published), port.get("protocol", "tcp")), service)
for (number, protocol), service in sorted(claims.items()):
    print(number, protocol, service)
')

	taken=""
	while read -r number protocol service; do
		[ -n "${number:-}" ] || continue
		printf '%s\n' "$mine" | grep -qx "$number $protocol" && continue
		if [ "$protocol" = udp ]; then flag=-lnu; else flag=-lnt; fi
		held=$(ss "$flag" -p 2>/dev/null |
			awk -v want=":$number\$" '$4 ~ want { print; exit }')
		[ -n "$held" ] || continue
		# ss names the holder only when it belongs to this user or this is
		# root. An empty name is not an empty port, so say which it is.
		who=$(printf '%s' "$held" |
			sed -n 's/.*users:((\([^,]*\),pid=\([0-9]*\).*/\1 pid \2/p' | tr -d '"')
		knob=$(printf '%s\n' "$overrides" |
			awk -v want="$number" '$1 == want { print $2; exit }')
		taken="$taken
        $number/$protocol, wanted by $service, held by ${who:-another user: sudo ss -lnp}${knob:+
            Move the stack instead: set $knob in .env}"
	done <<EOF
$wanted
EOF

	if [ -n "$taken" ]; then
		bad "host ports already taken. The start stops at the first of these and
        leaves every service after it unstarted:$taken"
	else
		ok "$(printf '%s\n' "$wanted" | grep -c .) host ports free"
	fi
fi

# ----------------------------------------------------------- the simnet subnet
# A bridge on the range the fleet flies shadows every route into it, .61
# included, and nothing reports it: the drone is simply not reachable. Only
# a selected service on simnet creates the bridge, so only then is it a
# fault. The ground and aircraft profiles are on the host network.
if [ -n "$config" ] && printf '%s' "$config" | python3 -c '
import json, sys
services = json.load(sys.stdin).get("services") or {}
sys.exit(0 if any("simnet" in (s.get("networks") or {}) for s in services.values()) else 1)'; then
	subnet="${SIMNET_PREFIX:-10.200.142}.0/24"
	shadowed=$(ip -4 route show 2>/dev/null | awk '$1 ~ /\// && $0 !~ /docker|br-|veth/ {print $1}' |
		python3 -c '
import ipaddress, sys
simnet = ipaddress.ip_network(sys.argv[1])
print(" ".join(r for r in sys.stdin.read().split() if ipaddress.ip_network(r, strict=False).overlaps(simnet)))' "$subnet")
	if [ -n "$shadowed" ]; then
		bad "simnet $subnet overlaps this host's route $shadowed. Set SIMNET_PREFIX=172.28.0 in .env."
	else
		ok "simnet $subnet overlaps no host route"
	fi
fi

# ------------------------------------------------------ host network ports
# A service on the host network publishes nothing, so the port check above
# sees nothing. MAVROS binds 14402/udp and the Foxglove bridge 8765/tcp. A
# container of this stack that already holds them is not a conflict.
if real_profiles_selected && command -v ss >/dev/null 2>&1; then
	ours=""
	[ -n "$project" ] &&
		ours=$(docker ps --filter "label=com.docker.compose.project=$project" \
		                 --format '{{.Names}} {{.Networks}}' 2>/dev/null | awk '$2 == "host"')
	while read -r number protocol what; do
		if [ "$protocol" = udp ]; then flag=-lnu; else flag=-lnt; fi
		held=$(ss "$flag" -p 2>/dev/null | awk -v want=":$number\$" '$4 ~ want { print; exit }')
		if [ -z "$held" ]; then
			ok "$number/$protocol free for $what"
		elif [ -n "$ours" ]; then
			ok "$number/$protocol held by this stack's own container ($what)"
		else
			bad "$number/$protocol is held on this host, and $what binds it on the host network:
        $held"
		fi
	done <<-EOF
		14402 udp MAVROS
		8765 tcp the Foxglove bridge
	EOF
fi

# --------------------------------------------------------------- source trees
if sim_selected; then
	if [ -d src/PX4-Autopilot ]; then
		ok "src/PX4-Autopilot present"
	else
		note "src/PX4-Autopilot missing. Run: ./px4sim setup"
	fi
fi
# The flight code. The onboard and offboard images build it from here, and a
# missing checkout fails the build rather than the run.
ws=${ROS2_WS_DIR:-../ros2_ws}
if [ -d "$ws/src/5g_drone" ]; then
	ok "$ws/src/5g_drone present"
else
	note "$ws/src/5g_drone missing. The onboard and offboard images build it.
        Check it out, or set ROS2_WS_DIR in .env."
fi
# The chimera-deploy checkout. compose passes it as a named build context for
# every ros-base build, MAVROS_PATCH=0 included, because docker resolves a
# named context before it reads the Dockerfile.
deploy=${CHIMERA_DEPLOY_DIR:-../chimera-deploy}
if [ ! -d "$deploy" ]; then
	note "$deploy is missing. ros-base cannot build without it:
        git clone git@github.com:UMD-UROC/chimera-deploy.git $deploy   (or set CHIMERA_DEPLOY_DIR)"
fi
# The MAVROS patch. ros-base builds it from chimera-deploy's submodules, and
# an empty submodule fails the build with a readable message. Say it earlier.
if [ "${MAVROS_PATCH:-1}" = 1 ]; then
	if [ -f "$deploy/submodules/mavros/mavros/package.xml" ] && [ -d "$deploy/submodules/angles/angles" ]; then
		ok "$deploy present, with the mavros and angles submodules"
	else
		note "$deploy/submodules/mavros or angles is empty. ros-base builds the PX4 v1.18
        MAVROS patch from them:  git -C $deploy submodule update --init submodules/mavros submodules/angles"
	fi
fi

# ------------------------------------------------------------- detector models
# The detector is the part of this stack that fails by producing nothing.
# nvinfer reads its artifacts from under ONBOARD_MODEL_DIR, and an empty
# directory costs no error anyone meets: the containers start, the video
# flows, the operator watches it, and no box is ever drawn. Every other way to
# learn this is downstream of a flight.
#
# Nothing about it is written down here. The flight code's own parameter files
# carry the names AND the directory, and the launch loads those in this order,
# each one beating the last -- so read the same files, in the same order, and
# take what the last one said.
models=${ONBOARD_MODEL_DIR:-./modules/onboard/models}

# Those files hold paths as the container sees them, where the tree is mounted
# at /models. This runs on the host, so put the mount back before opening
# anything. A path that is not under /models belongs to the image rather than
# to this mount, and is left as it is.
host_path() { # a path inside the container -> where it is on this host
	case ${1:-} in
		/models)   printf '%s' "$models" ;;
		/models/*) printf '%s/%s' "$models" "${1#/models/}" ;;
		*)         printf '%s' "${1:-}" ;;
	esac
}

model_name() { # key -- what the parameter files finally set it to
	local key=$1 value="" file found
	for file in \
		"$ws/src/5g_drone/config/param_files/onboard_common_params.yaml" \
		"$ws/src/5g_drone/config/param_files/onboard_container_params.yaml" \
		"$ws/src/5g_drone/config/param_files/sim/onboard_sim_params.yaml" \
		"$(host_path "${ONBOARD_PARAMS_FILE:-}")"
	do
		[ -n "$file" ] && [ -f "$file" ] || continue
		found=$(sed -n "s/^[[:space:]]*$key:[[:space:]]*[\"']\?\([^\"'#]*[^\"'# ]\).*/\1/p" \
			"$file" | tail -1)
		[ -n "$found" ] && value=$found
	done
	printf '%s' "$value"
}

# The log volumes bind to these directories. scripts/fleet.sh says why they
# are made here, and who owns them after a run under sudo.
make_bind_dirs logs logs/onboard logs/offboard logs/px4 logs/qgc 2>/dev/null ||
	bad "cannot create logs/onboard logs/offboard logs/px4 logs/qgc here."

if [ ! -d "$models" ]; then
	# Compose creates a missing bind-mount source as a root-owned directory,
	# and the container then cannot write the engine it builds. Make it now,
	# owned by whoever runs this.
	make_bind_dirs "$models" 2>/dev/null &&
		note "created $models. Put the detector artifacts in it.
        See modules/onboard/models/README.md." ||
		bad "$models does not exist and could not be created."
elif [ ! -w "$models" ]; then
	bad "$models is not writable by you. nvinfer builds the TensorRT engine
        beside the ONNX on the first run, so the detector needs to write here.
        A directory compose created for a missing mount is owned by root:
        sudo chown -R $(id -u):$(id -g) $models"
else
	# model.dir is a path inside the mount, not the mount. In 5g_drone's
	# perception_models tree it ends in local, the symlink fetch_models.py
	# points at the engine group this GPU can load -- so the same string
	# serves the Orin and either laptop, and each one resolves it elsewhere.
	said=$(model_name 'model\.dir')
	dir=$(host_path "$said")
	[ -n "$dir" ] || { dir=$models; said=/models; }
	if [ ! -d "$dir" ]; then
		note "the parameter files read the artifacts from $said, which is
        $dir here, and that is not a directory. In 5g_drone's
        perception_models tree the last element is a symlink to this
        machine's engine group, and only this machine can set it:
        ./scripts/fetch_models.py resolve --link"
	else
		wanted=""
		for key in 'model\.detector' 'model\.classifier'; do
			name=$(model_name "$key")
			[ -n "$name" ] || continue
			# Either will do: the ONNX nvinfer can build an engine from, or an
			# engine already built. An engine belongs to one GPU, one driver
			# and one TensorRT, so a directory holding one was filled by a
			# machine like this one.
			ls "$dir/$name.onnx" "$dir/$name".onnx_b*.engine >/dev/null 2>&1 ||
				wanted="$wanted $name"
		done
		if [ -n "$wanted" ]; then
			note "no detector artifacts in $dir for:$wanted
        The names and that directory both come from the parameter files under
        $ws. Nothing fails without them and nothing is ever detected. Fetch
        them:  ./scripts/fetch_models.py fetch --role onboard"
		elif [ -z "$(model_name 'model\.detector')" ]; then
			note "could not read model.detector from the parameter files under
        $ws, so no detector artifact was checked."
		else
			ok "detector artifacts present in $dir"
		fi
	fi
fi

# ----------------------------------------------------------------- aircraft
case "$selected" in
*,aircraft,*)
	for unit in rcam mavlink-router; do
		if systemctl is-active --quiet "$unit"; then
			ok "$unit.service active"
		else
			bad "$unit.service is not active. The container reads its cameras and its MAVLink from it:  sudo systemctl start $unit"
		fi
	done
	socks=$(for sock in /tmp/*ds_nv.sock; do [ -e "$sock" ] && basename "$sock" _nv.sock; done | paste -sd' ' -)
	if [ -n "$socks" ]; then
		ok "rcam sockets: $socks"
	else
		bad "no /tmp/*ds_nv.sock. rcam forks the cameras onto them at start:  journalctl -u rcam -n 30"
	fi
	if [ "$(date +%Y)" -ge 2020 ]; then
		ok "clock $(date -Is)"
	else
		note "clock says $(date +%Y). chrony has not stepped it. Logs and bags carry 1970 stamps until it does."
	fi
	command -v nvpmodel >/dev/null 2>&1 \
		&& note "power mode: $(nvpmodel -q 2>/dev/null | head -1). Measure detection at this mode before changing it."
	fetch=$ws/src/5g_drone/scripts/fetch_models.py
	if [ -x "$fetch" ]; then
		group=$("$fetch" resolve 2>/dev/null || true)
		if [ "$group" = orin ]; then
			ok "perception_models group: orin"
		else
			note "fetch_models.py resolve says '${group:-nothing}', not orin. /models/local may point at another machine's engines."
		fi
	fi
	;;
*,ground,*)
	for unit in lcam mavlink-router; do
		if systemctl is-active --quiet "$unit"; then
			ok "$unit.service active"
		else
			note "$unit.service is not active. The ground reads video and MAVLink from it."
		fi
	done
	;;
esac
# lcam and rcam have no API, so ./px4sim streams probes each mount with this.
if real_profiles_selected && ! command -v gst-discoverer-1.0 >/dev/null 2>&1; then
	note "gst-discoverer-1.0 not found, so ./px4sim streams cannot probe the RTSP mounts. Run: sudo apt install gstreamer1.0-plugins-base-apps"
fi

# The onboard and offboard images carry a copy of the flight code, taken when
# they were built. A tree edited after that leaves the image a launch file
# short of a node, and the stack says "executable not found" or names a launch
# file that is not there. That reads as a bug in the flight code, so say here
# that the image is behind the tree.
# What is pruned is what the image does not carry: the build products, the
# working copy's own directories, and the model tree, which is mounted rather
# than copied. Counting any of them makes a rebuild look due after opening a
# file in an editor or fetching an engine, and a warning that cries wolf is
# one nobody reads on the day the tree really did move.
newest=$(find "$ws/src" \( -name .git -o -name .vscode -o -name __pycache__ \
                          -o -name build -o -name install -o -name log \
                          -o -name perception_models \) -prune -o \
              -type f -printf '%T@\n' 2>/dev/null | sort -rn | head -1)
newest=${newest%%.*}
stale=""
ds_tag=$(./scripts/ds-select.sh --tag 2>/dev/null || echo 7.1)
for image in "onboard:${ds_tag}" "offboard:${ds_tag}"; do
	created=$(docker image inspect --format '{{.Created}}' "px4simstack/$image" 2>/dev/null) || continue
	built=$(date -d "$created" +%s 2>/dev/null) || continue
	[ -n "${newest:-}" ] && [ "$newest" -gt "$built" ] && stale="$stale ${image%%:*}"
done
if [ -n "$stale" ]; then
	note "The flight code in $ws/src is newer than the build inside:$stale
        ./px4sim start will rebuild it before starting the containers."
fi

echo ""
if [ "$fail" -gt 0 ]; then
	echo "${RED}$fail check(s) failed.${OFF} Fix them before you start the stack."
	exit 1
fi
# `setup` clones the PX4 and QGroundControl sources, which only the simulator
# builds. A real machine is one build and one start away from flying.
if [ "$FLEET_IS_SIMULATED" = true ]; then
	next="./px4sim setup, then ./px4sim start"
else
	next="./px4sim start"
fi
echo "${GRN}Ready.${OFF} $warn warning(s). Next: $next."
