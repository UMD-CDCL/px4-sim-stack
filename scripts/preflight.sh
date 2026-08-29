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
if docker version >/dev/null 2>&1; then
	ok "docker $(docker version --format '{{.Server.Version}}') reachable without sudo"
else
	bad "cannot talk to the docker daemon. Add yourself to the docker group."
fi

if docker compose version >/dev/null 2>&1; then
	ok "docker compose $(docker compose version --short)"
else
	bad "docker compose v2 not found."
fi

# ------------------------------------------------------- NVIDIA container runtime
if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
	ok "nvidia container runtime registered"
else
	bad "nvidia runtime missing. Install nvidia-container-toolkit, then run:
        sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker"
fi

# ---------------------------------------------------------------------- X11
if [ -z "${DISPLAY:-}" ]; then
	bad "DISPLAY is empty. Gazebo and QGroundControl need an X server."
else
	ok "DISPLAY=$DISPLAY"
fi
if [ "${XDG_SESSION_TYPE:-}" = "wayland" ]; then
	note "Wayland session. XWayland works, but GPU rendering can fall back to
        software. An X11 session is the tested path."
fi
if command -v xauth >/dev/null 2>&1; then
	ok "xauth present"
else
	bad "xauth not found. Run: sudo apt install xauth"
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

sed -i "s|^HOST_UID=.*|HOST_UID=$(id -u)|" .env
sed -i "s|^HOST_GID=.*|HOST_GID=$(id -g)|" .env
# The group that owns /dev/input/event*. The qgc container joins it so
# QGroundControl can read a joystick. Append rather than edit, because an .env
# copied from an older example does not have the line yet.
input_gid=$(getent group input | cut -d: -f3 || true)
if [ -n "${input_gid:-}" ]; then
	if grep -q '^INPUT_GID=' .env; then
		sed -i "s|^INPUT_GID=.*|INPUT_GID=$input_gid|" .env
	else
		echo "INPUT_GID=$input_gid" >> .env
	fi
fi
# DISPLAY stays out of .env. The containers take it from the session that
# starts them, because a value in the file goes stale on another machine.
sed -i "/^DISPLAY=/d" .env
ok ".env host values set (HOST_UID=$(id -u) HOST_GID=$(id -g))"

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
config=$(docker compose config --format json 2>/dev/null)
if ! command -v ss >/dev/null 2>&1 || ! command -v python3 >/dev/null 2>&1; then
	note "no ss or python3 here, so no host port was checked."
elif [ -z "$config" ]; then
	note "compose could not read its own file, so no host port was checked.
        Say why:  ./px4sim check"
else
	# What this stack already publishes is its own, not a conflict with itself.
	project=$(printf '%s' "$config" |
		python3 -c 'import json,sys; print(json.load(sys.stdin).get("name",""))')
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
		taken="$taken
        $number/$protocol, wanted by $service, held by ${who:-another user: sudo ss -lnp}"
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

# --------------------------------------------------------------- source trees
if [ -d src/PX4-Autopilot ]; then
	ok "src/PX4-Autopilot present"
else
	note "src/PX4-Autopilot missing. Run: ./px4sim setup"
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

# The onboard and offboard images carry a copy of the flight code, taken when
# they were built. A tree edited after that leaves the image a launch file
# short of a node, and the stack says "executable not found" or names a launch
# file that is not there. That reads as a bug in the flight code, so say here
# that the image is behind the tree.
newest=$(find "$ws/src" \( -name .git -o -name __pycache__ -o -name build \
                          -o -name install -o -name log \) -prune -o \
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
        Those containers run it as it was. Rebuild:  ./px4sim build$stale"
fi

echo ""
if [ "$fail" -gt 0 ]; then
	echo "${RED}$fail check(s) failed.${OFF} Fix them before you start the stack."
	exit 1
fi
echo "${GRN}Ready.${OFF} $warn warning(s). Next: ./px4sim setup, then ./px4sim build, then ./px4sim start."
