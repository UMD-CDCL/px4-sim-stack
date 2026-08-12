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
# DeepStream 8.0 is validated against driver 570.133. DeepStream 9.0 needs
# 590.48. The driver sets the ceiling on which DeepStream release can run.
if command -v nvidia-smi >/dev/null 2>&1; then
	drv=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)
	gpu=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)
	vram=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader | head -1)
	ok "GPU: $gpu ($vram), driver $drv"
	# Which release the driver allows, and which one the stack will therefore
	# build. Reported rather than left to the reader, because the two used to
	# disagree silently whenever the repository moved to another machine.
	major=${drv%%.*}
	if [ "$major" -lt 570 ]; then
		bad "driver $drv is below 570.133. DeepStream 8.0 will not start."
	else
		ok "$(./scripts/ds-select.sh --explain)"
		ok "perception builds $(./scripts/ds-select.sh --image)"
	fi
else
	bad "nvidia-smi not found. Install the NVIDIA driver."
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
# DISPLAY stays out of .env. The containers take it from the session that
# starts them, because a value in the file goes stale on another machine.
sed -i "/^DISPLAY=/d" .env
ok ".env host values set (HOST_UID=$(id -u) HOST_GID=$(id -g))"

# --------------------------------------------------------------- source trees
for d in src/PX4-Autopilot src/ros2_ws; do
	if [ -d "$d" ]; then ok "$d present"; else note "$d missing. Run: make bootstrap"; fi
done

echo ""
if [ "$fail" -gt 0 ]; then
	echo "${RED}$fail check(s) failed.${OFF} Fix them before you run make up."
	exit 1
fi
echo "${GRN}Ready.${OFF} $warn warning(s). Next: make bootstrap, then make build, then make up."
