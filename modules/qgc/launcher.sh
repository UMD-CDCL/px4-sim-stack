#!/usr/bin/env bash
set -euo pipefail

VERSION=5.1.4
IMAGE=${QGC_IMAGE:-px4simstack/qgc:chimera-5.1.4}

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Image $IMAGE is missing. Build it with ./px4sim build qgc." >&2
    exit 1
fi

# X11: hand over the display socket plus a cookie for it. The GDM cookie lives
# outside $HOME and is not readable from inside the container, so merge a
# wildcard-host copy into a throwaway file that is.
XAUTH=$(mktemp /tmp/.qgc-xauth-XXXXXX)
trap 'rm -f "$XAUTH"' EXIT
xauth nlist "${DISPLAY:-:0}" 2>/dev/null \
    | sed -e 's/^..../ffff/' \
    | xauth -f "$XAUTH" nmerge - 2>/dev/null || true
chmod 644 "$XAUTH"

ARGS=(
    --rm
    --user "$(id -u):$(id -g)"
    --workdir "$HOME"
    -e "HOME=$HOME"
    -e "XDG_CONFIG_HOME=$HOME/.config"
    -e "XDG_CACHE_HOME=$HOME/.cache"
    --name "qgc-${VERSION}-$$"
    # Host networking keeps every address in the settings file literally true:
    # the UDP links, the MAVLink forward, and the local map tile server.
    --network host
    --ipc host
    -e "DISPLAY=${DISPLAY:-:0}"
    -e XAUTHORITY=/tmp/.qgc-xauth
    -e QT_X11_NO_MITSHM=1
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v "$XAUTH:/tmp/.qgc-xauth:ro"
    -v "$HOME:$HOME:rw"
    # Serial radios, autopilots and joysticks are hotplugged, so bind /dev
    # wholesale rather than naming devices that may not exist at launch.
    # udev's database goes with it so Qt can enumerate them.
    -v /dev:/dev
    -v /run/udev:/run/udev:ro
    # The /dev bind makes hotplugged nodes visible, but Docker's device cgroup
    # still blocks access unless their character-device majors are allowed.
    # 166 = ttyACM, 188 = ttyUSB, 189 = raw USB (/dev/bus/usb).
    # 13 = Linux input devices (joysticks/gamepads and event nodes).
    --device-cgroup-rule "c 166:* rwm"
    --device-cgroup-rule "c 188:* rwm"
    --device-cgroup-rule "c 189:* rwm"
    --device-cgroup-rule "c 13:* rwm"
    --device /dev/dri
)

for grp in video dialout plugdev input; do
    gid=$(getent group "$grp" | cut -d: -f3)
    [[ -n "$gid" ]] && ARGS+=( --group-add "$gid" )
done

for directory in /media /mnt /run/media; do
    [[ -d "$directory" ]] && ARGS+=( -v "$directory:$directory:rw" )
done

# NVIDIA GL. Without this the container has no NVIDIA userspace driver, GLX
# falls back to software and the map crawls. DRIVER_CAPABILITIES must include
# graphics; the container toolkit's default of "utility" ships no GL at all.
DOCKER_RUNTIMES=$(docker info --format '{{.Runtimes}}' 2>/dev/null || true)
if [[ $DOCKER_RUNTIMES == *nvidia* ]] && command -v nvidia-smi >/dev/null 2>&1; then
    ARGS+=( --gpus all -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility,video,display )
fi

# Alert tones.
PULSE_SOCK="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/pulse/native"
if [[ -S "$PULSE_SOCK" ]]; then
    ARGS+=( -v "$PULSE_SOCK:/tmp/pulse-native:rw" -e PULSE_SERVER=unix:/tmp/pulse-native )
fi

# Spoken alerts. QGroundControl speaks through speech-dispatcher, a separate
# daemon from PulseAudio, so it needs its socket handed over separately.
SPEECH_SOCK="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/speech-dispatcher/speechd.sock"
if [[ -S "$SPEECH_SOCK" ]]; then
    ARGS+=( -v "$SPEECH_SOCK:/tmp/speechd.sock:rw" -e SPEECHD_ADDRESS=unix_socket:/tmp/speechd.sock )
fi

docker run "${ARGS[@]}" "$IMAGE" "$@"
