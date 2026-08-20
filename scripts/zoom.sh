# The framings a v3 gimbal lens reaches, and where each one puts the lens.
#
# Sourced by the simulator, which renders and crops the picture to a framing,
# by the companion, which writes the calibration and the lens parameters for
# it, and by ./px4sim. One table, so no two of them can disagree about what
# "mid" means.
#
# These numbers describe the SIMULATED lens only. The aircraft's framings come
# from its own calibration files in 5g_drone, which real calibration replaces
# later. Nothing here reaches a file the aircraft loads.

# Field of view of each framing, in degrees, at the widest of them. The
# simulated lens is 1x, 3x and 10x of the wide framing, so the magnifications
# are round numbers and the crop fraction of one framing inside the next is
# exactly one over the magnification.
#
#   wide    1x   f =  601.07 px   hfov 56.06   vfov 33.34   (at 640x360)
#   mid     3x   f = 1803.22 px   hfov 20.13   vfov 11.40
#   narrow 10x   f = 6010.73 px   hfov  6.09   vfov  3.43
#
# wide is measured: it is the only self-consistent one of the three aircraft
# calibration files (fx 601.11 at 640 wide gives 56.06 degrees).
UAS_ZOOM_PRESETS=${UAS_ZOOM_PRESETS:-"narrow=6.09 mid=20.13 wide=56.06"}

# Which framing each vehicle boots at, by its place in the fleet. A dash means
# the vehicle has no zoom.
UAS_ZOOM=${UAS_ZOOM:-"mid mid - -"}

# Where each framing puts the lens, in motor steps, zoom then focus. The
# emulated SCF4 controller moves these counters and the companion recalls these
# positions, so a framing is a position on both sides of the wire, as it is on
# the aircraft. Taken from uas1_params.yaml so the simulated lens is driven
# through the same numbers the aircraft is.
UAS_ZOOM_STEPS=${UAS_ZOOM_STEPS:-"narrow=29000,32380 mid=34500,32980 wide=40000,31140"}

# Travel of each axis in motor steps, low then high, and the step the homing
# datum sits at. The emulated controller puts its photo-interrupter at the
# datum and its mechanical stops just outside the travel, so homing lands where
# the companion's zoom.limits.* say it will.
UAS_ZOOM_TRAVEL=${UAS_ZOOM_TRAVEL:-"zoom=29000,40000 focus=29500,33500"}
UAS_ZOOM_DATUM=${UAS_ZOOM_DATUM:-"zoom=32000 focus=32000"}

# The emulated controller listens on this port plus the vehicle number, inside
# the sim container. The companion opens socket://sim:<port> in place of the
# USB serial device the aircraft has.
UAS_ZOOM_PORT_BASE=${UAS_ZOOM_PORT_BASE:-5900}

# One entry of a "key=value key=value" table. A key the table does not name
# answers with nothing, so a caller can tell absent from empty.
zoom_lookup() {
	local table=$1 wanted=$2 entry
	for entry in $table; do
		[ "${entry%%=*}" = "$wanted" ] && { echo "${entry#*=}"; return 0; }
	done
	return 1
}

# The field of view of one framing, in degrees. A name that is not a framing,
# or the dash that means a vehicle has no zoom, answers with nothing.
zoom_hfov_deg() { zoom_lookup "$UAS_ZOOM_PRESETS" "$1"; }

# Where one framing puts the lens: "<zoom steps>,<focus steps>".
zoom_steps() { zoom_lookup "$UAS_ZOOM_STEPS" "$1"; }

# The travel of one axis: "<low>,<high>". Axis names are zoom and focus.
zoom_travel() { zoom_lookup "$UAS_ZOOM_TRAVEL" "$1"; }

# The homing datum of one axis, in motor steps.
zoom_datum() { zoom_lookup "$UAS_ZOOM_DATUM" "$1"; }

# What one vehicle flies at, by its place in the fleet.
zoom_preset_of_slot() {
	local slot=$1 chosen
	# shellcheck disable=SC2206
	local presets=($UAS_ZOOM)
	chosen=${presets[$slot]:-${presets[0]:-}}
	[ "$chosen" = - ] && return 1
	echo "$chosen"
}

# Every framing, widest first. The widest is what the camera has to render,
# because every narrower one is reached by cropping or by a second camera
# pointed the same way.
zoom_presets_widest_first() {
	local entry
	for entry in $UAS_ZOOM_PRESETS; do
		printf '%s %s\n' "${entry#*=}" "${entry%%=*}"
	done | sort -g -r | awk '{ print $2 }'
}


# The port the emulated controller for one vehicle listens on.
zoom_port() { echo $((UAS_ZOOM_PORT_BASE + $1)); }
