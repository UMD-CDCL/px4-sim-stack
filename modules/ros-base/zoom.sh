# The zoom presets a gimbal lens reaches, and which one a vehicle flies at.
# Sourced by the simulator, which crops the picture to the preset, and by the
# companion, which writes the calibration for it. One table, so the two cannot
# disagree about what "mid" means.

UAS_ZOOM_PRESETS=${UAS_ZOOM_PRESETS:-"narrow=27.80 mid=27.45 wide=56.06"}
UAS_ZOOM=${UAS_ZOOM:-"mid mid - -"}

# The field of view of one preset, in degrees. A name that is not a preset, or
# the dash that means a vehicle has no zoom, answers with nothing.
zoom_hfov_deg() {
	local wanted=$1 entry
	for entry in $UAS_ZOOM_PRESETS; do
		[ "${entry%%=*}" = "$wanted" ] && { echo "${entry#*=}"; return 0; }
	done
	return 1
}

# What one vehicle flies at, by its place in the fleet.
zoom_preset_of_slot() {
	local slot=$1 chosen
	# shellcheck disable=SC2206
	local presets=($UAS_ZOOM)
	chosen=${presets[$slot]:-${presets[0]:-}}
	[ "$chosen" = - ] && return 1
	echo "$chosen"
}
