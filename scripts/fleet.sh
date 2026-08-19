# The fleet, derived from UAS_FLEET. ./px4sim sources this, and so does
# verify/run.sh, so the two agree on every number by construction.
# docs/uas-contract.md is the specification.

# shellcheck disable=SC2206
FLEET=(${UAS_FLEET:-chimera_v3 chimera_v3 chimera_v2 chimera_v2})
UAS_COUNT=${#FLEET[@]}
# A simulated vehicle is its real counterpart plus ten, so vehicle 1 of the
# fleet is uas11. Keep this equal to UAS_BASE in modules/sim/entrypoint.sh.
UAS_BASE=${UAS_BASE:-10}
FIRST_UAS=$((UAS_BASE + 1))
LAST_UAS=$((UAS_BASE + UAS_COUNT))
SIMNET_PREFIX=${SIMNET_PREFIX:-10.200.142}
GROUND_DOMAIN=${GROUND_DOMAIN:-70}

fleet_numbers() { seq "$FIRST_UAS" "$LAST_UAS"; }

# The airframe of one vehicle, and what it serves. The mark decides the stream
# names: a v3 carries a gimbal camera and a down camera, a v2 carries the gimbal
# alone and serves it under the down camera's name. See section 3 of
# docs/uas-contract.md and modules/sim/scenes/models/<model>/streams.conf.
model_of() {
	local slot=$(( ${1:-0} - UAS_BASE - 1 ))
	[ "$slot" -ge 0 ] && echo "${FLEET[$slot]:-}"
	return 0
}
mark_of() {
	case "$(model_of "$1")" in
	*_v3) echo v3 ;;
	*_v2) echo v2 ;;
	esac
}
gimbal_stream() {
	case "$(mark_of "$1")" in
	v3) echo "rgb$1" ;;
	v2) echo "pilot$1" ;;
	esac
}
streams_of() {
	case "$(mark_of "$1")" in
	v3) echo "rgb$1 rgbl$1 pilot$1 pilotl$1 thermal$1 thermall$1" ;;
	v2) echo "pilot$1 pilotl$1 thermal$1 thermall$1" ;;
	esac
}

# Everything else a UAS number decides, as arithmetic. Joining strings reads
# correctly below ten and gives 145511 and 611 above it.
uas_address()       { echo "$SIMNET_PREFIX.2$1"; }
uas_domain()        { echo "$((60 + $1))"; }
uas_gcs_port()      { echo "$((14550 + $1))"; }
uas_tcp_port()      { echo "$((5750 + $1))"; }
uas_foxglove_port() { echo "$((8760 + $1))"; }
