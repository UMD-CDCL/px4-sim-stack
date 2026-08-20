# shellcheck shell=bash
# Every fleet airframe expands, with the links and sensors the flight code reads
#
# The airframes are nested merge-includes: chimera_v2 and chimera_v3 merge
# chimera_common, which merges x500, the gimbal and the LW20. libsdformat
# resolves that or it does not, and Gazebo's failure is silent: PX4 asks for the
# model, Gazebo declines, and the vehicle never appears.

sim_image=px4simstack/sim:${PX4_REF:-v1.17.0}
sdf_errors=$(mktemp)
expansion=$(mktemp)

if ! docker image inspect "$sim_image" >/dev/null 2>&1; then
	fail "$sim_image is not built. Run ./px4sim build sim"
	return 0
fi

expand_airframe() {
	docker run --rm --entrypoint bash \
		-v ./src/PX4-Autopilot:/px4:ro -v ./modules/sim/scenes:/scenes:ro \
		"$sim_image" -c '
			set -e
			merged=/tmp/merged/models
			mkdir -p $merged/rendered
			find /px4/Tools/simulation/gz/models -mindepth 1 -maxdepth 1 -type d \
				-exec ln -sfn -t $merged {} +
			find /scenes/models -mindepth 1 -maxdepth 1 -type d -exec ln -sfn -t $merged {} +
			export GZ_SIM_RESOURCE_PATH=$merged SDF_PATH=$merged
			# Every placeholder the template actually holds, whatever it is.
			# A written-out list goes stale the moment the model gains a
			# variable: the model gained ${UAS_NUM} for the per-framing camera
			# topics, the list did not, and gz refused the file with the
			# placeholder still in it.
			template=/scenes/models/'"$1"'/model.sdf
			names=$(grep -o "\${[A-Z_][A-Z0-9_]*}" $template | sort -u)
			for name in $names; do
				bare=${name#\$\{}; bare=${bare%\}}
				case "$bare" in
				*_RAD) value=0.5 ;;
				UAS_NUM) value=11 ;;
				*) value=x ;;
				esac
				export "$bare=$value"
			done
			envsubst "$(echo $names)" < $template > $merged/rendered/model.sdf
			printf "<?xml version=\"1.0\"?><model><name>rendered</name><version>1.0</version><sdf version=\"1.9\">model.sdf</sdf></model>" \
				> $merged/rendered/model.config
			gz sdf -p $merged/rendered/model.sdf 2>/tmp/err
			grep "^Error" /tmp/err >&2 || true
		' 2>"$sdf_errors" >"$expansion"
}

has_element() { grep -q "<$1 name='$2'" "$expansion" && echo yes || echo no; }

for n in $(fleet_numbers); do
	model=$(model_of "$n")
	expand_airframe "$model" || true

	if [ ! -s "$expansion" ] || grep -q '^Error' "$sdf_errors"; then
		fail "uas$n ($model) expands"
		note "$(head -3 "$sdf_errors")"
		continue
	fi
	pass "uas$n ($model) expands"

	# The names the rest of the stack reads. camera_link is the gimbal end every
	# camera hangs from, and gimbal_lidar is what reaches MAVROS as id 1.
	missing=""
	for want in camera_link gimbal_camera_link thermal_camera_link; do
		[ "$(has_element link "$want")" = yes ] || missing="$missing $want"
	done
	for want in gimbal_camera thermal_camera gimbal_lidar navsat_sensor imu_sensor; do
		[ "$(has_element sensor "$want")" = yes ] || missing="$missing $want"
	done
	if [ -n "$missing" ]; then fail "uas$n carries every named link and sensor"; note "missing:$missing"
	else pass "uas$n carries every named link and sensor"; fi

	down=$(has_element sensor pilot_camera)
	case "$(mark_of "$n")" in
	v3) expect_eq "uas$n is a v3 and carries the fixed down camera" yes "$down" ;;
	v2) expect_eq "uas$n is a v2 and has no separate down camera"   no  "$down" ;;
	esac
done

rm -f "$sdf_errors" "$expansion"
