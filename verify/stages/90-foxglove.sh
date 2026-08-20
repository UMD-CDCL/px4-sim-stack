# shellcheck shell=bash
# The operator's view: what Foxglove is offered, and whether the layout draws it
#
# A topic that carries data still reaches nobody if the bridge does not offer
# it or the layout does not name it, and a panel setting Foxglove rejects takes
# the whole panel's configuration with it without saying so. Everything here is
# read over the protocol Foxglove itself speaks, against the ground station,
# which is the bridge an operator connects to.

layout=${FOXGLOVE_LAYOUT:-${ROS2_WS_DIR:-../ros2_ws}/src/5g_drone/config/foxglove/chimera_sim.json}
lead=$FIRST_UAS

if [ ! -f "$layout" ]; then
	fail "no layout at $layout"
	return 0
fi
if [ -z "$(COMPOSE_PROFILES=offboard docker compose ps -q offboard 2>/dev/null)" ]; then
	fail "the ground station is not running. Start it: ./px4sim start offboard"
	return 0
fi

# A value outside the set Foxglove accepts fails the panel's whole config, and
# it then draws its defaults: a street map, no colors, following nothing.
if problems=$(python3 verify/component/foxglove_layout.py "$layout"); then
	pass "every panel setting is one Foxglove accepts"
else
	fail "every panel setting is one Foxglove accepts"
	note "$problems"
fi

# What the bridge offers, against what the layout draws.
mapfile -t drawn < <(python3 verify/component/foxglove_layout.py --topics "$layout")
seen=$(./px4sim foxglove ground "${drawn[@]}" --seconds "${FOXGLOVE_SECONDS:-15}" 2>/dev/null)
missing=$(printf '%s\n' "$seen" | awk -F'\t' '$3 == "absent" { print $1 }')
if [ -z "$missing" ]; then
	pass "Foxglove is offered every one of the ${#drawn[@]} topics the layout draws"
else
	fail "Foxglove is offered every topic the layout draws"
	note "$(printf '%s' "$missing" | tr '\n' ' ')"
fi

# The live view. The camera reaches the ground as a stream, and the ground's
# own viewer turns it back into the topic the Image panel reads: a viewer that
# started before the stream did used to die there and leave the panel waiting.
verdict=$(printf '%s\n' "$seen" | awk -F'\t' -v t="/uas$lead/image" '$1 == t { print $3 }')
expect_eq "the Image panel's live view carries frames" data "${verdict:-absent}"
verdict=$(printf '%s\n' "$seen" | awk -F'\t' -v t="/uas$lead/camera/camera_info" '$1 == t { print $3 }')
expect_eq "the live view is calibrated, so the boxes land on it" data "${verdict:-absent}"

# The ground, drawn the right way round. The model's own bytes say which way
# its axes and its map face; a turned or mirrored one publishes just as
# convincingly as a correct one.
if scene=$(./px4sim uas ground scene 2>&1); then
	pass "the satellite map is drawn north up over the terrain"
	note "$(printf '%s' "$scene" | tr '\n' ' ' | cut -c1-160)"
else
	fail "the satellite map is drawn north up over the terrain"
	note "$(printf '%s' "$scene" | tr '\n' ' ' | cut -c1-200)"
fi

# The rings around the known targets, read back as the Map panel parses them.
rings="/viz/uas$lead/scoring/target_rings"
drawn_rings=$(./px4sim foxglove ground "$rings" --show "$rings" \
	--seconds "${FOXGLOVE_SECONDS:-15}" 2>/dev/null | sed -n '/^{/,$p')
if [ -z "$drawn_rings" ]; then
	fail "the Map panel is given a ring for every known target"
	note "nothing arrived on $rings"
elif measured=$(printf '%s' "$drawn_rings" | python3 verify/component/map_layer.py --outlines 2>&1); then
	pass "every known target has a 2 m outline in the color of its verdict"
	note "$(printf '%s' "$measured" | head -3 | tr '\n' ' ')"
else
	fail "every known target has a 2 m outline in the color of its verdict"
	note "$(printf '%s' "$measured" | tail -3 | tr '\n' ' ')"
fi

# Every verdict layer accounts for itself the same way, so an operator reading
# a false positive learns what it landed near rather than only that it was
# wrong. A kind that did not happen this run has an empty layer, which is an
# answer rather than a fault.
for kind in true_positives false_positives missed_localizations; do
	layer="/viz/uas$lead/scoring/$kind"
	body=$(./px4sim foxglove ground "$layer" --show "$layer" \
		--seconds "${FOXGLOVE_SECONDS:-15}" 2>/dev/null | sed -n '/^{/,$p')
	if [ -z "$body" ]; then
		fail "the $kind layer reaches Foxglove"
		continue
	fi
	if read_back=$(printf '%s' "$body" | python3 verify/component/map_layer.py \
			--verdicts --allow-empty 2>&1); then
		pass "every pin on $kind says its verdict, what it touched and its nearest target"
		note "$(printf '%s' "$read_back" | head -2 | tr '\n' ' ')"
	else
		fail "every pin on $kind says its verdict, what it touched and its nearest target"
		note "$(printf '%s' "$read_back" | tail -2 | tr '\n' ' ')"
	fi
done
