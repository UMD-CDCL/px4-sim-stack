#!/usr/bin/env bash
# Emit HOME_* and FIDUCIAL_* shell assignments from a generated scenario.
#
# scenegen writes home_* and fiducial_* lines into every scenario it
# builds, so SCENARIO in .env carries the world origin and the survey
# marker with it. Both front doors eval this output, the same pattern as
# ds-select.sh. A scenario without those lines, such as a hand-written
# one, emits nothing and the .env values stand.
#
#   scenario-env.sh modules/sim/scenes/scenarios/<name>.yaml
set -euo pipefail

file="${1:-}"
[ -f "$file" ] || exit 0

value_of() {
	sed -n "s/^$1:[[:space:]]*//p" "$file" | head -1
}

# Only digits, dot and minus may reach eval. Anything else, including an
# empty value, keeps the whole group silent.
numeric() {
	case "$1" in
		'' | *[!0-9.-]*) return 1 ;;
	esac
}

home_lat=$(value_of home_lat)
home_lon=$(value_of home_lon)
home_alt=$(value_of home_alt)
fiducial_lat=$(value_of fiducial_lat)
fiducial_lon=$(value_of fiducial_lon)
fiducial_alt=$(value_of fiducial_alt)

for value in "$home_lat" "$home_lon" "$home_alt" \
             "$fiducial_lat" "$fiducial_lon" "$fiducial_alt"; do
	numeric "$value" || exit 0
done

echo "HOME_LAT=$home_lat"
echo "HOME_LON=$home_lon"
echo "HOME_ALT=$home_alt"
echo "FIDUCIAL_ENABLED=1"
echo "FIDUCIAL_SURVEYED_LAT=$fiducial_lat"
echo "FIDUCIAL_SURVEYED_LON=$fiducial_lon"
echo "FIDUCIAL_SURVEYED_ALT=$fiducial_alt"
