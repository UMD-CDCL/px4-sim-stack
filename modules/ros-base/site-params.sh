# The site, worked out from the scene rather than configured.
#
# A terrain tile is anchored above mean sea level and a NavSatFix altitude is
# above the WGS84 ellipsoid. GeoidEval says how far apart those are here, and
# mavros already installs the datasets it reads.
#
# Both entry points source this, because the vehicle and the ground station
# draw the same scene against the same fixes. Left out on either side, that
# side's terrain, buildings and camera footprint stand a geoid separation off
# the ground: 33 m in Maryland.
#
# Reads SURFACE and SITE_PARAMS. Sets GEOID_HEIGHT_M for the launch files and
# writes SITE_PARAMS, which the launch loads last.

export GEOID_HEIGHT_M=0.0
if [ -f "${SURFACE}" ]; then
	GEOID_HEIGHT_M=$(python3 -c "
import json, subprocess
latitude, longitude, _ = json.load(open('${SURFACE}'))['origin_lla']
print(subprocess.run(['GeoidEval'], input=f'{latitude} {longitude}',
                     capture_output=True, text=True).stdout.strip())")
	# GeoidEval prints nothing when its datasets are missing, and an empty
	# height would stop the launch that reads it rather than draw the scene
	# where an unset one would.
	if [ -z "${GEOID_HEIGHT_M}" ]; then
		GEOID_HEIGHT_M=0.0
		echo "site: GeoidEval found no datasets, so the scene keeps the fix's datum" >&2
	fi
	export GEOID_HEIGHT_M
	echo "site: geoid height ${GEOID_HEIGHT_M} m"
fi
printf '/**/*:\n  ros__parameters:\n    localization.geoid_height_m: %s\n' \
	"${GEOID_HEIGHT_M}" > "${SITE_PARAMS}"
