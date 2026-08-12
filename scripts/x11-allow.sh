#!/usr/bin/env bash
# Build the X11 cookie file that the GUI containers mount.
#
# This is the narrow alternative to `xhost +local:`. It grants access to the
# containers through one cookie file instead of opening the X server to every
# local process.
#
# The cookie lives in the project directory, not in /tmp. Docker creates a
# missing bind-mount source as a *directory*, so a container that starts before
# this script runs leaves a directory where the file belongs. In /tmp that
# directory belongs to root and needs sudo to remove. In the project directory
# it belongs to you, and this script clears it without help.
set -euo pipefail

cd "$(dirname "$0")/.."

XAUTH=${XAUTH_FILE:-./.xauth}
DISP=${DISPLAY:-}

if [ -z "$DISP" ]; then
	echo "DISPLAY is empty. Run this from a graphical session." >&2
	exit 1
fi

if ! command -v xauth >/dev/null 2>&1; then
	echo "xauth not found. Run: sudo apt install xauth" >&2
	exit 1
fi

# Clear whatever is there, whether it is a file, a directory or a link.
if [ -e "$XAUTH" ] || [ -L "$XAUTH" ]; then
	if [ ! -f "$XAUTH" ]; then
		echo "$XAUTH is not a regular file. Docker most likely created it as a"
		echo "directory when a container started before this script ran. Removing it."
	fi
	if ! rm -rf "$XAUTH" 2>/dev/null; then
		echo "Cannot remove $XAUTH. It belongs to another user." >&2
		echo "Remove it by hand, then run this again:" >&2
		echo "    sudo rm -rf $XAUTH" >&2
		exit 1
	fi
fi

touch "$XAUTH"

# The wildcard family prefix (ffff) lets a client with a different hostname,
# which every container has, use the cookie.
if ! xauth nlist "$DISP" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - ; then
	echo "xauth could not read a cookie for $DISP." >&2
	exit 1
fi
chmod 644 "$XAUTH"

if [ ! -s "$XAUTH" ]; then
	echo "Warning: $XAUTH is empty. The X server gave no cookie for $DISP."
	echo "The GUI containers will not reach the display."
fi

echo "Wrote $(cd "$(dirname "$XAUTH")" && pwd)/$(basename "$XAUTH") for DISPLAY=$DISP"
