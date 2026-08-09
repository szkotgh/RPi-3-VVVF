#!/usr/bin/env bash
#
# Builds the kernel and mirrors kernel7.img onto the VVVF boot card.
# Run from the Stop hook, so it fires once per turn after all edits land.
#
# Stays quiet and exits 0 whenever it cannot do its job (card unplugged,
# build broken, no passwordless sudo) - a failed sync must never block the
# session. Prints a JSON systemMessage only when something worth reporting
# happened.
#
set -uo pipefail

PROJECT_DIR=/home/administrator/RPi-3-VVVF
IMG=DiskImg/kernel7.img

cd "$PROJECT_DIR" || exit 0

msg() { printf '{"systemMessage":%s}\n' "$(printf '%s' "$1" | python3 -c 'import json,sys; print(json.dumps(sys.stdin.read()))')"; }

# Nothing to sync unless the sources still compile.
if ! make Pi3 >/dev/null 2>&1; then
	msg "SD sync skipped: make Pi3 failed"
	exit 0
fi
[ -f "$IMG" ] || exit 0

# The VVVF card is the removable partition labelled BOOT. The internal card
# that runs this Pi is labelled bootfs and is not removable, so it can never
# match here.
DEV=$(lsblk -rno NAME,LABEL,RM,TYPE | awk '$2=="BOOT" && $3=="1" && $4=="part" {print "/dev/"$1; exit}')
[ -n "$DEV" ] || exit 0

# Reuse the automount if udisks already mounted it, otherwise mount it here
# and take responsibility for unmounting.
MNT=$(findmnt -nro TARGET "$DEV" | head -1)
OWN_MOUNT=0
if [ -z "$MNT" ]; then
	MNT=/mnt/sdboot
	sudo -n mkdir -p "$MNT" 2>/dev/null || exit 0
	sudo -n mount "$DEV" "$MNT" 2>/dev/null || exit 0
	OWN_MOUNT=1
fi

release() {
	sync
	if [ "$OWN_MOUNT" = 1 ]; then
		sudo -n umount "$MNT" 2>/dev/null
		sudo -n rmdir "$MNT" 2>/dev/null
	fi
}

# Already up to date.
if [ -f "$MNT/kernel7.img" ] && cmp -s "$IMG" "$MNT/kernel7.img"; then
	release
	exit 0
fi

if sudo -n cp "$IMG" "$MNT/kernel7.img" 2>/dev/null; then
	sync
	if cmp -s "$IMG" "$MNT/kernel7.img"; then
		SIZE=$(stat -c%s "$IMG")
		release
		msg "SD card synced: kernel7.img ($SIZE bytes)"
	else
		release
		msg "SD sync failed: kernel7.img on the card does not match the build"
	fi
else
	release
	msg "SD sync failed: could not write kernel7.img to $MNT"
fi
exit 0
