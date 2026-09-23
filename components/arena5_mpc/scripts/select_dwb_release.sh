#!/usr/bin/env bash
set -Eeuo pipefail

STABLE_WS="${ARENA_STABLE_WS:-/home/lpc/workspace/arena5_ws}"
RELEASE_ID="${1:?usage: select_dwb_release.sh RELEASE_ID}"
RELEASE_ROOT="$STABLE_WS/optional/mpc/releases/$RELEASE_ID"
WRAPPER="$STABLE_WS/scripts/run_six_behaviors_dwb_08.sh"

if [[ ! "$RELEASE_ID" =~ ^[0-9A-Za-z._-]+$ ]]; then
    echo "release ID contains unsupported characters" >&2
    exit 2
fi
for required in SHA256SUMS stable_protected.sha256 run_dwb_release.sh RELEASE; do
    if [[ ! -f "$RELEASE_ROOT/$required" ]]; then
        echo "release does not provide the DWB speed profile: $RELEASE_ROOT/$required" >&2
        exit 1
    fi
done

(cd "$RELEASE_ROOT" && sha256sum --check --quiet SHA256SUMS)
(cd "$STABLE_WS" && sha256sum --check --quiet "$RELEASE_ROOT/stable_protected.sha256")
ARENA_STABLE_WS="$STABLE_WS" "$RELEASE_ROOT/run_dwb_release.sh" --check-runtime-only

wrapper_header='#!/usr/bin/env bash
set -Eeuo pipefail
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/optional/mpc/releases/'
wrapper_footer='/run_dwb_release.sh" "$@"'
old_release_id="none"

if [[ -e "$WRAPPER" ]]; then
    if [[ ! -f "$WRAPPER" || -L "$WRAPPER" ]]; then
        echo "refusing to replace a non-regular DWB speed wrapper: $WRAPPER" >&2
        exit 1
    fi
    wrapper_text="$(<"$WRAPPER")"
    if [[ "$wrapper_text" != "$wrapper_header"*"$wrapper_footer" ]]; then
        echo "refusing to overwrite a modified DWB speed wrapper: $WRAPPER" >&2
        exit 1
    fi
    old_release_id="${wrapper_text#"$wrapper_header"}"
    old_release_id="${old_release_id%"$wrapper_footer"}"
    expected_old="$wrapper_header$old_release_id$wrapper_footer"
    if [[ ! "$old_release_id" =~ ^[0-9A-Za-z._-]+$ || "$wrapper_text" != "$expected_old" ]]; then
        echo "refusing to overwrite an unrecognized DWB speed wrapper: $WRAPPER" >&2
        exit 1
    fi
fi

wrapper_dir="$(dirname "$WRAPPER")"
new_wrapper="$(mktemp "$wrapper_dir/.run_six_behaviors_dwb_08.new.XXXXXX")"
old_wrapper="$(mktemp "$wrapper_dir/.run_six_behaviors_dwb_08.old.XXXXXX")"
cleanup() { rm -f "$new_wrapper" "$old_wrapper"; }
trap cleanup EXIT

printf '%s%s%s\n' "$wrapper_header" "$RELEASE_ID" "$wrapper_footer" >"$new_wrapper"
chmod 0755 "$new_wrapper"
if [[ "$old_release_id" != "none" ]]; then
    cp -p "$WRAPPER" "$old_wrapper"
fi
mv -f "$new_wrapper" "$WRAPPER"

if ! ARENA_STABLE_WS="$STABLE_WS" "$WRAPPER" --check-runtime-only; then
    if [[ "$old_release_id" != "none" ]]; then
        mv -f "$old_wrapper" "$WRAPPER"
        echo "DWB speed release selection failed; restored $old_release_id" >&2
    else
        rm -f "$WRAPPER"
        echo "DWB speed release selection failed; removed the new wrapper" >&2
    fi
    exit 1
fi

echo "DWB_SPEED_RELEASE_SELECTED old=$old_release_id new=$RELEASE_ID"
