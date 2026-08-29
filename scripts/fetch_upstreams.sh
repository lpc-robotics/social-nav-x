#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MANIFEST="$REPO_ROOT/upstream/manifest.tsv"

if [[ ! -s "$MANIFEST" ]]; then
    echo "Missing upstream manifest: $MANIFEST" >&2
    exit 1
fi

while IFS=$'\t' read -r path origin commit branch storage patch; do
    if [[ "$path" == "path" || -z "$path" ]]; then
        continue
    fi
    if [[ "$storage" == "vendored" ]]; then
        echo "VENDORED $path base=$commit branch=$branch"
        continue
    fi
    if [[ "$storage" != "fetched" ]]; then
        echo "Unknown storage mode '$storage' for $path" >&2
        exit 1
    fi

    destination="$REPO_ROOT/$path"
    if [[ -e "$destination" && ! -d "$destination/.git" ]]; then
        echo "Refusing to replace non-Git path: $destination" >&2
        exit 1
    fi

    if [[ ! -d "$destination/.git" ]]; then
        mkdir -p "$(dirname "$destination")"
        git clone --filter=blob:none --no-checkout "$origin" "$destination"
        git -C "$destination" checkout --detach "$commit"
    else
        actual_commit="$(git -C "$destination" rev-parse HEAD)"
        if [[ "$actual_commit" != "$commit" ]]; then
            echo "$path is at $actual_commit, expected $commit; not modifying it" >&2
            exit 1
        fi
    fi

    if [[ "$patch" != "-" ]]; then
        patch_path="$REPO_ROOT/$patch"
        if [[ ! -s "$patch_path" ]]; then
            echo "Missing patch for $path: $patch_path" >&2
            exit 1
        fi
        if git -C "$destination" apply --reverse --check "$patch_path"; then
            echo "PATCH_ALREADY_APPLIED $path"
        elif git -C "$destination" apply --check "$patch_path"; then
            git -C "$destination" apply "$patch_path"
            echo "PATCH_APPLIED $path"
        else
            echo "Patch cannot be applied cleanly to $path" >&2
            exit 1
        fi
    fi

    echo "UPSTREAM_READY $path commit=$commit"
done <"$MANIFEST"

echo "UPSTREAM_RECONSTRUCTION_OK"
