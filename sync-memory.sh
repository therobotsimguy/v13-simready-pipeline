#!/usr/bin/env bash
# sync-memory.sh — bootstrap Claude Code's auto-memory from V13's LEARNINGS.md
#
# After cloning v13-simready-pipeline on a new machine, run this to wire the
# repo's LEARNINGS.md into Claude's per-project memory directory so future
# Claude Code sessions load it automatically.
#
# Usage:
#   scripts/tools/simready_v13/sync-memory.sh                 # auto-detect target
#   scripts/tools/simready_v13/sync-memory.sh /path/to/memory # explicit target
#
# Idempotent — safe to re-run after every git pull.

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
learnings_src="${script_dir}/LEARNINGS.md"

if [[ ! -f "$learnings_src" ]]; then
    echo "error: LEARNINGS.md not found at $learnings_src" >&2
    exit 1
fi

# Target: Claude Code stores per-project memory under
# ~/.claude/projects/<slug>/memory/ where <slug> is the working directory
# path with '/' replaced by '-'. Default assumes this script is run from
# inside the user's IsaacLab checkout and the V13 clone lives at
# scripts/tools/simready_v13.
if [[ $# -ge 1 ]]; then
    target_dir="$1"
else
    # Walk up from the script to the IsaacLab root (first ancestor that
    # contains this V13 clone via scripts/tools/simready_v13).
    isaaclab_root="$(cd "${script_dir}/../../.." && pwd)"
    slug="$(echo "$isaaclab_root" | sed 's|/|-|g')"
    target_dir="${HOME}/.claude/projects/${slug}/memory"
fi

mkdir -p "$target_dir"

dest="${target_dir}/v13_learnings.md"
cp "$learnings_src" "$dest"
echo "copied: $learnings_src -> $dest"

# Ensure MEMORY.md has a one-line pointer. Create a fresh index if missing;
# otherwise append only if the pointer isn't already there.
index="${target_dir}/MEMORY.md"
pointer="- [V13 learnings](v13_learnings.md) — V13 workflow rules + notable fixes. Synced from v13-simready-pipeline repo."

if [[ ! -f "$index" ]]; then
    echo "$pointer" > "$index"
    echo "created: $index"
elif ! grep -q "v13_learnings.md" "$index"; then
    echo "$pointer" >> "$index"
    echo "updated: $index (pointer appended)"
else
    echo "unchanged: $index (pointer already present)"
fi

echo "done. Claude will load V13 learnings from $dest on next session."
