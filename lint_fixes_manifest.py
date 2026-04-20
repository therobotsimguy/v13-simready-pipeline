#!/usr/bin/env python3
"""V13 fixes manifest — learning-propagation drift detector.

The V13 SimReady pipeline enforces learnings via a 3-step rule:
  (1) SKILL   — document in skills/failure-modes/SKILL.md
  (2) CODE    — fix function in scripts/tools/simready_assets/make_simready.py
  (3) AUDIT   — audit() or _tier1_warnings() cites the ID so regressions FAIL

Nothing has enforced that these three stay in sync. This script is the
missing check. It reads ./fixes.json and validates every entry still
resolves to a real location, classifies each by propagation state, and
surfaces drift categories — especially the silent-regression bucket
(code exists, audit doesn't enforce it).

Status categories:
  ENFORCED          skill + code function + audit citation all present
  CODE_NO_AUDIT     skill + code, no audit citation (silent regression risk)
  AUDIT_INLINE      skill + audit citation, code is inline (no named fn)
  SKILL_ONLY        documented, zero enforcement
  CODE_NO_SKILL     cited in code but missing from failure-modes skill (F-number hole)

Usage:
  lint_fixes_manifest.py           lint existing fixes.json
  lint_fixes_manifest.py --scan    regenerate fixes.json from skill + code
  lint_fixes_manifest.py --json    machine-readable report (exit 1 if broken)

Exit codes:
  0  no broken refs
  1  broken refs (manifest points at missing file / function / citation)
  2  manifest missing (run --scan)
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

V13 = Path(__file__).resolve().parent
SKILL_FAILURES = V13 / "skills/failure-modes/SKILL.md"
CODE = V13 / "scripts/tools/simready_assets/make_simready.py"
MANIFEST = V13 / "fixes.json"

# audit() + _tier1_warnings() together are the enforcement surface. Keep this
# pair in sync with make_simready.py if either function moves.
AUDIT_START_LINE = 57       # def audit(...)
AUDIT_END_LINE = 1099       # end of _tier1_warnings()

# Hand-curated binding between failure-mode IDs and named fix functions.
# Only entries with a dedicated `def` go here. Inline enforcement (no named
# function) is marked with code_function=None; the linter still records
# whether the ID is cited in the audit body.
#
# When you add a new fix:
#   - Add a row here if the fix has a named function
#   - Run `lint_fixes_manifest.py --scan` to regenerate fixes.json
#   - The linter will now guard against drift on this entry
KNOWN_FIXES = {
    # id:    (code_function,                                 audit_cites)
    "F01":  ("normalize_to_meters",                          False),
    "F11":  ("reparent_prims_preserve_world_xform",          False),
    "F29":  ("wire_friction",                                True),
    "F31":  ("wire_friction",                                True),
    "F33":  ("strip_existing_physics",                       False),
    "F35":  ("_get_all_descendant_meshes",                   False),
    "F42":  ("split_wheel_structural_parts",                 True),
    "F43":  ("bake_xform_scales",                            True),
    "F47":  ("_is_degenerate_mesh",                          True),
    "F48":  ("_normalize_class_aliases",                     True),
    "F49":  ("make_world_anchor_joint",                      True),
    "F63":  ("weld_structural_siblings_into_body",           True),
    "F64":  ("synthesize_wheel_cylinder_collider",           True),
    "F64b": ("_strip_chassis_wheel_blockers",                False),
    "F64c": ("strip_chassis_floor_blockers",                 False),
}

# Table-row regex for skill markdown. Matches:
#   | F01 | Geometry/Units | Asset in cm not meters | ...
ROW_RE = re.compile(
    r"^\|\s*(?P<id>[FDKS]\d+[a-c]?)\s*\|"
    r"\s*(?P<category>[^|]+?)\s*\|"
    r"\s*(?P<symptom>[^|]+?)\s*\|"
)


def parse_skill(path: Path) -> dict:
    """Extract table rows keyed by ID from a failure-modes SKILL.md."""
    entries = {}
    for ln_no, line in enumerate(path.read_text().splitlines(), 1):
        m = ROW_RE.match(line)
        if m:
            fid = m.group("id")
            entries[fid] = {
                "category": m.group("category").strip(),
                "title": m.group("symptom").strip()[:120],
                "skill_line": ln_no,
            }
    return entries


def load_code() -> tuple[str, str]:
    """Return (full file text, audit+tier1_warnings body text)."""
    text = CODE.read_text()
    lines = text.splitlines()
    audit_body = "\n".join(lines[AUDIT_START_LINE - 1:AUDIT_END_LINE])
    return text, audit_body


def _sort_key(fid: str):
    m = re.match(r"([FDKS])(\d+)([a-c]?)", fid)
    if not m:
        return (fid, 0, "")
    tier, num, suffix = m.group(1), int(m.group(2)), m.group(3)
    return (tier, num, suffix)


def build_manifest() -> dict:
    skill_entries = parse_skill(SKILL_FAILURES)
    text, audit_body = load_code()

    entries = []
    for fid in sorted(skill_entries.keys(), key=_sort_key):
        meta = skill_entries[fid]
        code_fn, audit_flag = KNOWN_FIXES.get(fid, (None, False))
        # Auto-detect audit citation we didn't hard-code.
        if not audit_flag and re.search(rf"\b{re.escape(fid)}\b", audit_body):
            audit_flag = True
        entries.append({
            "id": fid,
            "tier": fid[0],
            "category": meta["category"],
            "title": meta["title"],
            "skill": {
                "path": str(SKILL_FAILURES.relative_to(V13)),
                "line": meta["skill_line"],
            },
            "code": {
                "function": code_fn,
                "file": str(CODE.relative_to(V13)) if code_fn else None,
            },
            "audit": {
                "cites_id": audit_flag,
                "file": str(CODE.relative_to(V13)) if audit_flag else None,
            },
        })

    # Detect F-number holes: IDs mentioned in code but absent from the skill file.
    cited_ids = set(re.findall(r"\b([FDKS]\d+[a-c]?)\b", text))
    # Filter obvious false positives (e.g. K in variable names like K8s — none
    # likely here, but protect against D4L / S3 etc. by requiring an F/D/K/S
    # followed by >=1 digit, already enforced by the regex).
    orphans = sorted(cited_ids - set(skill_entries.keys()), key=_sort_key)
    for fid in orphans:
        code_fn, _ = KNOWN_FIXES.get(fid, (None, False))
        audit_flag = bool(re.search(rf"\b{re.escape(fid)}\b", audit_body))
        entries.append({
            "id": fid,
            "tier": fid[0],
            "category": "UNDOCUMENTED",
            "title": "Referenced in code but missing from failure-modes/SKILL.md",
            "skill": {"path": None, "line": None},
            "code": {
                "function": code_fn,
                "file": str(CODE.relative_to(V13)),
            },
            "audit": {
                "cites_id": audit_flag,
                "file": str(CODE.relative_to(V13)) if audit_flag else None,
            },
        })

    return {
        "version": 1,
        "description": (
            "V13 learning-propagation manifest. One row per failure-mode ID. "
            "See lint_fixes_manifest.py for schema and regeneration."
        ),
        "entries": entries,
    }


def classify(entry: dict) -> str:
    skill_ok = entry["skill"]["path"] is not None
    code_ok = entry["code"]["function"] is not None
    audit_ok = entry["audit"]["cites_id"]
    if not skill_ok:
        return "CODE_NO_SKILL"
    if code_ok and audit_ok:
        return "ENFORCED"
    if code_ok and not audit_ok:
        return "CODE_NO_AUDIT"
    if audit_ok and not code_ok:
        return "AUDIT_INLINE"
    return "SKILL_ONLY"


def lint(manifest: dict) -> tuple[list[str], dict]:
    """Validate every manifest entry still resolves. Returns (broken_refs, counts)."""
    text, audit_body = load_code()
    broken: list[str] = []
    counts: dict[str, int] = {}

    for entry in manifest["entries"]:
        fid = entry["id"]

        # 1. Skill ref: path exists AND the recorded line still contains the ID.
        if entry["skill"]["path"]:
            skill_path = V13 / entry["skill"]["path"]
            if not skill_path.exists():
                broken.append(f"{fid}: skill file missing: {entry['skill']['path']}")
            else:
                lines = skill_path.read_text().splitlines()
                ln = entry["skill"]["line"]
                if ln < 1 or ln > len(lines) or not re.search(
                    rf"\|\s*{re.escape(fid)}\s*\|", lines[ln - 1]
                ):
                    broken.append(
                        f"{fid}: skill line {ln} no longer matches "
                        f"(ID moved or row renamed — rescan)"
                    )

        # 2. Code ref: if a function is named, `def <name>` must exist.
        fn = entry["code"]["function"]
        if fn and not re.search(rf"^def\s+{re.escape(fn)}\s*\(", text, re.MULTILINE):
            broken.append(
                f"{fid}: code function '{fn}' not found in {entry['code']['file']} "
                f"(rename drift — update fixes.json or restore function)"
            )

        # 3. Audit ref: if claimed, the ID must be cited in audit()/_tier1_warnings.
        if entry["audit"]["cites_id"]:
            if not re.search(rf"\b{re.escape(fid)}\b", audit_body):
                broken.append(
                    f"{fid}: audit claims citation but '{fid}' not in audit() "
                    f"or _tier1_warnings() body (citation removed silently)"
                )

        counts[classify(entry)] = counts.get(classify(entry), 0) + 1

    return broken, counts


def print_report(manifest: dict, broken: list[str], counts: dict) -> None:
    total = len(manifest["entries"])
    print(f"V13 fixes manifest — {total} entries")
    print()
    print("  status breakdown:")
    order = ["ENFORCED", "AUDIT_INLINE", "CODE_NO_AUDIT", "CODE_NO_SKILL", "SKILL_ONLY"]
    for status in order:
        if status in counts:
            print(f"    {status:16s} {counts[status]:3d}")
    for status, n in sorted(counts.items()):
        if status not in order:
            print(f"    {status:16s} {n:3d}")
    print()

    silent = sorted(
        (e["id"] for e in manifest["entries"] if classify(e) == "CODE_NO_AUDIT"),
        key=_sort_key,
    )
    if silent:
        print(f"  CODE_NO_AUDIT ({len(silent)} — silent regression risk, fix first):")
        print(f"    {', '.join(silent)}")
        print()

    orphans = sorted(
        (e["id"] for e in manifest["entries"] if classify(e) == "CODE_NO_SKILL"),
        key=_sort_key,
    )
    if orphans:
        print(f"  CODE_NO_SKILL ({len(orphans)} — in code, missing from skill):")
        print(f"    {', '.join(orphans)}")
        print()

    if broken:
        print(f"  BROKEN REFS ({len(broken)}) — manifest points at missing artifact:")
        for msg in broken:
            print(f"    - {msg}")
    else:
        print("  no broken refs.")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scan", action="store_true",
                   help="regenerate fixes.json from skill + code scan")
    p.add_argument("--json", action="store_true",
                   help="emit machine-readable status (exit 1 if broken)")
    args = p.parse_args()

    if args.scan:
        manifest = build_manifest()
        MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"wrote {MANIFEST.relative_to(V13)} ({len(manifest['entries'])} entries)")
        return 0

    if not MANIFEST.exists():
        print(f"manifest missing: {MANIFEST}\nrun: lint_fixes_manifest.py --scan", file=sys.stderr)
        return 2

    manifest = json.loads(MANIFEST.read_text())
    broken, counts = lint(manifest)

    if args.json:
        print(json.dumps({"counts": counts, "broken": broken}, indent=2))
    else:
        print_report(manifest, broken, counts)

    return 1 if broken else 0


if __name__ == "__main__":
    sys.exit(main())
