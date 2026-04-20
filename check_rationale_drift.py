#!/usr/bin/env python3
"""V13 Phase 5 — runtime rationale-vs-audit drift detector.

The classifier emits a `rationale` list per part (e.g.
`["F07:name-contains-wheel", "F49:world-anchor-applied"]`) stating
which skill rules it applied while labeling that part. Audit runs
separately and cites F-numbers in its C1-C7 detail messages when
a criterion fails.

Nothing today verifies these two views agree. Phase 5 cross-references
them per asset build:

  contradiction — classifier claimed F## AND audit FAILS with F## cited
                  (the rule was supposed to fire but the output violates it)
  blind spot    — audit FAILS with F## but classifier never claimed F##
                  (classifier didn't anticipate the failure)
  unverifiable  — classifier claimed F## but F## has no audit check
                  (no way to verify the claim; use fixes.json to see
                  if an enforcement gap should be closed)

Usage:
  check_rationale_drift.py \\
      --classify ~/SimReady_Output/simready/classify/foo_classify.json \\
      --usd ~/SimReady_Output/simready/foo/foo_physics.usd

Exit codes:
  0  no contradictions
  1  contradictions present (classifier claim contradicted by audit)
  2  inputs missing / unreadable
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

V13 = Path(__file__).resolve().parent
MANIFEST = V13 / "fixes.json"

ID_RE = re.compile(r"\b([FDKS]\d+[a-c]?)\b")


def parse_rationale(classify_path: Path) -> dict[str, set[str]]:
    """Return {part_name: {ID1, ID2, ...}} from classify.json rationale fields."""
    if not classify_path.exists():
        sys.exit(f"classify.json missing: {classify_path}")
    data = json.loads(classify_path.read_text())
    out: dict[str, set[str]] = {}
    for part_name, info in (data.get("parts") or {}).items():
        if not isinstance(info, dict):
            continue
        ids = set()
        for rule in info.get("rationale") or []:
            if not isinstance(rule, str):
                continue
            head = rule.split(":", 1)[0].strip()
            m = ID_RE.fullmatch(head)
            if m:
                ids.add(m.group(1))
        if ids:
            out[part_name] = ids
    return out


def load_enforceable_ids() -> set[str]:
    """From fixes.json, return IDs with an audit citation (ENFORCED or AUDIT_INLINE)."""
    if not MANIFEST.exists():
        sys.exit(f"fixes.json missing. run: make scan")
    manifest = json.loads(MANIFEST.read_text())
    return {
        e["id"] for e in manifest["entries"]
        if e.get("audit", {}).get("cites_id")
    }


def run_audit(usd_path: Path) -> dict | None:
    """Open the USD, import make_simready.audit, return results dict.
    Returns None if pxr USD is unavailable (e.g. running outside Isaac Sim)."""
    try:
        from pxr import Usd  # noqa: F401
    except ImportError:
        print("[warn] pxr not available — skipping audit phase; "
              "rationale will be reported without cross-check", file=sys.stderr)
        return None

    sys.path.insert(0, str(V13 / "scripts/tools/simready_assets"))
    try:
        from make_simready import audit  # type: ignore
    except ImportError as e:
        sys.exit(f"could not import make_simready.audit: {e}")

    from pxr import Usd
    stage = Usd.Stage.Open(str(usd_path))
    if not stage:
        sys.exit(f"USD failed to open: {usd_path}")
    return audit(stage)


def extract_audit_ids(results: dict) -> tuple[set[str], set[str]]:
    """Return (ids_in_failed_criteria, ids_in_passed_criteria)."""
    failed: set[str] = set()
    passed: set[str] = set()
    for criterion, r in results.items():
        if criterion.startswith("_"):
            continue
        detail = r.get("detail", "") or ""
        ids_here = set(ID_RE.findall(detail))
        if r.get("pass", True):
            passed |= ids_here
        else:
            failed |= ids_here
    return failed, passed


def render_report(per_part: dict[str, set[str]],
                  enforceable: set[str],
                  audit_failed: set[str] | None,
                  audit_passed: set[str] | None,
                  usd_path: Path) -> str:
    all_claimed = set()
    for ids in per_part.values():
        all_claimed |= ids

    lines = [
        f"# Rationale-vs-audit drift — {usd_path.name}",
        "",
        f"Parts classified: {len(per_part)}   Unique IDs cited: {len(all_claimed)}",
        "",
    ]

    unverifiable = sorted(all_claimed - enforceable)
    if unverifiable:
        lines.append(f"## UNVERIFIABLE ({len(unverifiable)}) — classifier claim, no audit check")
        lines.append(f"IDs: {', '.join(unverifiable)}")
        lines.append("These IDs have no corresponding audit citation in make_simready.py. "
                     "Either enforcement is inline (see fixes.json AUDIT_INLINE) or missing "
                     "(SKILL_ONLY). Consider extending audit() — run `make lint` for status.")
        lines.append("")

    if audit_failed is None:
        lines.append("## Audit phase SKIPPED (pxr/USD not available)")
        lines.append("")
        return "\n".join(lines)

    contradictions = sorted(all_claimed & audit_failed)
    blind_spots = sorted(audit_failed - all_claimed)
    confirmed = sorted(all_claimed & audit_passed)

    lines.append(f"## CONTRADICTION ({len(contradictions)}) — classifier claim + audit FAIL")
    if contradictions:
        lines.append(f"IDs: {', '.join(contradictions)}")
        lines.append(
            "The classifier said these rules applied, but audit finds the output "
            "violates them. Highest-priority drift. Re-examine the per-part "
            "rationale AND the audit failure detail — likely either the classifier "
            "mis-applied the rule or the fix function silently errored."
        )
    else:
        lines.append("(none — classifier claims consistent with audit results)")
    lines.append("")

    lines.append(f"## BLIND SPOT ({len(blind_spots)}) — audit FAIL without classifier claim")
    if blind_spots:
        lines.append(f"IDs: {', '.join(blind_spots)}")
        lines.append(
            "Audit caught these failures but the classifier didn't predict them. "
            "Informational — the classifier doesn't need to cite every rule, but "
            "recurring blind spots suggest a classifier prompt gap."
        )
    else:
        lines.append("(none)")
    lines.append("")

    lines.append(f"## CONFIRMED ({len(confirmed)}) — classifier claim + audit PASS")
    if confirmed:
        lines.append(f"IDs: {', '.join(confirmed)}")
    else:
        lines.append("(no overlap — enforceable claims are all either contradictions or silent)")
    lines.append("")

    lines.append("## Per-part rationale")
    for part_name, ids in sorted(per_part.items()):
        tag = []
        c_here = ids & set(contradictions)
        if c_here:
            tag.append(f"⚠ contradicts {','.join(sorted(c_here))}")
        u_here = ids & set(unverifiable)
        if u_here:
            tag.append(f"? unverifiable {','.join(sorted(u_here))}")
        suffix = f"   [{'; '.join(tag)}]" if tag else ""
        lines.append(f"- `{part_name}`: {', '.join(sorted(ids))}{suffix}")
    return "\n".join(lines)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--classify", required=True, type=Path,
                   help="classify.json with rationale fields per part")
    p.add_argument("--usd", required=True, type=Path,
                   help="physics USD built from the same asset")
    p.add_argument("--output", type=Path, default=None,
                   help="write markdown report here (default: stdout)")
    args = p.parse_args(argv)

    per_part = parse_rationale(args.classify)
    enforceable = load_enforceable_ids()
    audit_results = run_audit(args.usd)

    if audit_results is None:
        report = render_report(per_part, enforceable, None, None, args.usd)
        exit_code = 0
    else:
        failed, passed = extract_audit_ids(audit_results)
        report = render_report(per_part, enforceable, failed, passed, args.usd)
        contradictions = {i for i in per_part for i in per_part[i]} & failed  # type: ignore
        all_claimed = set().union(*per_part.values()) if per_part else set()
        exit_code = 1 if (all_claimed & failed) else 0

    if args.output:
        args.output.write_text(report + "\n")
        print(f"wrote {args.output}", file=sys.stderr)
    else:
        print(report)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
