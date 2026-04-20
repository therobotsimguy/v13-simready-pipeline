#!/usr/bin/env python3
"""V13 Learning Propagator — 3-step fix rule, automated.

Takes a diagnosed failure mode (observation + root cause + proposed fix)
and drafts the three artifacts the V13 pipeline requires:

  1. SKILL   — table row for skills/failure-modes/SKILL.md
  2. CODE    — fix function for make_simready.py + apply_physics call site
  3. AUDIT   — assertion in audit() that FAILs if the condition reappears

The draft is emitted as a markdown document for human review. This tool
never writes to the three target files directly and never commits —
the human reviews, copies, runs `make scan && make lint`, and rebuilds
a baseline asset to confirm no regression.

Allocates the next available F-number (or D/K/S) from fixes.json so
numbering stays consistent.

Usage:
  propagate_learning.py \\
      --observation "wheels skid on cube-shape casters when dragged" \\
      --diagnosis "wheel Xform has no tire sub-mesh, bbox aspect 1.06" \\
      --proposed-fix "synthesize primitive cylinder sized to wheel bbox" \\
      --tier F

  propagate_learning.py --observation ... --dry-run
      (prints the prompt that would be sent; no LLM call, no API spend)

Dependencies: claude_agent_sdk (already required by simready_agent.py).

Cost note: default model is Claude Sonnet 4.6 with bounded max_tokens.
The prompt is ~4K tokens of context + ~3K of output — ≤ $0.05 per run.
Run with --dry-run first if you're unsure.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import re
import sys
from datetime import date
from pathlib import Path

V13 = Path(__file__).resolve().parent
MANIFEST = V13 / "fixes.json"
SKILL_FAILURES = V13 / "skills/failure-modes/SKILL.md"
CODE = V13 / "scripts/tools/simready_assets/make_simready.py"
DRAFTS_DIR = V13 / ".research_delta/propagations"

DEFAULT_MODEL = "claude-sonnet-4-5"

# F63/F64 are the cleanest gold-standard precedents — both completed all 3
# steps in one session and the audit FAIL messages name the fix function.
# The propagator includes them as few-shot examples.
GOLD_EXAMPLES = ("F63", "F64")


def load_manifest() -> dict:
    if not MANIFEST.exists():
        sys.exit(f"manifest missing: {MANIFEST}\nrun: make scan")
    return json.loads(MANIFEST.read_text())


def next_id(manifest: dict, tier: str) -> str:
    """Return next available ID in tier, e.g. 'F65'."""
    nums = []
    for e in manifest["entries"]:
        if e["tier"] == tier:
            m = re.match(rf"{tier}(\d+)", e["id"])
            if m:
                nums.append(int(m.group(1)))
    return f"{tier}{max(nums) + 1 if nums else 1:02d}"


def extract_example(manifest: dict, fid: str) -> str:
    """Pull the skill row, code function name, audit citation for a gold
    example. Used to show the LLM exactly what shape the output must take."""
    entry = next((e for e in manifest["entries"] if e["id"] == fid), None)
    if not entry:
        return f"(no entry for {fid})"

    skill_line = None
    if entry["skill"]["path"]:
        lines = SKILL_FAILURES.read_text().splitlines()
        ln = entry["skill"]["line"]
        if 1 <= ln <= len(lines):
            skill_line = lines[ln - 1]

    fn_name = entry["code"]["function"]
    # Grep the first few lines of the function's docstring.
    fn_doc = None
    if fn_name:
        text = CODE.read_text()
        m = re.search(
            rf"^def {re.escape(fn_name)}\(.*?\):\n\s*\"\"\"(.+?)\"\"\"",
            text, re.MULTILINE | re.DOTALL,
        )
        if m:
            fn_doc = m.group(1).strip().split("\n")[0][:200]

    # Grep the audit FAIL message that cites the ID.
    audit_cite = None
    if entry["audit"]["cites_id"]:
        text = CODE.read_text()
        m = re.search(
            rf"(c\d_detail\s*\+=\s*\([^)]*?{fid}[^)]*?\))",
            text, re.DOTALL,
        )
        if m:
            audit_cite = re.sub(r"\s+", " ", m.group(1))[:250]

    return (
        f"### {fid} precedent\n"
        f"- Skill row: `{skill_line}`\n"
        f"- Code fn: `{fn_name}` — {fn_doc}\n"
        f"- Audit cite: `{audit_cite}`\n"
    )


def load_context(manifest: dict) -> str:
    """Compact context for the prompt: skill overview, code structure,
    audit criteria, and two gold-standard precedents."""
    # Skill: just tiers + category counts, not full table.
    skill = SKILL_FAILURES.read_text()
    skill_head = "\n".join(skill.splitlines()[:95])  # up to end of Tier 1–3 table

    # Code: function inventory (names only).
    code = CODE.read_text()
    fn_names = re.findall(r"^def (\w+)", code, re.MULTILINE)
    fn_list = ", ".join(fn_names[:50]) + (", ..." if len(fn_names) > 50 else "")

    # Audit criteria from manifest breakdown.
    tiers = {}
    for e in manifest["entries"]:
        tiers[e["tier"]] = tiers.get(e["tier"], 0) + 1

    examples = "\n\n".join(extract_example(manifest, fid) for fid in GOLD_EXAMPLES)

    return (
        f"# V13 pipeline context (for drafting a propagation)\n\n"
        f"## Failure-modes skill file (partial — Tier 1–3 header + first rows)\n\n"
        f"```markdown\n{skill_head}\n```\n\n"
        f"## Fix-function inventory (make_simready.py)\n\n"
        f"{fn_list}\n\n"
        f"## Audit surface\n\n"
        f"`audit()` at make_simready.py:57 enforces C1–C7 criteria. "
        f"`_tier1_warnings()` at make_simready.py:809 emits advisory-level "
        f"warnings for F50/F59/F61/F62/K12/K16. Each FAIL message must "
        f"include `F##:` prefix and name the fix function for traceability.\n\n"
        f"## Manifest tier counts\n\n"
        f"- F (classical rigid): {tiers.get('F', 0)}\n"
        f"- D (deformable): {tiers.get('D', 0)}\n"
        f"- K (kinematic/synthesis): {tiers.get('K', 0)}\n"
        f"- S (solver/engine-specific): {tiers.get('S', 0)}\n\n"
        f"## Gold-standard precedents (follow this shape exactly)\n\n"
        f"{examples}\n"
    )


def build_prompt(obs: str, diag: str, fix: str, new_id: str,
                 asset: str | None, context: str) -> str:
    asset_line = f"\nFailing asset: {asset}" if asset else ""
    return f"""You are extending the V13 SimReady pipeline with a new failure mode.
The pipeline enforces learnings via a 3-step rule (SKILL + CODE + AUDIT)
— all three artifacts must land together or future regressions slip.

{context}

## New failure to propagate as {new_id}

OBSERVATION: {obs}
ROOT CAUSE DIAGNOSIS: {diag}
PROPOSED FIX: {fix}{asset_line}

## Your task

Draft the three artifacts as a single markdown document with the sections
below. Be concrete — no placeholders, no "TODO", no hand-waving.

### 1. SKILL ENTRY

A single markdown table row for skills/failure-modes/SKILL.md in the
format: `| {new_id} | <Category> | <Symptom> | <Root cause> | <Fix summary with fn name> |`

Category should match existing tiers: Geometry/Units, Classification,
Hierarchy, Position, Limits, Mass, Collision, Friction, Clean, Inertial,
Wheel/Collision, Limits/Drive, Authoring, Clean/Hardware.

### 2. CODE — FIX FUNCTION

Python function to add to make_simready.py. Include:
- Signature with full arg list
- Docstring starting with "{new_id}: <brief>."
- Body that performs the fix idempotently (safe to call on clean assets)
- Return a count or list so the apply_physics call site can log progress

Choose a snake_case fn name that reads well in an audit FAIL message.

### 3. CODE — CALL SITE

A short snippet (5-10 lines) showing where in `apply_physics` to invoke
the new function and what to print for progress.

### 4. AUDIT CHECK

Python snippet for the `audit()` function that detects the condition
reappearing and marks the right criterion as failing. The FAIL message
MUST contain `{new_id}:` prefix and MUST name the fix function.

Pick the correct criterion: C1 (rigid bodies), C2 (collision), C3
(friction), C4 (hierarchy), C5 (joints), C6 (drives), C7 (clean).

### 5. NEXT STEPS

A 4-item checklist for the human reviewer:
1. Apply SKILL/CODE/AUDIT edits
2. Run `make scan && make lint` → confirm {new_id} status = ENFORCED
3. Rebuild InstrumentTrolley_B01_01 → confirm AUDIT 7/7 (no regression)
4. Rebuild the failing asset → confirm fix resolves the observation

## Output requirements

- Emit markdown only. No preamble. No "Here is the draft".
- Start directly with `# Propagation draft — {new_id}`
- Use fenced code blocks for all Python snippets
- Total length ≤ 1500 words. Prefer density over prose."""


async def run_llm(prompt: str, model: str) -> str:
    """Invoke claude_agent_sdk.query; stream text blocks, return concatenated."""
    try:
        from claude_agent_sdk import query, ClaudeAgentOptions, AssistantMessage, TextBlock
    except ImportError:
        sys.exit("claude_agent_sdk not installed. pip install claude-agent-sdk")

    options = ClaudeAgentOptions(
        model=model,
        max_turns=1,
        system_prompt=(
            "You are an expert at V13 SimReady pipeline surgery. "
            "You produce precise, paste-ready code and documentation. "
            "You never invent APIs or file paths you haven't been shown."
        ),
    )

    chunks: list[str] = []
    async for message in query(prompt=prompt, options=options):
        if isinstance(message, AssistantMessage):
            for block in message.content:
                if isinstance(block, TextBlock):
                    chunks.append(block.text)
    return "".join(chunks)


def save_draft(new_id: str, draft: str, obs: str) -> Path:
    DRAFTS_DIR.mkdir(parents=True, exist_ok=True)
    today = date.today().isoformat()
    slug = re.sub(r"[^a-z0-9]+", "-", obs.lower())[:40].strip("-")
    out = DRAFTS_DIR / f"{today}_{new_id}_{slug}.md"
    out.write_text(draft + "\n")
    return out


def parse_args(argv):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--observation", required=True,
                   help="one-line description of the observed failure")
    p.add_argument("--diagnosis", required=True,
                   help="root-cause analysis")
    p.add_argument("--proposed-fix", required=True,
                   help="one-line description of the fix approach")
    p.add_argument("--tier", default="F", choices=list("FDKS"),
                   help="tier for the new ID (default F=classical rigid)")
    p.add_argument("--asset", default=None,
                   help="optional path to failing asset USD")
    p.add_argument("--model", default=DEFAULT_MODEL,
                   help=f"Claude model (default {DEFAULT_MODEL})")
    p.add_argument("--dry-run", action="store_true",
                   help="print the prompt; do not call the LLM")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    manifest = load_manifest()
    new_id = next_id(manifest, args.tier)
    context = load_context(manifest)
    prompt = build_prompt(
        args.observation, args.diagnosis, args.proposed_fix,
        new_id, args.asset, context,
    )

    if args.dry_run:
        print(f"# DRY RUN — next ID would be {new_id}")
        print(f"# Prompt length: ~{len(prompt)} chars (~{len(prompt)//4} tokens)")
        print()
        print(prompt)
        return 0

    print(f"propagating {new_id} via {args.model}...", file=sys.stderr)
    draft = asyncio.run(run_llm(prompt, args.model))
    if not draft.strip():
        sys.exit("LLM returned empty response")

    out_path = save_draft(new_id, draft, args.observation)
    print(draft)
    print(f"\n---\n  draft saved: {out_path.relative_to(V13)}", file=sys.stderr)
    print(f"  next: review draft, apply edits, `make scan && make lint`, "
          f"rebuild InstrumentTrolley_B01_01", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
