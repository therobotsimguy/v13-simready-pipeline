#!/usr/bin/env python3
"""
pipeline_debugger.py — V13 Pipeline Debugger & Data Flywheel

Wraps make_simready.py to capture:
  1. Skill activation trace (what was consulted, what it recommended)
  2. Stage profiling (per-part timing + decisions)
  3. Physics diagnostics (post-build sanity checks)
  4. User verdict (PASS/FAIL + notes → the label)

Data accumulates in ~/SimReady_Debug/ across runs.
After enough runs, summary.json shows patterns: which skills help,
which part types fail, which parameters work.

Usage:
    from pipeline_debugger import PipelineDebugger

    dbg = PipelineDebugger("InstrumentTrolley_B01_01")
    dbg.log_skill("failure-modes", "F09: axis from tire bbox", impact="warning")
    dbg.start_stage("collision", part="sm_wheel_b01_01")
    dbg.end_stage(decisions={"approx": "convexDecomposition", "count": 3})
    dbg.run_diagnostics(physics_usd_path)
    dbg.collect_verdict()  # asks user PASS/FAIL
    dbg.save()
"""

import json
import os
import time
from datetime import datetime
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

DEBUG_DIR = os.path.expanduser("~/SimReady_Debug")
SUMMARY_PATH = os.path.join(DEBUG_DIR, "summary.json")


class PipelineDebugger:
    def __init__(self, asset_name="unknown", object_type="unknown"):
        self.asset_name = asset_name
        self.object_type = object_type
        self.run_id = self._next_run_id()
        self.start_time = time.time()
        self.timestamp = datetime.now().isoformat(timespec="seconds")

        # Layer 1: Skill trace
        self.skills = []

        # Layer 2: Stage profiler
        self.stages = []
        self._current_stage = None

        # Layer 3: Physics diagnostics
        self.diagnostics = {}

        # Layer 4: User verdict
        self.verdict = None
        self.user_notes = ""
        self.issues_found = []

        # Metadata
        self.audit_score = ""
        self.mujoco_score = ""
        self.classification = {}
        self.gemini_output = {}

        os.makedirs(DEBUG_DIR, exist_ok=True)
        print(f"\n  [debugger] Run {self.run_id} — {asset_name}")

    def _next_run_id(self):
        """Auto-increment run ID from existing files."""
        existing = list(Path(DEBUG_DIR).glob("run_*.json"))
        if not existing:
            return "run_001"
        nums = []
        for f in existing:
            try:
                nums.append(int(f.stem.split("_")[1]))
            except (IndexError, ValueError):
                pass
        next_num = max(nums) + 1 if nums else 1
        return f"run_{next_num:03d}"

    # ═══════════════════════════════════════════════════════════════
    # LAYER 0: HISTORY CHECK (flywheel feedback)
    # ═══════════════════════════════════════════════════════════════

    def check_history(self, part_types=None, scale=None):
        """Check past runs for relevant learnings before starting a new run.

        Reads all run records in ~/SimReady_Debug/, finds matches by:
          - Same asset name (exact reruns)
          - Same object_type (e.g. all trolley runs)
          - Same part types (e.g. all runs with caster wheels)
          - Scale-related failures

        Returns list of learnings. Also prints them.

        Args:
            part_types: list of part type strings (e.g. ["wheel", "drawer", "slider"])
            scale: asset_scale being used (e.g. 5.0) — triggers scale warnings
        """
        learnings = []
        past_runs = []

        # Load all past run records
        for f in sorted(Path(DEBUG_DIR).glob("run_*.json")):
            try:
                with open(f) as fh:
                    past_runs.append(json.load(fh))
            except (json.JSONDecodeError, IOError):
                continue

        if not past_runs:
            print(f"  [history] No past runs found. First run!")
            return learnings

        print(f"\n  [history] Checking {len(past_runs)} past runs for relevant learnings...")

        # ── 1. Exact asset matches ──
        same_asset = [r for r in past_runs if r.get("asset") == self.asset_name]
        if same_asset:
            passes = sum(1 for r in same_asset if r.get("verdict") == "PASS")
            fails = sum(1 for r in same_asset if r.get("verdict") == "FAIL")
            print(f"  [history] This asset ran {len(same_asset)} times before: {passes} pass, {fails} fail")
            # Surface failure notes
            for r in same_asset:
                if r.get("verdict") == "FAIL":
                    note = r.get("user_notes", "")
                    variant = r.get("variant", "")
                    learning = f"PAST FAIL ({r.get('id','')}): {variant} — {note}"
                    learnings.append(learning)
                    print(f"    !! {learning}")
                # Surface learnings
                if r.get("learning"):
                    learnings.append(f"PAST LEARNING: {r['learning']}")
                    print(f"    >> PAST LEARNING: {r['learning']}")

        # ── 2. Same object type matches ──
        same_type = [r for r in past_runs if r.get("object_type") == self.object_type]
        if same_type and self.object_type != "unknown":
            passes = sum(1 for r in same_type if r.get("verdict") == "PASS")
            fails = sum(1 for r in same_type if r.get("verdict") == "FAIL")
            total = len(same_type)
            rate = passes / total * 100 if total > 0 else 0
            print(f"  [history] Object type '{self.object_type}': {total} runs, {rate:.0f}% pass rate")

            # Extract common parameters from passing runs
            pass_runs = [r for r in same_type if r.get("verdict") == "PASS"]
            fail_runs = [r for r in same_type if r.get("verdict") == "FAIL"]

            # Surface failure patterns for this type
            for r in fail_runs:
                note = r.get("user_notes", "")
                if note:
                    learning = f"PAST {self.object_type} FAIL: {note[:100]}"
                    if learning not in learnings:
                        learnings.append(learning)
                        print(f"    !! {learning}")

        # ── 3. Scale-related warnings ──
        if scale and scale > 1:
            scale_fails = [r for r in past_runs
                          if r.get("verdict") == "FAIL"
                          and (f"{scale:.0f}x" in r.get("variant", "")
                               or f"{scale:.0f}x" in r.get("user_notes", "")
                               or "scale" in r.get("user_notes", "").lower())]
            if scale_fails:
                learning = f"SCALE WARNING: {len(scale_fails)} past failures at scale={scale}x. PhysX prismatic limits don't auto-scale — teleop pre-scale fix required."
                learnings.append(learning)
                print(f"    !! {learning}")
            # Check if pre-scale fix exists
            prescale_passes = [r for r in past_runs
                              if r.get("verdict") == "PASS"
                              and "prescale" in r.get("variant", "")]
            if prescale_passes and scale_fails:
                learning = f"SCALE FIX AVAILABLE: pre-scaling prismatic limits in temp USD works ({len(prescale_passes)} confirmed passes)"
                learnings.append(learning)
                print(f"    >> {learning}")

        # ── 4. Part-type specific learnings ──
        if part_types:
            for pt in part_types:
                pt_lower = pt.lower()
                # Search all past runs for notes mentioning this part type
                relevant = []
                for r in past_runs:
                    notes = (r.get("user_notes", "") + " " + r.get("learning", "")).lower()
                    variant = r.get("variant", "").lower()
                    if pt_lower in notes or pt_lower in variant:
                        relevant.append(r)

                if relevant:
                    passes = sum(1 for r in relevant if r.get("verdict") == "PASS")
                    fails = sum(1 for r in relevant if r.get("verdict") == "FAIL")
                    if fails > 0:
                        for r in relevant:
                            if r.get("verdict") == "FAIL":
                                note = r.get("user_notes", "")[:80]
                                learning = f"PART '{pt}' had failures: {note}"
                                if learning not in learnings:
                                    learnings.append(learning)
                                    print(f"    !! {learning}")
                    if passes > 0:
                        learning = f"PART '{pt}': {passes} past successes"
                        learnings.append(learning)
                        print(f"    OK {learning}")

        # ── 5. Summary stats ──
        if os.path.exists(SUMMARY_PATH):
            with open(SUMMARY_PATH) as f:
                summary = json.load(f)
            total = summary.get("total_runs", 0)
            p = summary.get("pass_count", 0)
            fail = summary.get("fail_count", 0)
            print(f"  [history] Flywheel: {total} total runs, {p} pass, {fail} fail ({p/total*100:.0f}% pass rate)" if total > 0 else "")

        if not learnings:
            print(f"  [history] No relevant warnings or learnings found. Clean slate.")

        self.history_learnings = learnings
        return learnings

    # ═══════════════════════════════════════════════════════════════
    # LAYER 1: SKILL TRACE
    # ═══════════════════════════════════════════════════════════════

    def log_skill(self, skill_name, reason, decision="", impact="info"):
        """Log a skill activation.

        impact: "override"     — skill changed a parameter
                "confirmation" — skill confirmed pipeline default
                "warning"      — skill flagged potential problem
                "info"         — informational, no parameter change
        """
        entry = {
            "skill": skill_name,
            "reason": reason,
            "decision": decision,
            "impact": impact,
            "elapsed_s": round(time.time() - self.start_time, 2),
        }
        self.skills.append(entry)
        icon = {"override": ">>", "confirmation": "OK", "warning": "!!", "info": "--"}
        print(f"    [{icon.get(impact, '--')}] {skill_name}: {reason}")
        if decision:
            print(f"        → {decision}")

    # ═══════════════════════════════════════════════════════════════
    # LAYER 2: STAGE PROFILER
    # ═══════════════════════════════════════════════════════════════

    def start_stage(self, stage_name, part=None):
        """Start timing a pipeline stage."""
        if self._current_stage:
            self.end_stage()
        self._current_stage = {
            "stage": stage_name,
            "part": part,
            "start": time.time(),
            "decisions": {},
        }

    def end_stage(self, decisions=None):
        """End current stage, record timing + decisions."""
        if not self._current_stage:
            return
        elapsed = round(time.time() - self._current_stage["start"], 3)
        record = {
            "stage": self._current_stage["stage"],
            "part": self._current_stage["part"],
            "elapsed_s": elapsed,
            "decisions": decisions or self._current_stage["decisions"],
        }
        self.stages.append(record)
        self._current_stage = None

    def add_decision(self, key, value):
        """Add a decision to the current stage."""
        if self._current_stage:
            self._current_stage["decisions"][key] = value

    # ═══════════════════════════════════════════════════════════════
    # LAYER 3: PHYSICS DIAGNOSTICS
    # ═══════════════════════════════════════════════════════════════

    def run_diagnostics(self, physics_usd_path):
        """Run post-build physics sanity checks on the output USD."""
        print(f"\n  [debugger] Running physics diagnostics...")
        stage = Usd.Stage.Open(str(physics_usd_path))
        if not stage:
            self.diagnostics = {"error": "Cannot open USD"}
            return

        checks = {}
        issues = []

        # ── D1: Disconnected rigid bodies (no joint) ──
        rigid_paths = set()
        joint_connected = set()
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_paths.add(str(prim.GetPath()))
            if prim.IsA(UsdPhysics.Joint):
                b0 = prim.GetRelationship("physics:body0").GetTargets()
                b1 = prim.GetRelationship("physics:body1").GetTargets()
                if b0:
                    joint_connected.add(str(b0[0]))
                if b1:
                    joint_connected.add(str(b1[0]))
        disconnected = rigid_paths - joint_connected
        # Body (root) won't have a joint connecting TO it in non-articulation mode
        # Filter: only flag if there are >1 rigid bodies and some are disconnected
        if len(rigid_paths) > 1 and disconnected:
            # The body itself is expected to not be connected as body1
            dp = stage.GetDefaultPrim()
            body_candidates = {str(c.GetPath()) for c in dp.GetChildren()
                              if c.HasAPI(UsdPhysics.RigidBodyAPI)}
            truly_disconnected = disconnected - body_candidates
            if truly_disconnected:
                issues.append(f"D1: {len(truly_disconnected)} rigid body(s) with no joint: {truly_disconnected}")
        checks["D1_disconnected"] = {"count": len(disconnected), "paths": list(disconnected)}

        # ── D2: Orphaned colliders (no RigidBody ancestor) ──
        orphaned = []
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            has_rb = False
            p = prim
            while p and p.GetPath() != Sdf.Path("/"):
                if p.HasAPI(UsdPhysics.RigidBodyAPI):
                    has_rb = True
                    break
                p = p.GetParent()
            if not has_rb:
                orphaned.append(str(prim.GetPath()))
        if orphaned:
            issues.append(f"D2: {len(orphaned)} collider(s) with no RigidBody ancestor")
        checks["D2_orphaned_colliders"] = {"count": len(orphaned), "paths": orphaned[:5]}

        # ── D3: Mass ratio check ──
        masses = {}
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                ma = prim.GetAttribute("physics:mass")
                m = ma.Get() if ma and ma.HasValue() else 0
                masses[prim.GetName()] = m
        if masses:
            max_mass = max(masses.values())
            min_mass = min(v for v in masses.values() if v > 0) if any(v > 0 for v in masses.values()) else 0
            ratio = max_mass / min_mass if min_mass > 0 else 0
            if ratio > 1000:
                issues.append(f"D3: Mass ratio {ratio:.0f}:1 (max={max_mass:.1f}kg, min={min_mass:.3f}kg)")
            checks["D3_mass_ratio"] = {"ratio": round(ratio, 1), "masses": masses}

        # ── D4: Joint anchor world-space match ──
        anchor_mismatches = []
        for prim in stage.Traverse():
            if not prim.IsA(UsdPhysics.Joint):
                continue
            lp0 = prim.GetAttribute("physics:localPos0")
            lp1 = prim.GetAttribute("physics:localPos1")
            b0 = prim.GetRelationship("physics:body0").GetTargets()
            b1 = prim.GetRelationship("physics:body1").GetTargets()
            if not (lp0 and lp1 and b0 and b1):
                continue
            lp0v = lp0.Get()
            lp1v = lp1.Get()
            if lp0v is None or lp1v is None:
                continue
            # Transform to world
            b0_prim = stage.GetPrimAtPath(b0[0])
            b1_prim = stage.GetPrimAtPath(b1[0])
            if not b0_prim or not b1_prim:
                continue
            xf0 = UsdGeom.Xformable(b0_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            xf1 = UsdGeom.Xformable(b1_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            wp0 = xf0.TransformAffine(Gf.Vec3d(float(lp0v[0]), float(lp0v[1]), float(lp0v[2])))
            wp1 = xf1.TransformAffine(Gf.Vec3d(float(lp1v[0]), float(lp1v[1]), float(lp1v[2])))
            dist = ((wp0[0]-wp1[0])**2 + (wp0[1]-wp1[1])**2 + (wp0[2]-wp1[2])**2) ** 0.5
            if dist > 0.01:  # 1cm tolerance
                anchor_mismatches.append({
                    "joint": prim.GetName(),
                    "world0": [round(wp0[i], 4) for i in range(3)],
                    "world1": [round(wp1[i], 4) for i in range(3)],
                    "distance_m": round(dist, 4),
                })
                issues.append(f"D4: Joint '{prim.GetName()}' anchors {dist*1000:.1f}mm apart in world space")
        checks["D4_anchor_mismatch"] = {"count": len(anchor_mismatches), "mismatches": anchor_mismatches}

        # ── D5: Zero-volume parts ──
        zero_vol = []
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            has_mesh = False
            for desc in Usd.PrimRange(prim):
                if desc.GetTypeName() == "Mesh":
                    pts = desc.GetAttribute("points")
                    if pts and pts.HasValue() and len(pts.Get()) > 0:
                        has_mesh = True
                        break
            if not has_mesh:
                zero_vol.append(prim.GetName())
                issues.append(f"D5: Rigid body '{prim.GetName()}' has no mesh geometry")
        checks["D5_zero_volume"] = {"count": len(zero_vol), "parts": zero_vol}

        # ── D6: Collision coverage ──
        bodies_without_colliders = []
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            has_col = False
            for desc in Usd.PrimRange(prim):
                if desc.HasAPI(UsdPhysics.CollisionAPI):
                    has_col = True
                    break
            if not has_col:
                bodies_without_colliders.append(prim.GetName())
                issues.append(f"D6: Rigid body '{prim.GetName()}' has zero colliders")
        checks["D6_collision_coverage"] = {"count": len(bodies_without_colliders), "parts": bodies_without_colliders}

        self.diagnostics = {
            "checks": checks,
            "issues": issues,
            "total_checks": 6,
            "issues_found": len(issues),
        }

        # Print report
        if issues:
            print(f"  [debugger] {len(issues)} issue(s) found:")
            for issue in issues:
                print(f"    !! {issue}")
        else:
            print(f"  [debugger] All 6 diagnostics PASS")

        self.issues_found = issues
        return self.diagnostics

    # ═══════════════════════════════════════════════════════════════
    # LAYER 4: USER VERDICT
    # ═══════════════════════════════════════════════════════════════

    def set_verdict(self, verdict, notes=""):
        """Set user verdict: 'PASS' or 'FAIL' + optional notes."""
        self.verdict = verdict.upper()
        self.user_notes = notes
        print(f"  [debugger] Verdict: {self.verdict}")
        if notes:
            print(f"  [debugger] Notes: {notes}")

    def collect_verdict_interactive(self):
        """Ask user for verdict via terminal input."""
        print(f"\n  [debugger] Asset: {self.asset_name}")
        print(f"  [debugger] Did the asset work correctly in Isaac Sim?")
        v = input("  [debugger] Enter PASS or FAIL: ").strip().upper()
        while v not in ("PASS", "FAIL"):
            v = input("  [debugger] Please enter PASS or FAIL: ").strip().upper()
        notes = input("  [debugger] Any notes (or Enter to skip): ").strip()
        self.set_verdict(v, notes)

    # ═══════════════════════════════════════════════════════════════
    # SAVE & SUMMARY
    # ═══════════════════════════════════════════════════════════════

    def to_dict(self):
        """Full run record as dict."""
        elapsed = round(time.time() - self.start_time, 2)

        # Skill summary
        skill_summary = {}
        for s in self.skills:
            name = s["skill"]
            if name not in skill_summary:
                skill_summary[name] = {"calls": 0, "overrides": 0, "confirmations": 0, "warnings": 0}
            skill_summary[name]["calls"] += 1
            if s["impact"] == "override":
                skill_summary[name]["overrides"] += 1
            elif s["impact"] == "confirmation":
                skill_summary[name]["confirmations"] += 1
            elif s["impact"] == "warning":
                skill_summary[name]["warnings"] += 1

        # Stage summary
        stage_total = {}
        for st in self.stages:
            stage_name = st["stage"]
            if stage_name not in stage_total:
                stage_total[stage_name] = {"calls": 0, "total_s": 0}
            stage_total[stage_name]["calls"] += 1
            stage_total[stage_name]["total_s"] = round(
                stage_total[stage_name]["total_s"] + st["elapsed_s"], 3)

        return {
            "id": self.run_id,
            "asset": self.asset_name,
            "object_type": self.object_type,
            "timestamp": self.timestamp,
            "elapsed_s": elapsed,

            "skills": {
                "entries": self.skills,
                "summary": skill_summary,
                "total_activations": len(self.skills),
                "total_overrides": sum(1 for s in self.skills if s["impact"] == "override"),
            },

            "stages": {
                "entries": self.stages,
                "summary": stage_total,
            },

            "diagnostics": self.diagnostics,

            "audit_score": self.audit_score,
            "mujoco_score": self.mujoco_score,

            "verdict": self.verdict,
            "user_notes": self.user_notes,
            "issues_found": self.issues_found,

            "classification": self.classification,
            "gemini_output": self.gemini_output,
        }

    def save(self):
        """Save run record and update summary."""
        record = self.to_dict()

        # Save individual run
        safe_name = self.asset_name.replace(" ", "_").replace("/", "_")
        run_path = os.path.join(DEBUG_DIR, f"{self.run_id}_{safe_name}.json")
        with open(run_path, "w") as f:
            json.dump(record, f, indent=2)
        print(f"  [debugger] Run saved: {run_path}")

        # Update summary
        self._update_summary(record)
        return run_path

    def _update_summary(self, record):
        """Update the aggregate summary.json with this run's data."""
        if os.path.exists(SUMMARY_PATH):
            with open(SUMMARY_PATH) as f:
                summary = json.load(f)
        else:
            summary = {
                "total_runs": 0,
                "pass_count": 0,
                "fail_count": 0,
                "unlabeled": 0,
                "by_object_type": {},
                "by_part_type": {},
                "skill_effectiveness": {},
            }

        summary["total_runs"] += 1
        if record["verdict"] == "PASS":
            summary["pass_count"] += 1
        elif record["verdict"] == "FAIL":
            summary["fail_count"] += 1
        else:
            summary["unlabeled"] += 1

        # By object type
        ot = record["object_type"]
        if ot not in summary["by_object_type"]:
            summary["by_object_type"][ot] = {"runs": 0, "pass": 0, "fail": 0}
        summary["by_object_type"][ot]["runs"] += 1
        if record["verdict"] == "PASS":
            summary["by_object_type"][ot]["pass"] += 1
        elif record["verdict"] == "FAIL":
            summary["by_object_type"][ot]["fail"] += 1

        # Skill effectiveness
        for s in record["skills"]["entries"]:
            sname = s["skill"]
            if sname not in summary["skill_effectiveness"]:
                summary["skill_effectiveness"][sname] = {
                    "times_fired": 0, "in_pass_runs": 0, "in_fail_runs": 0,
                    "overrides": 0, "confirmations": 0, "warnings": 0,
                }
            se = summary["skill_effectiveness"][sname]
            se["times_fired"] += 1
            if record["verdict"] == "PASS":
                se["in_pass_runs"] += 1
            elif record["verdict"] == "FAIL":
                se["in_fail_runs"] += 1
            if s["impact"] == "override":
                se["overrides"] += 1
            elif s["impact"] == "confirmation":
                se["confirmations"] += 1
            elif s["impact"] == "warning":
                se["warnings"] += 1

        with open(SUMMARY_PATH, "w") as f:
            json.dump(summary, f, indent=2)

    def print_report(self):
        """Print full debugging report."""
        elapsed = round(time.time() - self.start_time, 1)
        print(f"\n  {'═' * 58}")
        print(f"  PIPELINE DEBUGGER REPORT — {self.run_id}")
        print(f"  Asset: {self.asset_name} ({self.object_type})")
        print(f"  {'═' * 58}")

        # Skills
        n_skills = len(self.skills)
        n_overrides = sum(1 for s in self.skills if s["impact"] == "override")
        n_confirms = sum(1 for s in self.skills if s["impact"] == "confirmation")
        n_warns = sum(1 for s in self.skills if s["impact"] == "warning")
        print(f"\n  SKILLS: {n_skills} activations ({n_overrides} overrides, {n_confirms} confirms, {n_warns} warnings)")
        by_skill = {}
        for s in self.skills:
            by_skill.setdefault(s["skill"], []).append(s)
        for skill, entries in by_skill.items():
            print(f"    {skill}: {len(entries)} calls")

        # Stages
        if self.stages:
            print(f"\n  STAGES:")
            by_stage = {}
            for st in self.stages:
                sn = st["stage"]
                if sn not in by_stage:
                    by_stage[sn] = {"count": 0, "total_s": 0}
                by_stage[sn]["count"] += 1
                by_stage[sn]["total_s"] += st["elapsed_s"]
            for sn, info in by_stage.items():
                bar = "█" * max(1, int(info["total_s"] / max(elapsed, 0.1) * 30))
                print(f"    {sn:25s} {info['total_s']:6.2f}s  ({info['count']} parts)  {bar}")

        # Diagnostics
        if self.diagnostics:
            n_issues = self.diagnostics.get("issues_found", 0)
            status = "CLEAN" if n_issues == 0 else f"{n_issues} ISSUES"
            print(f"\n  DIAGNOSTICS: {status}")
            for issue in self.diagnostics.get("issues", []):
                print(f"    !! {issue}")

        # Scores
        if self.audit_score:
            print(f"\n  AUDIT: {self.audit_score}")
        if self.mujoco_score:
            print(f"  MUJOCO: {self.mujoco_score}")

        # Verdict
        if self.verdict:
            icon = "✓" if self.verdict == "PASS" else "✗"
            print(f"\n  VERDICT: {self.verdict} {icon}")
            if self.user_notes:
                print(f"  NOTES: {self.user_notes}")

        print(f"\n  Elapsed: {elapsed}s")
        print(f"  {'═' * 58}")
