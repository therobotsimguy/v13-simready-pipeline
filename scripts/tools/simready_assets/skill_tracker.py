#!/usr/bin/env python3
"""
skill_tracker.py — V13 Skill Usage Tracker

Logs which skills were consulted during asset generation,
what they contributed, and what decisions changed because of them.

Usage:
    tracker = SkillTracker(asset_name)
    tracker.log("simready-joint-params", "Looked up caster wheel params",
                decision="damping=2.0 (was 10.0)", impact="override")
    tracker.report()
    tracker.save("/tmp/v13_output/trolleyB/skill_log.json")
"""

import json
import time


class SkillTracker:
    def __init__(self, asset_name="unknown"):
        self.asset_name = asset_name
        self.entries = []
        self.start_time = time.time()
        self.summary = {"activated": 0, "overrides": 0, "confirmations": 0, "no_match": 0}

    def log(self, skill_name, reason, decision="", impact="info"):
        """Log a skill activation.

        impact: "override"      — skill changed a parameter from pipeline default
                "confirmation"  — skill confirmed pipeline's choice was correct
                "warning"       — skill flagged a potential problem
                "info"          — informational lookup, no parameter change
        """
        entry = {
            "skill": skill_name,
            "reason": reason,
            "decision": decision,
            "impact": impact,
            "timestamp": round(time.time() - self.start_time, 2),
        }
        self.entries.append(entry)
        self.summary["activated"] += 1
        if impact == "override":
            self.summary["overrides"] += 1
        elif impact == "confirmation":
            self.summary["confirmations"] += 1

        # Live print
        icon = {"override": ">>", "confirmation": "OK", "warning": "!!", "info": "--"}
        print(f"    [{icon.get(impact, '--')}] {skill_name}: {reason}")
        if decision:
            print(f"        → {decision}")

    def report(self):
        """Print final skill usage report."""
        elapsed = round(time.time() - self.start_time, 1)
        print(f"\n  {'═' * 56}")
        print(f"  SKILL TRACKER REPORT — {self.asset_name}")
        print(f"  {'═' * 56}")
        print(f"  Skills activated:  {self.summary['activated']}")
        print(f"  Parameter overrides: {self.summary['overrides']}")
        print(f"  Confirmations:     {self.summary['confirmations']}")
        print(f"  Elapsed:           {elapsed}s")
        print(f"  {'─' * 56}")

        # Group by skill
        by_skill = {}
        for e in self.entries:
            by_skill.setdefault(e["skill"], []).append(e)

        for skill, entries in by_skill.items():
            overrides = sum(1 for e in entries if e["impact"] == "override")
            confirms = sum(1 for e in entries if e["impact"] == "confirmation")
            warns = sum(1 for e in entries if e["impact"] == "warning")
            print(f"  {skill}:")
            print(f"    {len(entries)} calls ({overrides} overrides, {confirms} confirms, {warns} warnings)")
            for e in entries:
                icon = {"override": ">>", "confirmation": "OK", "warning": "!!", "info": "--"}
                print(f"      [{icon.get(e['impact'], '--')}] {e['reason']}")
                if e["decision"]:
                    print(f"          → {e['decision']}")

        print(f"  {'═' * 56}")
        return self.summary

    def save(self, path):
        """Save full log to JSON."""
        data = {
            "asset": self.asset_name,
            "summary": self.summary,
            "entries": self.entries,
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"  Skill log saved: {path}")
