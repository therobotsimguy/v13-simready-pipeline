.PHONY: lint scan propagate-dry test-audit help

help:
	@echo "V13 pipeline targets:"
	@echo "  make lint            validate fixes.json against skill/code/audit artifacts"
	@echo "  make scan            regenerate fixes.json from skill-file + code scan"
	@echo "  make test-audit      run the audit regression test suite"
	@echo "  make propagate-dry   show propagator prompt for a sample input (no LLM call)"
	@echo ""
	@echo "Live tools (call directly):"
	@echo "  propagate_learning.py --observation ... --diagnosis ... --proposed-fix ..."
	@echo "  check_rationale_drift.py --classify <file>.json --usd <file>.usd"

lint:
	@python3 lint_fixes_manifest.py

scan:
	@python3 lint_fixes_manifest.py --scan

test-audit:
	@python3 test_audit_fixes.py

propagate-dry:
	@python3 propagate_learning.py \
	  --observation "SAMPLE: wheels don't rotate when chassis is dragged" \
	  --diagnosis "SAMPLE: no tire mesh, cube-aspect collider" \
	  --proposed-fix "SAMPLE: synthesize primitive Cylinder sized to wheel bbox" \
	  --dry-run | head -40
