#!/usr/bin/env bash
# Static contract check for MTC candidate scene-revision binding.
set -eo pipefail
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
verifier="$workspace_root/scripts/verify_candidate_revision_contract.py"
if [[ ! -f "$verifier" ]]; then
  echo "FAIL missing verifier: scripts/verify_candidate_revision_contract.py" >&2
  exit 1
fi
exec /usr/bin/python3 "$verifier"
