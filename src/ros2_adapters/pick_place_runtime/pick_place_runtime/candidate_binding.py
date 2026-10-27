"""Scene revision check shared by planning consumers and execution adapters."""

def scene_revision_status(candidate_revision, current_revision, scene_ready, revision_seen):
    """Return whether a candidate is still valid for the observed scene."""
    if not revision_seen:
        return True, "scene_revision_unavailable_legacy_mode"
    if candidate_revision is None:
        return False, "candidate_has_no_scene_revision"
    if not scene_ready:
        return False, "scene_not_ready"
    if (isinstance(current_revision, bool) or not isinstance(current_revision, int)
            or current_revision < 0 or current_revision != candidate_revision):
        return False, "stale_scene_revision"
    return True, "scene_revision_confirmed"
