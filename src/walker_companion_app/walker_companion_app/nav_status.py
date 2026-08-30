"""Pure Nav2 status-code-to-label mapping for walker_companion_app. See
docs/superpowers/specs/2026-08-30-walker-companion-app-design.md Sec 2.5.
"""

_STATUS_LABELS = {
    0: 'idle',        # action_msgs/GoalStatus.STATUS_UNKNOWN
    1: 'accepted',    # STATUS_ACCEPTED
    2: 'navigating',  # STATUS_EXECUTING
    3: 'canceling',   # STATUS_CANCELING
    4: 'succeeded',   # STATUS_SUCCEEDED
    5: 'canceled',    # STATUS_CANCELED
    6: 'aborted',     # STATUS_ABORTED
}


def status_code_to_label(status_codes):
    """status_codes: list of int action_msgs/GoalStatus codes, in the
    order GoalStatusArray reported them (most-recent goal last). Returns
    a human label for the latest entry; an empty list (no goal ever
    sent) maps to 'idle'."""
    if not status_codes:
        return 'idle'
    return _STATUS_LABELS.get(status_codes[-1], 'unknown')
