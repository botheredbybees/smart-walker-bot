"""Pure Python: builds an LLM-facing system message summarizing the
walker's latest gait/anomaly data, so the assistant can answer wellness
questions from real numbers instead of guessing. No ROS import - the
node extracts primitives from messages before calling this. See
docs/superpowers/specs/2026-08-30-walker-llm-bridge-design.md and
CLAUDE.md's wellness/monitoring feature design principle (health data
should be exposed conversationally, warmly, not paternalistically).
"""

PREFACE = (
    "Background wellness information for you (the assistant) only - mention it "
    "only if relevant to what the user is asking, and speak about it warmly and "
    "matter-of-factly, not clinically. Don't invent numbers that aren't listed here."
)


def _alert_summary(alert_counts, latest_alert_type):
    if not alert_counts:
        return "No anomaly alerts reported so far."

    # Alphabetical, not chronological - latest_alert_type is the
    # explicit, separate way chronology is conveyed below.
    phrases = [
        f"{count} {alert_type} alert{'' if count == 1 else 's'}"
        for alert_type, count in sorted(alert_counts.items())
    ]
    if len(phrases) == 1:
        joined = phrases[0]
    else:
        joined = ', '.join(phrases[:-1]) + ' and ' + phrases[-1]

    return f"{joined} reported, most recently a {latest_alert_type}."


def build_wellness_context_message(gait, alert_counts, latest_alert_type):
    """Returns a {'role': 'system', 'content': ...} message summarizing
    known gait/alert data, or None if nothing is known yet (gait is
    None and no alerts have been seen)."""
    if gait is None and not alert_counts:
        return None

    parts = [PREFACE]
    if gait is not None:
        parts.append(
            f"Steps so far: {int(gait['step_count'])}. Distance covered: about "
            f"{gait['total_distance_m']:.1f} meters. Average step length: about "
            f"{gait['avg_step_length_m']:.1f} meters."
        )
    parts.append(_alert_summary(alert_counts, latest_alert_type))

    return {'role': 'system', 'content': ' '.join(parts)}
