"""Tests for deterministic mission progress monitoring."""

import pytest

from drone_utils.progress_monitor import ProgressMonitor


def test_required_improvement_must_be_strictly_positive():
    with pytest.raises(ValueError, match="greater than zero"):
        ProgressMonitor(window_s=2.0, min_improvement=0.0)


def test_continuing_progress_restarts_the_stall_window():
    monitor = ProgressMonitor(window_s=2.0, min_improvement=0.5)

    assert not monitor.observe(10.0, now=100.0)
    assert not monitor.observe(9.4, now=101.9)
    assert not monitor.observe(9.4, now=103.8)
    assert monitor.observe(9.4, now=103.9)


def test_retry_marker_is_available_exactly_once():
    monitor = ProgressMonitor(window_s=2.0, min_improvement=0.5)
    monitor.observe(10.0, now=100.0)
    assert monitor.observe(10.0, now=102.0)

    assert monitor.mark_retry(now=102.0)
    assert monitor.retry_used
    assert not monitor.mark_retry(now=102.1)


def test_no_progress_is_detected_again_after_the_retry_window():
    monitor = ProgressMonitor(window_s=2.0, min_improvement=0.5)
    monitor.observe(10.0, now=100.0)
    assert monitor.observe(10.0, now=102.0)
    monitor.mark_retry(now=102.0)

    assert not monitor.observe(10.0, now=103.9)
    assert monitor.observe(10.0, now=104.0)


def test_observation_time_must_be_explicit_and_monotonic():
    monitor = ProgressMonitor(window_s=2.0, min_improvement=0.5)

    with pytest.raises(TypeError):
        monitor.observe(10.0)

    monitor.observe(10.0, now=100.0)
    with pytest.raises(ValueError, match="monotonic"):
        monitor.observe(9.0, now=99.0)


def test_reset_starts_a_new_action_with_an_unused_retry():
    monitor = ProgressMonitor(window_s=2.0, min_improvement=0.5)
    monitor.observe(10.0, now=100.0)
    monitor.mark_retry(now=102.0)

    monitor.reset()

    assert not monitor.retry_used
    assert not monitor.observe(4.0, now=1.0)
