import math
import unittest

from formal_social_behavior.config import EventThresholds
from formal_social_behavior.event_extractor import (
    EventExtractor,
    disk_ttc_seconds,
    interaction_metrics,
)
from formal_social_behavior.model import (
    EventMemory,
    MotionSnapshot,
    PlanarKinematics,
    SocialEvent,
)


def snapshot(
    stamp_ns,
    *,
    robot_x,
    robot_y=0.0,
    robot_vx=0.0,
    robot_vy=0.0,
    robot_radius=0.35,
    human_x=0.0,
    human_y=0.0,
    human_vx=0.0,
    human_vy=0.0,
    human_yaw=0.0,
    human_radius=0.4,
):
    return MotionSnapshot(
        sim_time_ns=stamp_ns,
        robot=PlanarKinematics(
            x=robot_x,
            y=robot_y,
            vx=robot_vx,
            vy=robot_vy,
            radius=robot_radius,
        ),
        human=PlanarKinematics(
            x=human_x,
            y=human_y,
            vx=human_vx,
            vy=human_vy,
            yaw=human_yaw,
            radius=human_radius,
        ),
    )


class GeometryTests(unittest.TestCase):
    def test_closing_speed_and_disk_ttc(self):
        metrics = interaction_metrics(
            snapshot(0, robot_x=5.0, robot_vx=-1.0)
        )
        self.assertAlmostEqual(metrics.distance, 5.0)
        self.assertAlmostEqual(metrics.closing_speed, 1.0)
        self.assertAlmostEqual(metrics.ttc_seconds, 4.25)

    def test_static_and_departing_trajectories_have_infinite_ttc(self):
        static = interaction_metrics(snapshot(0, robot_x=5.0))
        departing = interaction_metrics(
            snapshot(1, robot_x=5.0, robot_vx=1.0)
        )
        self.assertTrue(math.isinf(static.ttc_seconds))
        self.assertTrue(math.isinf(departing.ttc_seconds))
        self.assertAlmostEqual(departing.closing_speed, -1.0)

    def test_overlap_has_zero_ttc_even_when_static(self):
        metrics = interaction_metrics(snapshot(0, robot_x=0.5))
        self.assertEqual(metrics.ttc_seconds, 0.0)
        self.assertEqual(
            disk_ttc_seconds(0.5, 0.0, 0.0, 0.0, 0.75), 0.0
        )

    def test_miss_trajectory_has_infinite_ttc(self):
        self.assertTrue(
            math.isinf(disk_ttc_seconds(5.0, 2.0, -1.0, 0.0, 0.75))
        )


class HysteresisTests(unittest.TestCase):
    def setUp(self):
        self.extractor = EventExtractor()

    def evaluate(self, memory, stamp_ns, **kwargs):
        return self.extractor.evaluate(snapshot(stamp_ns, **kwargs), memory)

    def test_fov_wraparound(self):
        bearing = math.radians(-179.0)
        result = self.evaluate(
            None,
            0,
            robot_x=5.0 * math.cos(bearing),
            robot_y=5.0 * math.sin(bearing),
            human_yaw=math.radians(179.0),
        )
        self.assertIn(SocialEvent.ROBOT_VISIBLE, result.event_snapshot.events)
        self.assertAlmostEqual(
            abs(math.degrees(result.event_snapshot.metrics.relative_bearing)),
            2.0,
        )

    def test_visibility_uses_enter_exit_distance_hysteresis(self):
        first = self.evaluate(None, 0, robot_x=5.0)
        inside_band = self.evaluate(first.memory, 1, robot_x=6.25)
        at_exit = self.evaluate(inside_band.memory, 2, robot_x=6.5)
        still_out = self.evaluate(at_exit.memory, 3, robot_x=6.25)
        at_enter = self.evaluate(still_out.memory, 4, robot_x=6.0)

        self.assertIn(SocialEvent.ROBOT_VISIBLE, first.event_snapshot.events)
        self.assertIn(SocialEvent.ROBOT_VISIBLE, inside_band.event_snapshot.events)
        self.assertEqual(
            at_exit.event_snapshot.events,
            frozenset({SocialEvent.ROBOT_LOST}),
        )
        self.assertNotIn(SocialEvent.ROBOT_VISIBLE, still_out.event_snapshot.events)
        self.assertIn(SocialEvent.ROBOT_VISIBLE, at_enter.event_snapshot.events)

    def test_visibility_uses_enter_exit_fov_hysteresis(self):
        def at_angle(memory, stamp_ns, angle_deg):
            angle = math.radians(angle_deg)
            return self.evaluate(
                memory,
                stamp_ns,
                robot_x=5.0 * math.cos(angle),
                robot_y=5.0 * math.sin(angle),
                human_yaw=0.0,
            )

        entered = at_angle(None, 0, 99.9)
        held = at_angle(entered.memory, 1, 105.0)
        exited = at_angle(held.memory, 2, 110.1)
        still_out = at_angle(exited.memory, 3, 105.0)
        reentered = at_angle(still_out.memory, 4, 99.9)

        self.assertIn(SocialEvent.ROBOT_VISIBLE, entered.event_snapshot.events)
        self.assertIn(SocialEvent.ROBOT_VISIBLE, held.event_snapshot.events)
        self.assertEqual(
            exited.event_snapshot.events,
            frozenset({SocialEvent.ROBOT_LOST}),
        )
        self.assertNotIn(SocialEvent.ROBOT_VISIBLE, still_out.event_snapshot.events)
        self.assertIn(SocialEvent.ROBOT_VISIBLE, reentered.event_snapshot.events)

    def test_near_and_personal_space_distance_hysteresis(self):
        near = self.evaluate(None, 0, robot_x=2.5)
        near_band = self.evaluate(near.memory, 1, robot_x=2.7)
        near_exit = self.evaluate(near_band.memory, 2, robot_x=2.8)
        personal = self.evaluate(near_exit.memory, 3, robot_x=1.0)
        personal_band = self.evaluate(personal.memory, 4, robot_x=1.1)
        personal_exit = self.evaluate(personal_band.memory, 5, robot_x=1.2)

        self.assertIn(SocialEvent.ROBOT_NEAR, near.event_snapshot.events)
        self.assertIn(SocialEvent.ROBOT_NEAR, near_band.event_snapshot.events)
        self.assertNotIn(SocialEvent.ROBOT_NEAR, near_exit.event_snapshot.events)
        self.assertIn(
            SocialEvent.PERSONAL_SPACE_VIOLATION,
            personal.event_snapshot.events,
        )
        self.assertIn(
            SocialEvent.PERSONAL_SPACE_VIOLATION,
            personal_band.event_snapshot.events,
        )
        self.assertNotIn(
            SocialEvent.PERSONAL_SPACE_VIOLATION,
            personal_exit.event_snapshot.events,
        )

    def test_fast_approach_hysteresis(self):
        entered = self.evaluate(None, 0, robot_x=5.0, robot_vx=-0.5)
        band = self.evaluate(
            entered.memory, 1, robot_x=5.0, robot_vx=-0.4
        )
        exited = self.evaluate(
            band.memory, 2, robot_x=5.0, robot_vx=-0.35
        )
        self.assertIn(
            SocialEvent.ROBOT_FAST_APPROACH, entered.event_snapshot.events
        )
        self.assertIn(
            SocialEvent.ROBOT_FAST_APPROACH, band.event_snapshot.events
        )
        self.assertNotIn(
            SocialEvent.ROBOT_FAST_APPROACH, exited.event_snapshot.events
        )

    def test_low_ttc_hysteresis(self):
        entered = self.evaluate(None, 0, robot_x=2.0, robot_vx=-1.0)
        band = self.evaluate(
            entered.memory, 1, robot_x=2.65, robot_vx=-1.0
        )
        exited = self.evaluate(
            band.memory, 2, robot_x=2.75, robot_vx=-1.0
        )
        self.assertAlmostEqual(entered.event_snapshot.metrics.ttc_seconds, 1.25)
        self.assertAlmostEqual(band.event_snapshot.metrics.ttc_seconds, 1.9)
        self.assertAlmostEqual(exited.event_snapshot.metrics.ttc_seconds, 2.0)
        self.assertIn(SocialEvent.TTC_LOW, entered.event_snapshot.events)
        self.assertIn(SocialEvent.TTC_LOW, band.event_snapshot.events)
        self.assertNotIn(SocialEvent.TTC_LOW, exited.event_snapshot.events)

    def test_leaving_hysteresis(self):
        entered = self.evaluate(None, 0, robot_x=4.0, robot_vx=0.1)
        band = self.evaluate(
            entered.memory, 1, robot_x=4.0, robot_vx=0.05
        )
        exited = self.evaluate(band.memory, 2, robot_x=4.0, robot_vx=0.0)
        self.assertIn(SocialEvent.ROBOT_LEAVING, entered.event_snapshot.events)
        self.assertIn(SocialEvent.ROBOT_LEAVING, band.event_snapshot.events)
        self.assertNotIn(SocialEvent.ROBOT_LEAVING, exited.event_snapshot.events)

    def test_sudden_near_is_a_safe_entry_edge_only(self):
        outside = self.evaluate(
            None,
            0,
            robot_x=3.0,
            robot_vx=-0.3,
            robot_radius=0.0,
            human_radius=0.0,
        )
        edge = self.evaluate(
            outside.memory,
            1,
            robot_x=2.5,
            robot_vx=-0.3,
            robot_radius=0.0,
            human_radius=0.0,
        )
        repeated = self.evaluate(
            edge.memory,
            2,
            robot_x=2.4,
            robot_vx=-0.3,
            robot_radius=0.0,
            human_radius=0.0,
        )
        self.assertIn(SocialEvent.SUDDEN_NEAR, edge.event_snapshot.events)
        self.assertNotIn(SocialEvent.SUDDEN_NEAR, repeated.event_snapshot.events)

    def test_fast_or_unsafe_near_entry_is_not_sudden_near(self):
        fast = self.evaluate(
            None,
            0,
            robot_x=2.5,
            robot_vx=-0.5,
            robot_radius=0.0,
            human_radius=0.0,
        )
        personal = self.evaluate(None, 1, robot_x=1.0, robot_vx=-0.3)
        self.assertNotIn(SocialEvent.SUDDEN_NEAR, fast.event_snapshot.events)
        self.assertNotIn(
            SocialEvent.SUDDEN_NEAR, personal.event_snapshot.events
        )

    def test_safe_approach_has_configured_upper_bound(self):
        safe = self.evaluate(
            None,
            0,
            robot_x=2.5,
            robot_vx=-0.15,
            robot_radius=0.0,
            human_radius=0.0,
        )
        boundary = self.evaluate(
            None,
            1,
            robot_x=2.5,
            robot_vx=-0.25,
            robot_radius=0.0,
            human_radius=0.0,
        )
        self.assertIn(
            SocialEvent.ROBOT_SAFE_APPROACH, safe.event_snapshot.events
        )
        self.assertNotIn(
            SocialEvent.ROBOT_SAFE_APPROACH, boundary.event_snapshot.events
        )
        self.assertIn(SocialEvent.SUDDEN_NEAR, boundary.event_snapshot.events)

    def test_clock_rollback_clears_every_latch(self):
        memory = EventMemory(
            robot_visible=True,
            robot_near=True,
            personal_space_violation=True,
            robot_fast_approach=True,
            ttc_low=True,
            robot_leaving=True,
            last_stamp_ns=10,
        )
        result = self.evaluate(memory, 9, robot_x=0.5, robot_vx=-1.0)
        self.assertTrue(result.event_snapshot.clock_rollback)
        self.assertEqual(
            result.event_snapshot.events,
            frozenset({SocialEvent.TIME_RESET}),
        )
        self.assertEqual(result.memory, EventMemory(last_stamp_ns=9))
        self.assertEqual(memory.last_stamp_ns, 10)

    def test_equal_timestamp_is_not_a_clock_rollback(self):
        memory = EventMemory(last_stamp_ns=10)
        result = self.evaluate(memory, 10, robot_x=5.0)
        self.assertFalse(result.event_snapshot.clock_rollback)
        self.assertNotIn(SocialEvent.TIME_RESET, result.event_snapshot.events)

    def test_custom_thresholds_are_used(self):
        extractor = EventExtractor(
            EventThresholds(
                visible_enter_distance=3.0,
                visible_exit_distance=3.5,
            )
        )
        result = extractor.evaluate(snapshot(0, robot_x=4.0))
        self.assertNotIn(SocialEvent.ROBOT_VISIBLE, result.event_snapshot.events)


if __name__ == "__main__":
    unittest.main()
