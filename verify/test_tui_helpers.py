import unittest

from scripts.tui import (ACTIONS, VERIFICATION_STAGES, Action, Console, FRONT_DOOR, complete_text,
                          state_watch_command, wrap_output)
from scripts.state import service_lifecycle


class TuiHelpers(unittest.TestCase):
    def test_completion_cycles_matching_tokens(self):
        self.assertEqual(complete_text("sc", ["scene", "scenario"]),
                         ("scene", 1))
        self.assertEqual(complete_text("sc", ["scene", "scenario"], 1),
                         ("scenario", 0))

    def test_completion_preserves_command_prefix(self):
        self.assertEqual(complete_text("uas sc", ["scene", "scenario"])[0],
                         "uas scene")

    def test_wrap_output_handles_empty_and_long_lines(self):
        self.assertEqual(wrap_output([""], 4), [""])
        self.assertEqual(wrap_output(["abcdefgh"], 4), ["abcd", "efgh"])

    def test_cancel_menu_action_is_a_runner_action(self):
        action = Action("stack", "", "stop the running command",
                        ("__cancel_running__",))
        self.assertEqual(action.command, ("__cancel_running__",))

    def test_service_lifecycle_distinguishes_startup_and_failure(self):
        self.assertEqual(service_lifecycle({"state": "absent"}), "not_started")
        self.assertEqual(service_lifecycle({"state": "running", "health": "starting"}),
                         "starting")
        self.assertEqual(service_lifecycle({"state": "running", "health": "unhealthy"}),
                         "failed")
        self.assertEqual(service_lifecycle({"state": "exited", "exit_code": 0}),
                         "stopped")

    def test_ui_requests_a_fresh_report_without_changing_the_watch_period(self):
        self.assertEqual(state_watch_command(2.0), [
            str(FRONT_DOOR),
            "state", "--watch", "2.0", "--initial-delay", "0"])

    def test_verification_is_selected_by_stage_and_checks_are_menu_only(self):
        verify = next(action for action in ACTIONS
                      if action.scope == "stack" and action.label == "run one verification stage")
        self.assertEqual(verify.command, ("verify", "{value}"))
        self.assertEqual(verify.ask.choices, VERIFICATION_STAGES)
        self.assertIn("contract", VERIFICATION_STAGES)
        self.assertIn("foxglove", VERIFICATION_STAGES)

    def test_command_candidates_include_front_door_and_live_config_values(self):
        ui = object.__new__(Console)
        ui.rows = type("Rows", (), {
            "config": {"scene": "uroc", "scenario": "uroc_casualties",
                       "models": "chimera_v3", "streams": "rgb11",
                       "zoom_presets": "wide narrow"},
            "vehicles": [{"n": 11}],
        })()
        candidates = Console.command_candidates(ui)
        self.assertIn("verify", candidates)
        self.assertIn("uroc_casualties", candidates)
        self.assertIn("chimera_v3", candidates)
        self.assertIn("11", candidates)

    def test_rows_retain_detection_summary_by_vehicle(self):
        from scripts.tui import Rows
        rows = Rows({"detections": [{"number": 11, "state": "live", "boxes": 4}]})
        self.assertEqual(rows.detections[11]["boxes"], 4)
        for label in ("check the front door and the docs", "check the host",
                      "where the Foxglove layout lives"):
            action = next(action for action in ACTIONS if action.label == label)
            self.assertEqual(action.key, "")


if __name__ == "__main__":
    unittest.main()
