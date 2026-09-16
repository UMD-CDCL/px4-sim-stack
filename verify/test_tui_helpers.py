import unittest

from scripts.tui import Action, complete_text, wrap_output
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


if __name__ == "__main__":
    unittest.main()
