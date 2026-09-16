import unittest

from scripts.tui import complete_text, wrap_output


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


if __name__ == "__main__":
    unittest.main()
