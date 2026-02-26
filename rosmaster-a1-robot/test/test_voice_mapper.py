import unittest
from scripts.voice_mapper import VoiceMapper

class TestVoiceMapper(unittest.TestCase):

    def setUp(self):
        self.voice_mapper = VoiceMapper()

    def test_initialization(self):
        self.assertIsNotNone(self.voice_mapper)

    def test_voice_command_processing(self):
        command = "Start exploration"
        response = self.voice_mapper.process_voice_command(command)
        self.assertIn("Starting exploration", response)

    def test_invalid_command(self):
        command = "Invalid command"
        response = self.voice_mapper.process_voice_command(command)
        self.assertIn("Command not recognized", response)

    def test_navigation_functionality(self):
        self.voice_mapper.start_navigation()
        self.assertTrue(self.voice_mapper.is_navigating)

    def test_slam_functionality(self):
        self.voice_mapper.start_slam()
        self.assertTrue(self.voice_mapper.is_slam_active)

if __name__ == '__main__':
    unittest.main()