import unittest
from scripts.llm_robot_brain import OpenAIProvider, AnthropicProvider, OllamaProvider

class TestLLMProviders(unittest.TestCase):

    def setUp(self):
        self.openai_provider = OpenAIProvider()
        self.anthropic_provider = AnthropicProvider()
        self.ollama_provider = OllamaProvider()

    def test_openai_response_format(self):
        response = self.openai_provider.get_response("Test input")
        self.assertIn("thinking", response)
        self.assertIn("actions", response)
        self.assertIn("response", response)

    def test_anthropic_response_format(self):
        response = self.anthropic_provider.get_response("Test input")
        self.assertIn("thinking", response)
        self.assertIn("actions", response)
        self.assertIn("response", response)

    def test_ollama_response_format(self):
        response = self.ollama_provider.get_response("Test input")
        self.assertIn("thinking", response)
        self.assertIn("actions", response)
        self.assertIn("response", response)

if __name__ == '__main__':
    unittest.main()