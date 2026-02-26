from enum import Enum
import json
import requests

class OpenAIProvider:
    def __init__(self, api_key):
        self.api_key = api_key

    def query(self, prompt):
        headers = {
            'Authorization': f'Bearer {self.api_key}',
            'Content-Type': 'application/json'
        }
        data = {
            'model': 'gpt-4',
            'messages': [{'role': 'user', 'content': prompt}]
        }
        response = requests.post('https://api.openai.com/v1/chat/completions', headers=headers, json=data)
        return response.json()

class AnthropicProvider:
    def __init__(self, api_key):
        self.api_key = api_key

    def query(self, prompt):
        headers = {
            'Authorization': f'Bearer {self.api_key}',
            'Content-Type': 'application/json'
        }
        data = {
            'model': 'claude-v1',
            'prompt': prompt
        }
        response = requests.post('https://api.anthropic.com/v1/complete', headers=headers, json=data)
        return response.json()

class OllamaProvider:
    def __init__(self, model_name):
        self.model_name = model_name

    def query(self, prompt):
        response = requests.post(f'http://localhost:11434/generate', json={'model': self.model_name, 'prompt': prompt})
        return response.json()

class LLMProvider(Enum):
    OPENAI = "openai"
    ANTHROPIC = "anthropic"
    OLLAMA = "ollama"

class LLMRobotBrain:
    def __init__(self, provider_type, api_key=None, model_name=None):
        if provider_type == LLMProvider.OPENAI:
            self.provider = OpenAIProvider(api_key)
        elif provider_type == LLMProvider.ANTHROPIC:
            self.provider = AnthropicProvider(api_key)
        elif provider_type == LLMProvider.OLLAMA:
            self.provider = OllamaProvider(model_name)
        else:
            raise ValueError("Unsupported LLM provider")

    def get_response(self, prompt):
        response = self.provider.query(prompt)
        return self.parse_response(response)

    def parse_response(self, response):
        if 'choices' in response:
            return json.loads(response['choices'][0]['message']['content'])
        return {"thinking": "", "actions": [], "response": "No valid response"}

# Example usage
if __name__ == "__main__":
    llm_brain = LLMRobotBrain(LLMProvider.OPENAI, api_key='your_openai_api_key')
    prompt = "What is the weather like today?"
    response = llm_brain.get_response(prompt)
    print(response)