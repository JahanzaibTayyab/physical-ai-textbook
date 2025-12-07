---
sidebar_position: 2
title: "Voice-to-Action with Whisper"
description: "Using OpenAI Whisper for voice command recognition"
---

import PersonalizationButton from '@site/src/components/PersonalizationButton';
import TranslationButton from '@site/src/components/TranslationButton';

<PersonalizationButton 
  chapterPath="/docs/module-4-vla/voice-to-action"
  originalContent="sidebar_position: 2"
/>

<TranslationButton 
  chapterPath="/docs/module-4-vla/voice-to-action"
  originalContent="sidebar_position: 2"
/>

# Voice-to-Action with Whisper

OpenAI Whisper provides accurate speech recognition that we can use to convert voice commands into robot actions.

## Installing Whisper

```bash
pip install openai-whisper
# Or with UV
uv add openai-whisper
```

## Basic Usage

```python
import whisper

# Load model
model = whisper.load_model("base")

# Transcribe audio
result = model.transcribe("audio.wav")
print(result["text"])
```

## ROS 2 Integration

### Voice Command Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import pyaudio
import wave

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.model = whisper.load_model("base")
        self.command_pub = self.create_publisher(
            String,
            '/voice_command',
            10
        )
        self.setup_audio()
    
    def setup_audio(self):
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )
    
    def listen_and_transcribe(self):
        # Record audio
        frames = []
        for _ in range(0, int(16000 / 1024 * 2)):  # 2 seconds
            data = self.stream.read(1024)
            frames.append(data)
        
        # Save to file
        wf = wave.open("temp_audio.wav", 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(self.audio.get_sample_size(pyaudio.paInt16))
        wf.setframerate(16000)
        wf.writeframes(b''.join(frames))
        wf.close()
        
        # Transcribe
        result = self.model.transcribe("temp_audio.wav")
        command = result["text"]
        
        # Publish command
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        
        self.get_logger().info(f'Heard: {command}')
```

## Command Processing

```python
class CommandProcessor(Node):
    def __init__(self):
        super().__init__('command_processor')
        self.subscription = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10
        )
    
    def command_callback(self, msg):
        command = msg.data.lower()
        
        if "move forward" in command:
            self.move_forward()
        elif "turn left" in command:
            self.turn_left()
        elif "pick up" in command:
            self.pick_up_object()
        # ... more commands
```

## Best Practices

1. **Use appropriate model size**: Base for speed, Large for accuracy
2. **Handle noise**: Use noise reduction
3. **Validate commands**: Confirm before executing
4. **Handle errors**: Graceful fallback for unclear commands

[Next: Cognitive Planning →](./cognitive-planning.md)