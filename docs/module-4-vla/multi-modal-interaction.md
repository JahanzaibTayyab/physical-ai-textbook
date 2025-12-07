---
sidebar_position: 4
title: "Multi-Modal Interaction"
description: "Combining speech, gesture, and vision for natural interaction"
---

import PersonalizationButton from '@site/src/components/PersonalizationButton';
import TranslationButton from '@site/src/components/TranslationButton';

<PersonalizationButton 
  chapterPath="/docs/module-4-vla/multi-modal-interaction"
  originalContent="sidebar_position: 4"
/>

<TranslationButton 
  chapterPath="/docs/module-4-vla/multi-modal-interaction"
  originalContent="sidebar_position: 4"
/>

# Multi-Modal Interaction

Multi-modal interaction combines multiple input modalities (speech, gesture, vision) for more natural and robust human-robot interaction.

## Combining Modalities

### Multi-Modal Fusion

```python
class MultiModalInterface(Node):
    def __init__(self):
        super().__init__('multi_modal_interface')
        
        # Subscribers for different modalities
        self.voice_sub = self.create_subscription(
            String, '/voice_command', self.voice_callback, 10)
        self.gesture_sub = self.create_subscription(
            String, '/gesture', self.gesture_callback, 10)
        self.vision_sub = self.create_subscription(
            Image, '/camera/image_raw', self.vision_callback, 10)
        
        # Fused command publisher
        self.fused_pub = self.create_publisher(
            String, '/fused_command', 10)
    
    def fuse_modalities(self, voice, gesture, vision):
        # Combine information from all modalities
        if voice and gesture:
            # Voice + gesture = more confident command
            confidence = 0.9
        elif voice:
            confidence = 0.7
        elif gesture:
            confidence = 0.6
        
        return {
            'command': self.interpret(voice, gesture, vision),
            'confidence': confidence
        }
```

## Gesture Recognition

### Using MediaPipe

```python
import mediapipe as mp

class GestureRecognizer(Node):
    def __init__(self):
        super().__init__('gesture_recognizer')
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
    
    def camera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.hands.process(cv_image)
        
        if results.multi_hand_landmarks:
            gesture = self.classify_gesture(results)
            self.publish_gesture(gesture)
    
    def classify_gesture(self, hand_landmarks):
        # Classify gesture (point, wave, grab, etc.)
        # Return gesture type
        pass
```

## Vision-Based Interaction

### Pointing Detection

```python
class PointingDetector(Node):
    def detect_pointing(self, image, hand_landmarks):
        # Detect pointing direction
        index_finger = hand_landmarks[8]  # Index finger tip
        wrist = hand_landmarks[0]
        
        # Calculate pointing vector
        pointing_vector = index_finger - wrist
        
        # Project to 3D using depth
        target_3d = self.project_to_3d(pointing_vector, depth_image)
        
        return target_3d
```

## Unified Command Interface

```python
class UnifiedInterface(Node):
    def process_command(self, voice=None, gesture=None, vision=None):
        # Prioritize based on confidence and context
        if voice and self.is_confident(voice):
            return self.process_voice(voice)
        elif gesture and vision:
            return self.process_pointing(gesture, vision)
        elif voice:
            return self.process_voice(voice)
        else:
            return None
```

## Best Practices

1. **Fuse modalities**: Combine for robustness
2. **Handle conflicts**: Resolve when modalities disagree
3. **Context matters**: Use environment context
4. **Fallback gracefully**: Handle missing modalities

[Next: Capstone Project →](./capstone-project.md)