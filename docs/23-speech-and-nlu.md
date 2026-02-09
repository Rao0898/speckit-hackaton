---
sidebar_position: 23
---

# Speech Recognition and Natural Language Understanding (NLU)

For a robot to engage in a spoken conversation, it must first be able to convert the sound waves of human speech into a structured understanding of the user's intent. This process is a pipeline with two main stages: **Automatic Speech Recognition (ASR)** and **Natural Language Understanding (NLU)**.

## 1. Automatic Speech Recognition (ASR)

ASR is the technology that converts spoken audio into text. It is the robotic equivalent of hearing.

### How ASR Works

Modern ASR systems are based on deep learning. They are trained on massive datasets of audio recordings and their corresponding human-verified transcripts. The model learns to map the complex patterns in the audio signal (phonemes, accents, intonations) to words and sentences.

- **Input**: An audio stream, typically from a microphone array on the robot.
- **Output**: A string of text.

### ASR in a Robotic Context

For a robot, ASR is more challenging than for a smartphone assistant.
- **Noise**: The robot's own motors and fans generate noise, which can interfere with the microphones.
- **Distance**: The user may be speaking to the robot from across the room, which can result in a faint or reverberant signal.
- **Microphone Arrays**: To combat these issues, robots are often equipped with a microphone array. By using signal processing techniques (like **beamforming**), the robot can focus its "hearing" in the direction of the person speaking and filter out background noise.

Popular ASR toolkits include NVIDIA Riva, and open-source models like Whisper from OpenAI.

## 2. Natural Language Understanding (NLU)

Once the user's speech has been converted to text, the NLU system must figure out what that text *means*. NLU is the process of extracting intent and entities from a piece of text.

- **Input**: A string of text (e.g., "robot, please bring me the red cup from the kitchen table").
- **Output**: A structured representation of the user's intent.

### Key Components of NLU

1.  **Intent Classification**: The NLU model first tries to determine the user's overall goal. In the example above, the intent is `TransportObject`.

2.  **Entity Extraction** (also called Slot Filling): The model then identifies the key pieces of information (the entities or slots) associated with that intent.
    - `object_to_transport`: "the red cup"
    - `source_location`: "the kitchen table"
    - `destination_location`: "me" (i.e., the user's current location)

The final output of the NLU system might look something like this in a JSON format:
```json
{
  "intent": "TransportObject",
  "entities": {
    "object_to_transport": {
      "name": "cup",
      "color": "red"
    },
    "source_location": {
      "name": "table",
      "room": "kitchen"
    },
    "destination_location": "user"
  }
}
```

This structured data can then be passed to the robot's task planner, which knows how to execute a `TransportObject` routine.

## The Rise of LLMs in NLU

Traditionally, NLU systems required training a custom machine learning model for a specific set of intents and entities. You had to provide many examples for each intent you wanted the robot to understand.

Large Language Models (LLMs) have dramatically changed this landscape. With a carefully crafted prompt, you can now use a powerful LLM (like GPT-4) to perform NLU in a "zero-shot" or "few-shot" manner, without any specific training.

You can provide the LLM with a prompt that describes the desired intents and entities and give it the user's utterance. The LLM is often capable of correctly parsing the text into the desired structured format on its first try. This makes developing conversational interfaces much faster and more flexible, as you can easily add new capabilities just by updating the prompt.

Whether using a traditional NLU model or a modern LLM, the goal is the same: to transform the beautiful, messy, and ambiguous nature of human language into the precise, structured commands that a robot needs to take action.
