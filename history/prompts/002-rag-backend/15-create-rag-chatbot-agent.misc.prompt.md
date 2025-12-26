--- 
id: 15
title: Create RAG-powered chatbot agent
stage: misc
date: 2025-12-20
surface: agent
model: gemini-pro
feature: 002-rag-backend
branch: 002-rag-backend
user: 
command: python rag_chatbot_agent.py
labels: ["rag", "chatbot", "python", "openai-agents", "voyageai", "qdrant"]
links:
  spec: specs/002-rag-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
- rag_chatbot_agent.py
- .env
tests: null
---

## Prompt

This is the Gemini CLI. We are setting up the context for our chat.
Today's date is Saturday, December 20, 2025 (formatted according to the user's locale).
My operating system is: win32
The project's temporary directory is: C:\Users\cv\.gemini\tmp\9550a104651418ad616d98f302284874a24ea4ff602319469c3627756b7cb392
I'm currently working in the directory: D:\hackaton-specify\humanoid-book
Here is the folder structure of the current working directories:

Showing up to 200 items (files + folders). Folders or files indicated with ... contain more items not shown, were ignored, or the display limit (200 items) was reached.

D:\hackaton-specify\humanoid-book\
├───.gitignore
├───docusaurus.config.ts
├───final_status.txt
├───GEMINI.md
├───hackaton-specify - Shortcut.lnk
├───package-lock.json
├───package.json
├───rag_chatbot_agent.py
├───README.md
├───sidebars.ts
├───status.txt
├───tsconfig.json
├───types.d.ts
├───.gemini\
│   ├───settings.json
│   └───commands\
│       ├───sp.adr.toml
│       ├───sp.analyze.toml
│       ├───sp.checklist.toml
│       ├───sp.clarify.toml
│       ├───sp.constitution.toml
│       ├───sp.git.commit_pr.toml
│       ├───sp.implement.toml
│       ├───sp.phr.toml
│       ├───sp.plan.toml
│       ├───sp.specify.toml
│       └───sp.tasks.toml
├───.git\...
├───.github\
│   └───workflows\
│       └───deploy.yml
├───.specify\
│   ├───memory\
│   │   └───constitution.md
│   ├───scripts\
│   │   └───powershell\
│   └───templates\
│       ├───adr-template.md
│       ├───agent-file-template.md
│       ├───checklist-template.md
│       ├───phr-template.prompt.md
│       ├───plan-template.md
│       ├───spec-template.md
│       └───tasks-template.md
├───blog\
│   ├───2019-05-28-first-blog-post.md
│   ├───2019-05-29-long-blog-post.md
│   ├───2021-08-01-mdx-blog-post.mdx
│   ├───authors.yml
│   ├───lab1-pub-sub.md
│   ├───tags.yml
│   └───2021-08-26-welcome\
│       ├───docusaurus-plushie-banner.jpeg
│       └───index.md
├───chapters\
├───docs\
│   ├───_category_.json
│   ├───01-gazebo-intro.md
│   ├───01-intro.md
│   ├───01-isaac-intro.md
│   ├───01-vla-intro.md
│   ├───02-nodes-topics.md
│   ├───03-physical-ai-foundations.md
│   ├───04-digital-to-physical-ai.md
│   ├───05-humanoid-robotics-landscape.md
│   ├───06-sensor-systems.md
│   ├───07-ros2-services-actions.md
│   ├───08-ros2-packages-python.md
│   ├───09-ros2-launch-files-parameters.md
│   ├───10-gazebo-setup.md
│   ├───11-urdf-sdf.md
│   ├───12-gazebo-physics-sensors.md
│   ├───13-unity-for-robotics.md
│   ├───14-nvidia-isaac-intro.md
│   ├───15-isaac-ai-perception-manipulation.md
│   ├───16-isaac-reinforcement-learning.md
│   ├───17-isaac-sim-to-real.md
│   ├───18-humanoid-kinematics-dynamics.md
│   ├───19-bipedal-locomotion.md
│   ├───20-humanoid-manipulation.md
│   ├───21-human-robot-interaction.md
│   ├───22-gpt-for-robotics.md
│   ├───23-speech-and-nlu.md
│   ├───24-multimodal-interaction.md
│   ├───lab1-gazebo-urdf.md
│   └───img\
│       ├───docsVersionDropdown.png
│       └───localeDropdown.png
├───i18n\
│   ├───en\
│   │   ├───code.json
│   │   ├───docusaurus-plugin-content-blog\
│   │   ├───docusaurus-plugin-content-docs\
│   │   └───docusaurus-theme-classic\
│   └───ur\
│       ├───code.json
│       ├───docusaurus-plugin-content-blog\
│       ├───docusaurus-plugin-content-docs\
│       └───docusaurus-theme-classic\
├───labs\
├───modules\
├───my-book\
│   ├───.gitignore
│   ├───docusaurus.config.ts
│   ├───package-lock.json
│   ├───package.json
│   ├───README.md
│   ├───sidebars.ts
│   ├───tsconfig.json
│   ├───.docusaurus\...
│   ├───.github\
│   │   └───workflows\
│   ├───blog\
│   │   ├───2019-05-28-first-blog-post.md
│   │   ├───2019-05-29-long-blog-post.md
│   │   ├───2021-08-01-mdx-blog-post.mdx
│   │   ├───authors.yml
│   │   ├───lab1-pub-sub.md
│   │   ├───tags.yml
│   │   └───2021-08-26-welcome\
│   ├───docs\
│   │   ├───_category_.json
│   │   ├───01-gazebo-intro.md
│   │   ├───01-intro.md
│   │   ├───01-isaac-intro.md
│   │   ├───01-vla-intro.md
│   │   ├───02-nodes-topics.md
│   │   ├───lab1-gazebo-urdf.md
│   │   └───img\
│   ├───i18n\
│   │   ├───en\
│   │   └───ur\
│   ├───node_modules\...
│   ├───src\
│   │   ├───components\
│   │   ├───contexts\
│   │   ├───css\
│   │   ├───pages\
│   │   └───theme\
│   └───static\
│       ├───.nojekyll
│       └───img\
├───rag-backend\
│   ├───.gitignore
│   ├───poetry.lock
│   ├───pyproject.toml
│   ├───Quickstart.md
│   ├───rag.db
│   ├───README.md
│   ├───run_ingest.py
│   ├───src\
│   │   ├───__init__.py
│   │   ├───main.py
│   │   ├───__pycache__\
│   │   ├───api\
│   │   ├───core\
│   │   ├───models\
│   │   ├───scripts\
│   │   └───services\
│   └───tests\
├───specs\
│   ├───001-spec-physical-ai-textbook\
│   │   ├───data-model.md
│   │   ├───plan.md
│   │   ├───quickstart.md
│   │   ├───research.md
│   │   ├───spec.md
│   │   ├───tasks.md
│   │   ├───checklists\
│   │   └───contracts\
│   └───002-rag-backend\
│       ├───data-model.md
│       ├───plan.md
│       ├───quickstart.md
│       ├───research.md
│       ├───spec.md
│       ├───tasks.md
│       ├───checklists\
│       └───contracts\
├───src\
│   ├───components\
│   │   ├───Chatbot\
│   │   ├───HomepageFeatures\
│   │   └───PersonalizedSection\
│   ├───contexts\
│   │   └───ChatbotContext.tsx
│   ├───css\
│   │   └───custom.css
│   ├───pages\
│   │   └───chatbot.tsx
│   │   └───...
│   └───theme\
└───static\
    ├───.nojekyll
    └───img\

Reminder: Do not return an empty response when a tool call is required.

My setup is complete. I will provide my first command in the next turn.
You are an AI developer assistant. Your task is to create a RAG-powered chatbot agent for a textbook. 

Instructions:

1. Use **OpenAI Agents Python SDK** to build the agent. 
   - Reference: https://openai.github.io/openai-agents-python/
   - The agent will handle reasoning, chat, and answering user questions. 
   - The model to use for the agent is **Gemini API**. 
   - Do not use Gemini for embeddings.

2. For embeddings:
   - Use **Voyage AI** to convert text chunks from the textbook into vector embeddings.
   - These embeddings will be stored in **Qdrant Cloud** for similarity search.

3. The agent should:
   - Receive a user query via CLI input.
   - Retrieve relevant context from Qdrant using embeddings.
   - Answer strictly based on the retrieved context.
   - If the answer is not found in the selected text, respond with: "Not in selected text".

4. Provide a **Python code example** showing:
   - How to define a retrieval tool (using Voyage embeddings + Qdrant search)
   - How to create the agent using OpenAI Agents SDK
   - How to query the agent with a user question from CLI
   - How to output the answer to CLI

5. Mention clearly in comments which parts are:
   - Agent definition
   - Retrieval function
   - CLI input
   - Response generation

End result: a working Python code template for a CLI-based RAG chatbot using Gemini for reasoning and Voyage AI for embeddings, following best practices from the OpenAI Agents Python documentation.

## Response snapshot

I have created the `rag_chatbot_agent.py` script and a corresponding `.env` file as requested.

To run the chatbot, please follow these steps:

1.  **Update the `.env` file**: Open the `.env` file and replace the placeholder values (`"YOUR_VOYAGE_API_KEY"`, `"YOUR_QDRANT_API_KEY"`, `"YOUR_QDRANT_CLUSTER_URL"`, `"YOUR_GEMINI_API_KEY"`) with your actual API keys and Qdrant cluster URL.

2.  **Run the script**: Execute the script from your terminal using the following command:
    ```bash
    python rag_chatbot_agent.py
    ```

3.  **Interact with the chatbot**: Once the script is running, it will prompt you for a "User Query". Type your question and press Enter. The agent will then retrieve the relevant information and provide an answer. To exit the chatbot, type `exit`.

This script sets up a complete RAG pipeline using Voyage AI for embeddings, Qdrant for the vector store, and a Gemini-powered agent for reasoning, all accessible through a simple command-line interface.

## Outcome

- ✅ Impact: Created a Python script for a RAG-powered chatbot agent and a corresponding .env file.
- 🧪 Tests: No tests were created.
- 📁 Files: 2 files created: rag_chatbot_agent.py, .env
- 🔁 Next prompts: null
- 🧠 Reflection: The user's request was clear and I was able to fulfill it by creating the Python script and the .env file. I also provided instructions on how to run the script.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
