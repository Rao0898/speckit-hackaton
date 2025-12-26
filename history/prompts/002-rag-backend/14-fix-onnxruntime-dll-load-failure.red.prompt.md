---
id: 14
title: Fix onnxruntime DLL Load Failure
stage: red
date: 2025-12-18
surface: agent
model: gemini
feature: 002-rag-backend
branch: 002-rag-backend
user: user
command: ... ImportError: DLL load failed while importing onnxruntime_pybind11_state ...
labels: ["onnxruntime", "dll-error", "fastembed", "windows", "dependencies"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - none
tests:
 - none
---

## Prompt

<The user provides a long traceback indicating that even though `fastembed` was installed, the application fails to start with the error: `ImportError: DLL load failed while importing onnxruntime_pybind11_state: The specified module could not be found.`>

## Response snapshot

The `fastembed` package is correctly installed, but its dependency, `onnxruntime`, is failing to load a required system library (DLL) on Windows. This is almost always caused by a missing Microsoft Visual C++ Redistributable. The user was instructed to download and install the X64 version of the latest VC++ Redistributable from the official Microsoft website and then restart their computer before trying to run the application again.

## Outcome

- ✅ Impact: Diagnosed a deep, system-level dependency issue (`onnxruntime` missing VC++ redistributable) and provided a precise solution with a link to the official download. This should resolve the final blocker preventing the application from running.
- 🧪 Tests: none
- 📁 Files: none
- 🔁 Next prompts: Restart the PC, then run `poetry run uvicorn src.main:app --reload --port 8000` from the `rag-backend` directory.
- 🧠 Reflection: When a Python package with C++ bindings fails on `ImportError` on Windows, especially with a "DLL load failed" message, the first suspect should always be a missing Visual C++ Redistributable. Providing a direct link to the official installer is the most effective solution.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): none
