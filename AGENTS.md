# AGENTS.md

This file tells AI coding agents (Claude Code, Copilot, Cursor, Codex, etc.) how to work in this repository. It is written for an **Applied Programming** course project: a web or mobile app designed and built by a student. Agents must follow these rules for every change, large or small.

> **Students:** Fill in the "Project Details" section below before you start using agents. The more specific you are, the better your agent will follow your architecture.

---

## 1. Project Details (student fills this in)

- **App name:** Wifi Controller
- **One-sentence description:** A web app that runs on an esp32 devkit v1 that controls the movement of an omnidirectional robot.
- **Platform:** Web
- **Language(s):** C/C++
- **Frameworks:** Ardunio Framework for ESP32
- **Data storage:** local device storage
- **Test framework(s):** to be determined
- **Command to run the app:** n/a
- **Command to run all tests:** n/a
- **Command to run a single test file:** n/a
- **Command to run the linter/formatter:** n/a

---

## 2. How Agents Should Behave in This Project

This is a learning environment. The student is responsible for understanding every line of code in the project. Agents must support that.

1. **Work in small steps.** Make one focused change at a time. Prefer several small, reviewable changes over one large one.
2. **Explain your reasoning.** After each change, briefly state what you changed, which layer it belongs to, and why.
3. **Ask before big decisions.** Adding a new dependency, changing the database schema, restructuring folders, or introducing a new pattern requires the student's approval first.
4. **Do not invent requirements.** If behavior is unclear, ask the student instead of guessing.
5. **Never hide failures.** If tests fail, a command errors, or you are unsure something works, say so plainly.
6. **Keep secrets out of code.** API keys, passwords, and tokens go in environment variables or config files that are excluded by `.gitignore`. Never commit them.
7. **Ask before making any changes any files** Do not change any part of the code without the user's explicit permission. Have the user write most of the changes. You may suggest what to do, provide the code, and help plan. But, do not start changing files unless the user tells you that you can. You may ask the user if they want you to start making changes.

---