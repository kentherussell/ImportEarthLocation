# Import Earth coding conventions

- Write professional, idiomatic, human-readable C# and follow nearby repository conventions.
- Use meaningful, concise names; retain conventional mathematical notation where it helps clarity.
- Prefer straightforward implementations without unnecessary abstractions, boilerplate, or repetitive comments.
- Explain non-obvious mathematical assumptions, coordinate systems, units, numerical limits, and ECS ownership/lifecycle decisions when needed. Verify explanations against the code and authoritative specification; do not speculate.
- Preserve hot-path performance characteristics, including allocation, iteration, and synchronization behavior.
- Keep formatting edits mechanical, minimal, and limited to code that needs them. Do not reorder declarations or move code as incidental cleanup.
- Avoid unrelated refactoring. Preserve serialized, reflection-sensitive, and native ECS contracts unless the requested feature explicitly requires a verified change.
- Respect task-specific scope and existing exceptions. Preserve unrelated work and report existing verification failures separately from new regressions.

For conservative cleanup tasks, change only whitespace, accurate comments, and
obviously safe local names within the authorized scope. Preserve executable
behavior, structure, formulas, operation ordering, constants, logging, and tests.
Do not deploy or publish unless explicitly authorized for the current task.

These cleanup restrictions are not a ban on feature engineering: an explicitly
requested feature may require new methods, structures, or algorithms. Keep those
changes scoped to that feature and verify them proportionately.
