# Version Management

* `main` always contains **only the latest stable/current framework**.
* When developing a new version, build on the latest `main` version and keep only the files required by the new architecture.
* **Do not keep obsolete files from older versions** in `main` if they are no longer required by the current framework.
* Older versions are preserved through **Git history and version tags** (for example `v0.1`, `v3.0`, `v9.0`).
* Before removing or replacing major components, verify that the new framework builds and runs correctly.
* After completing a stable version:

  1. Update `main` with the latest framework.
  2. Remove obsolete files that are no longer required.
  3. Create a version tag such as `v11.0`, `v12.0`, etc.
  4. Push the tag to GitHub.
* Never copy old-version files into `main` just for backup. Git history/tags are the backup.

**Rule:** `main = latest stable framework only. Older versions = Git tags/history.`
