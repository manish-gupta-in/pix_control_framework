# PIX Control Framework (PCF) - Developer & Git Workflow Guide

This document preserves the full context of the project and provides step-by-step instructions on how to restore, modify, and upload the codebase if the local directory is ever deleted.

## 1. Project Context
**Project Name:** PIX Control Framework (PCF)  
**Repository:** https://github.com/manish-gupta-in/pix_control_framework  
**Maintainer:** Manish Gupta  

**Objective:**
PCF is a completely modular, vendor-independent autonomous vehicle control framework. It provides a clean, scalable architecture separating autonomous driving algorithms from hardware-specific implementations (similar to Autoware or Apollo, but highly modular).

**Architecture Layers:**
1. **Algorithm API:** High-level API for external controllers.
2. **Command Manager:** Command arbitration and prioritization.
3. **State Manager:** Maintains Init, Manual, Autonomous, and Fault states.
4. **Safety Manager:** Monitors limits and triggers Emergency Stops (AEB).
5. **DBW Manager:** Translates standard commands to platform-specific interfaces.
6. **Vehicle Interface & CAN Stack:** Hardware-specific DBC decoding/encoding and SocketCAN drivers.

---

## 2. How to Restore the Project (If Deleted)
If you accidentally delete your local folder, you have two ways to get it back:

### Option A: The Git Clone Method (Highly Recommended)
This is the best method because it automatically keeps the `.git` tracking history.
1. Open your terminal in the folder where you want to keep the project (e.g., `~/Desktop/Push`).
2. Run the clone command:
   ```bash
   git clone https://github.com/manish-gupta-in/pix_control_framework.git
   ```
3. Enter the folder:
   ```bash
   cd pix_control_framework
   ```

### Option B: The ZIP Download Method
If you downloaded the `.zip` file directly from the GitHub website instead of cloning:
1. Extract the ZIP file.
2. Open a terminal inside the extracted folder.
3. Because downloading a ZIP removes the Git history, you must re-initialize it and link it back to your GitHub repository to be able to push again:
   ```bash
   git init
   git remote add origin https://github.com/manish-gupta-in/pix_control_framework.git
   git branch -M main
   
   # Optional: To make sure your local folder syncs exactly with the online version
   git fetch
   git reset --mixed origin/main
   ```

---

## 3. Daily Git Workflow (How to Push Changes)
Whenever you modify files, add new code, or want to save your daily progress, follow these exact steps inside your terminal:

**1. Check what changed:**
```bash
git status
```

**2. Stage all changes:**
```bash
git add .
```

**3. Commit the changes (Save locally):**
```bash
git commit -m "Describe what you changed here, e.g., Updated DBW logic"
```

**4. Push the changes (Upload to GitHub):**
```bash
git push origin main
```
*(Note: Because you are using HTTPS, it will prompt you for your GitHub username and your Personal Access Token).*
