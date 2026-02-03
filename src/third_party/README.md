# Third-Party Dependencies

This directory contains external dependencies managed as Git submodules.

## LeRobot

**Location**: `lerobot/`
**Repository**: https://github.com/anh0001/lerobot.git
**Purpose**: Vision-Language-Action (VLA) model framework for robot learning

### Usage

LeRobot is used for:
- OpenVLA model inference for manipulation policies
- Pre-trained model loading and inference
- Future fine-tuning of VLA models on custom datasets

### Installation

From the root of MobileManipulationCore:

```bash
# Install lerobot dependencies
cd src/third_party/lerobot
pip install -e .
```

### Updating LeRobot

To update to the latest version from the fork:

```bash
cd src/third_party/lerobot
git pull origin main
cd ../../..
git add src/third_party/lerobot
git commit -m "Update lerobot submodule"
```

### Fine-tuning Workflow

When fine-tuning VLA models:

1. Make changes in `src/third_party/lerobot/`
2. Test your modifications
3. Commit changes within the submodule:
   ```bash
   cd src/third_party/lerobot
   git checkout -b feature/your-finetuning-work
   git add .
   git commit -m "Add fine-tuning modifications"
   git push origin feature/your-finetuning-work
   ```
4. Update the parent repository to reference the new commit:
   ```bash
   cd ../../..
   git add src/third_party/lerobot
   git commit -m "Update lerobot with fine-tuning changes"
   ```

## For Collaborators

When cloning this repository for the first time:

```bash
git clone <repository-url>
cd MobileManipulationCore

# Initialize and update all submodules
git submodule init
git submodule update --recursive
```

Or clone with submodules in one command:

```bash
git clone --recursive <repository-url>
```
