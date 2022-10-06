# ATLAS Humanoid Robot - A case study

This repository is a case study of the ATLAS humanoid robot. More information are yet to be added.

## Prerequisites

| Software | Version  |
| -------- | -------- |
| Python   | >=3.8.0  |
| pip      | >=22.0.0 |

## Setup

To setup the repository, run the following commands:

```bash
# Clone the repository
git clone <git-repo-id>

# Install the dependencies
cd <repo-name>
python3 -m pip install poetry
poetry install
```

| Note: Any desired development dependencies can be added to the `pyproject.toml` file.

## Usage

To spawn the pybullet environment,

```bash
# To spawn robot with empty environment
python3 python/entrypoint.py empty

# To spawn robot with botlab environment
python3 python/entrypoint.py
```
