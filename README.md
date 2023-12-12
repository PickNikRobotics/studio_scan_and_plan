# MoveIt Studio Scanning Robot configuration

This package contains descriptions, and configurations for a UR10e to a table so scan objects with Moveit Studio.
Developers can switch between different robot configurations by setting the `STUDIO_CONFIG_PACKAGE` environment variable.

Valid configurations included in this package:
```
# Simulate UR10e with Gazebo
STUDIO_CONFIG_PACKAGE="ur10e_gazebo_scan_and_plan_config"
# Real hardware
STUDIO_CONFIG_PACKAGE="ur10e_scan_config"
```

<img src="https://picknik.ai/assets/images/logo.jpg" width="100">

[![Format Status](https://github.com/PickNikRobotics/studio_scan_and_plan/actions/workflows/format.yaml/badge.svg)](https://github.com/PickNikRobotics/studio_scan_and_plan/actions/workflows/format.yaml)

---

## Installation

### Git LFS

This repo uses **[git-lfs](https://git-lfs.github.com)** to store large binary files. It is installed and activated with:

```shell
sudo apt install git-lfs
git lfs install
```

If you are retroactively adding LFS after having cloned the repository, run `git lfs pull` to update the repo.

If you are working with an old branch with differing LFS settings, you can run `git lfs uninstall` to temporarily disable it.

---

## Build a Docker dev container
First, clone MoveIt Studio and set up a dev container.

```shell
# Set to the version of MoveIt Studio you want to use
STUDIO_DOCKER_TAG=2.11.0

# Set to the location of this repo
STUDIO_HOST_USER_WORKSPACE=/path/to/scan_config_ws

# Set to the config package you want to run
STUDIO_CONFIG_PACKAGE=ur10e_gazebo_scan_and_plan_config
```

Then, build the image.

```shell
docker compose build dev
```

Then, start the container with the following.

```shell
docker compose up dev
```

You can now enter the container through an interactive shell and do your work.

```shell
docker compose exec -it dev bash
```

---
