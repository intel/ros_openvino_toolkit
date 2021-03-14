# Intel ros_openvino_toolkit Contributing Guide

Hi! I'm really excited that you are interested in contributing to ros_openvino_toolkit. Before submitting your contribution, please make sure to take a moment and read through the following guidelines:

- [Code of Conduct](https://github.com/intel/ros_openvino_toolkit/tree/dev-ov2020.3/.github/CODE_OF_CONDUCT.md)
- [Issue Reporting Guidelines](#issue-reporting-guidelines)
- [Pull Request Guidelines](#pull-request-guidelines)
- [Development Setup](#development-setup)
- [Project Structure](#project-structure)

## Issue Reporting Guidelines

- Always use [BEFORE_YOU_START.md](https://github.com/intel/ros_openvino_toolkit/tree/dev-ov2020.3/.github/BEFORE_YOU_START.md) to create new issues.
- [Issue template](https://github.com/intel/ros_openvino_toolkit/tree/dev-ov2020.3/.github/ISSUE_TEMPLATE.md)

## Pull Request Guidelines

- The `master` branch is just a snapshot of the latest stable release. All development should be done in dedicated branches. **Do not submit PRs against the `master` branch.**

- Checkout a topic branch from the relevant branch, e.g. `dev`, and merge back against that branch.

- It's OK to have multiple small commits as you work on the PR - GitHub will automatically squash it before merging.

- If adding a new feature:
  - Add accompanying test case.
  - Provide a convincing reason to add this feature. Ideally, you should open a suggestion issue first and have it approved before working on it.

- If fixing bug:
  - If you are resolving a special issue, add `(fix #xxxx[,#xxxx])` (#xxxx is the issue id) in your PR title for a better release log, e.g. `update entities encoding/decoding (fix #3899)`.
  - Provide a detailed description of the bug in the PR. Live demo preferred.
  - Add appropriate test coverage if applicable.

- [Pull Request template](https://github.com/intel/ros_openvino_toolkit/tree/dev-ov2020.3/.github/PULL_REQUEST_TEMPLATE.md)

## Development Setup

You will need [install openvino 2020.3](https://software.intel.com/content/www/us/en/develop/tools/openvino-toolkit.html) **version 2020.3.341**, [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) .

### install openvino 2020.3
``` bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://apt.repos.intel.com/openvino/2020/GPG-PUB-KEY-INTEL-OPENVINO-2020 |sudo apt-key add -
echo "deb https://apt.repos.intel.com/openvino/2020 all main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2020.list
sudo apt update
sudo apt-cache search openvino
sudo apt-get install -y \
  intel-openvino-runtime-ubuntu18-2020.3.341 \
  intel-openvino-ie-samples-2020.3.341 \
  intel-openvino-omz-dev-2020.3.341 \
  intel-openvino-omz-tools-2020.3.341 \
  intel-openvino-gstreamer-rt-ubuntu-bionic-2020.3.341 \
  intel-openvino-gva-dev-ubuntu-bionic-2020.3.341 \
  intel-openvino-gva-rt-ubuntu-bionic-2020.3.341 \
  intel-openvino-ie-bin-python-tools-ubuntu-bionic-2020.3.341 \
  intel-openvino-ie-rt-core-ubuntu-bionic-2020.3.341 \
  intel-openvino-ie-rt-cpu-ubuntu-bionic-2020.3.341 \
  intel-openvino-ie-rt-gna-ubuntu-bionic-2020.3.341 \
  intel-openvino-ie-rt-gpu-ubuntu-bionic-2020.3.341 \
  intel-openvino-ie-rt-hddl-ubuntu-bionic-2020.3.341 \
  intel-openvino-ie-rt-vpu-ubuntu-bionic-2020.3.341 \
  intel-openvino-ie-sdk-ubuntu-bionic-2020.3.341 \
  intel-openvino-opencv-lib-ubuntu-bionic-2020.3.341
sudo apt-get install -y libgflags-dev
ls -lh /opt/intel/openvino
source /opt/intel/openvino/bin/setupvars.sh
```

### build ros openvino toolkit
```bash
mkdir -p ~/catkin_ws/src
cp -rf ${GITHUB_WORKSPACE} ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/intel/object_msgs.git
cd ~/catkin_ws/
source /opt/ros/melodic/setup.bash
source /opt/intel/openvino/bin/setupvars.sh
export CPU_EXTENSION_LIB+=/opt/intel/openvino_2020.3.341/deployment_tools/inference_engine/lib/intel64/libinference_engine.so
export GFLAGS_LIB+=/usr/lib/x86_64-linux-gnu/libgflags_nothreads.a
env
catkin_make
```

### Committing Changes

Commit messages should follow the [commit message convention](./COMMIT_CONVENTION.md) so that changelogs can be automatically generated. Commit messages will be automatically validated upon commit. If you are not familiar with the commit message convention, you can use `npm run commit` instead of `git commit`, which provides an interactive CLI for generating proper commit messages.


## Project Structure

- TODO

## Credits

Thank you to all the people who have already contributed to ros_openvino_toolkit!

<a href="https://github.com/intel/ros_openvino_toolkit/graphs/contributors">
