# 2026 Season Robot Code

A repository containing robot code for 5690's 2026 season.

## Table of Contents

- [2026 Season Robot Code](#2026-season-robot-code)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [2027 Port Notes](#2027-port-notes)
  - [Can IDs](#can-ids)
  - [Network Map](#network-map)
  - [Button Bindings](#button-bindings)
    - [Xbox Drive Controller](#xbox-drive-controller)
  - [Autos](#autos)
  - [Making Changes](#making-changes)
    - [Creating Issues](#creating-issues)
    - [Creating Branches](#creating-branches)
    - [Creating Commits](#creating-commits)
    - [Creating Pull Requests](#creating-pull-requests)
    - [Creating Releases](#creating-releases)

## Introduction

This is a guide to using and updating 5690's season code for our 2026 Command Robot. 

## 2027 Port Notes

This branch ports the 2026 code to WPILib `2027.0.0-alpha-7` for Systemcore, using the Commands v3 project layout. It does not change robot behavior beyond what the new APIs require.

- **Versions:** GradleRIO `2027.0.0-alpha-7` (Java 25, Gradle 9.4.1), Commands v3, REVLib `2027.0.0-alpha-8`, Phoenix 6 `26.70.0-alpha-2`, DogLog `2027.3.0`, and PhotonLib `dev-v2027.0.0-alpha-2-69-g71416112`.
- **PhotonLib:** the vendordep is PhotonVision's latest development build, which targets WPILib alpha-7 even though it is labeled alpha-2. The released `v2027.0.0-alpha-2` targets WPILib alpha-5 and will not work, so do not use VS Code's "check for updates" on this vendordep: its update URL points to that release.
- **Layout:** `Robot` extends `OpModeRobot` and holds what used to be in `RobotContainer`. Subsystems are Commands v3 mechanisms in `mechanisms/`, constants are in `constants/`, and the teleop and auto opmodes are in `opmodes/`. The button bindings are active while the `DriverTeleop` opmode is selected.
- **Autos:** each auto that was in the SendableChooser is now an autonomous opmode, picked on the Driver Station.
- **CAN bus:** every device uses `CANConstants.kCanPort`, which is set to Systemcore port `CAN_S0`. Change it if the bus is wired to a different port.
- **REV units:** REVLib 2027 removed SPARK conversion factors. The swerve module converts between native units and meters/radians in code, and its PID and feed forward gains are scaled to match the old conversion factors.
- **Not yet available:** PathPlannerLib has no WPILib alpha-7 or Commands v3 build. The PathPlanner autos are commented out with TODOs until it does. The `.path` and `.auto` files are unchanged.

## Can IDs

All devices connected to the CAN bus along with their corresponding CAN IDs

| Device                     | CAN ID |
| -------------------------- | ------ |
| Front Right Drive Motor    | 1      |
| Front Left Drive Motor     | 10     |
| Rear Left Drive Motor      | 14     |
| Rear Right Drive Motor     | 2      |
| Front Right Turn Motor     | 62     |
| Front Left Turn Motor      | 11     |
| Rear Left Turn Motor       | 15     |
| Rear Right Turn Motor      | 3      |
| Pigeon Gyro                | 13     |
| Turret Turning Motor       | 18     |
| Turret Shooter Motor       | 5      |
| Shooter Hood Motor         | 4      |
| Agitator Motor             | 9      |
| Roller Motor               | 12     |
| First Intake Deploy Motor  | 13     |
| Second Intake Deploy Motor | 8      |
| Intake Motor               | 7      |
| Climber Motor              | 17     |

## Network Map

All devices connected to the robot's local network along with each device's assigned IP address

| Device             | IP          |
| ------------------ | ----------- |
| Gateway            | 10.56.90.1  |
| RoboRio            | 10.56.90.2  |
| Vision Coprocessor | 10.56.90.10 |

## Button Bindings

Button bindings for the devices used to control the robot

### Xbox Drive Controller

| Button/Axis   | Action                                     |
| ------------- | ------------------------------------------ |
| Left Stick X  | Robot translation along the field's X axis |
| Left Stick Y  | Robot translation along the field's Y axis |
| Right Stick X | Robot rotation                             |

## Autos

Auto names along with their actions will be listed here.

## Making Changes

This is a guide to the development cycle of this repository. This should apply to anyone interested in making changes to this season's robot code.

### Creating Issues

Issues describe either bugs or errors within code/documentation or features which should be implemented. There is no specific format for creating issues, but please keep your issues succinct and specific to either a problem or feature. You can create an issue by clicking on the `Issues` tab at the top of the repository and selecting `New issue`. Tags should be added to the issue in order to indicate what the issue pertains to, i.e. drive train, autonomous routines, vision, etc.

### Creating Branches

Branches should be created only off of the `main` branch to address issues. These branches are not required to pass CI or work during development, but they should by the time a PR is made. Branch names should be prefixed with `feature/` or `bug/` depending on the nature of the issue the branch is addressing. Ensure your branch has been published to remote, called `origin` by default, in order to create PRs and ensure everyone can see your progress on an issue.

### Creating Commits

Like issues, there is no specific format to creating commits. However, commits should only be made to development branches outside of `main` and commit messages should briefly but accurately describe the changes made in that commit. Commits should be made frequently in case a problem is encountered and you want to find where exactly the problem originated.

### Creating Pull Requests

Pull requests should be made in GitHub once a branch has adequately solved an issue. To create a PR, simply go to the `Pull requests` tab on the repository in GitHub and select `New pull request`. The pull request should include how it solved an issue along with `Closes <issue-number>` or `Fixes <issue-number>` so GitHub knows to automatically close an issue once a PR has been accepted and pulled into the `main` branch. These PRs should contain code that has been tested on the robot and pass CI in order to keep `main` free of significant problems. Your PRs should be thoroughly reviewed by at least one other person on the programming department.

### Creating Releases

Releases should only be made prior to competitions and based off of the main branch. Releases should be competition-ready and thoroughly tested in order to prevent code changes at competition. These will be used at competition hopefully without alteration.