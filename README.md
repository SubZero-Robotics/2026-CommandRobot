# 2026 Season Robot Code

A repository containing robot code for 5690's 2026 season.

## Table of Contents

- [2026 Season Robot Code](#2026-season-robot-code)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
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

## Can IDs

All devices connected to the CAN bus along with their corresponding CAN IDs

| Device                  | CAN ID |
| ----------------------- | ------ |
| Front Right Drive Motor | 1      |
| Front Left Drive Motor  | 2      |
| Rear Left Drive Motor   | 3      |
| Rear Right Drive Motor  | 4      |
| Front Right Turn Motor  | 5      |
| Front Left Turn Motor   | 6      |
| Rear Left Turn Motor    | 7      |
| Rear Right Turn Motor   | 8      |
| Pigeon Gyro             | 13     |

## Network Map

All devices connected to the robot's local network along with each device's assigned IP address

| Device  | IP         |
| ------- | ---------- |
| Gateway | 10.56.90.1 |
| RoboRio | 10.56.90.2 |

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