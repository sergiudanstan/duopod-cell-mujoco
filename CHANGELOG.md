# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-11-25

First tagged release. A MuJoCo physics simulation of a ceiling-mounted 2-axis parallel robot
(modelled on the Autonox DuoPod RVS2-1000-3kg) moving 3 kg cartons from an infeed conveyor to the
conveyor beside it: closed-chain kinematics, 1 kHz servo control with feed-forward torque, a vacuum
gripper, sensor-driven cell control, and a pytest suite of physics checks.

### Added
- CI runs the physics tests from `uv.lock` on every push and pull request, and checks that the
  collision test fails when the escapement interlock is removed (`--perturb`).
- CI, CodeQL, OpenSSF Scorecard and a tag-triggered release workflow with build provenance and
  a CycloneDX SBOM; Dependabot for actions and uv.
- `SECURITY.md`, `CONTRIBUTING.md` and this changelog.

[Unreleased]: https://github.com/sergiudanstan/duopod-cell-mujoco/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/sergiudanstan/duopod-cell-mujoco/releases/tag/v0.1.0
