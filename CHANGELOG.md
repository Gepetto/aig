# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.3.1] - 2025-10-23

- CMake: fix BUILD_STANDALONE_PYTHON_INTERFACE

## [1.3.0] - 2025-10-23

- ⚠️ Move the default branch on github to devel ⚠️
- Add nix CI
- Add mergify rules on devel
- Update pre-commit rules and format.


## [1.2.1] - 2023-12-03

- add CHANGELOG
- require CMake >= 3.10
- require example-robot-data >= 4.0.7
- update CMake
- update tooling (add isort, ruff, cmake-format, toml-sort, drop flake8)

## [1.2.0] - 2022-12-12

## [1.1.0] - 2022-09-12

- require C++ >= 11
- example-robot-data is optional
- fix the computation of the dynamcis using Pioncchio and a unit-test on a static posture.
- fix the check on the creation of settings using a robot name
- rename the private method `void configurateLegs()` into `void configureLegs()`
- add some getters on the CoM and its derivatives
- update the `urdf_path` into `urdf`, and same for `srdf`

## [1.0.0] - 2022-04-06

Initial release

[Unreleased]: https://github.com/gepetto/aig/compare/v1.3.1...HEAD
[1.3.1]: https://github.com/gepetto/aig/compare/v1.3.0...v1.3.1
[1.3.0]: https://github.com/gepetto/aig/compare/v1.2.1...v1.3.0
[1.2.1]: https://github.com/gepetto/aig/compare/v1.2.0...v1.2.1
[1.2.0]: https://github.com/gepetto/aig/compare/v1.1.0...v1.2.0
[1.1.0]: https://github.com/gepetto/aig/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/gepetto/aig/releases/tag/v1.0.0
