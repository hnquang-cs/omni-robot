#!/usr/bin/env bash

set -euo pipefail

exec rosrun omni_base_controller test_discrete_45_primitives.py "$@"
