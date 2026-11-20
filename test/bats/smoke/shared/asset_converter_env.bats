#!/usr/bin/env bats
#
# Repo-specific runtime smoke tests. Exercise the `devel` image built
# from this repo's Dockerfile, via the `devel-test` stage. Use the shared
# helpers in test_helper.bash (assert_cmd_installed, assert_file_exists,
# assert_dir_exists, assert_file_owned_by, assert_pip_pkg, ...) to keep
# assertions terse. Add one assertion per meaningful installation
# artifact. Assertions here run on EVERY -test stage (shared/), so keep
# them to the universal surface; put stage-specific checks under
# test/bats/smoke/<stage>/.

setup() {
  load "${BATS_TEST_DIRNAME}/test_helper"
}

@test "entrypoint.sh is installed and executable" {
  assert_file_exists /entrypoint.sh
  assert [ -x /entrypoint.sh ]
}

@test "bash is available on PATH" {
  assert_cmd_installed bash
}
