# Releasing tf2_rs

Releases are published to crates.io by the `Release` workflow when a version
tag is pushed.

1. Bump the version in `Cargo.toml`, `package.xml`, and the README dependency
   example, then update `Cargo.lock`. The workflow refuses to publish if the tag
   or resolved README example does not match the crate version.
2. Merge that change to `master` through a PR, so CI runs on it.
3. Tag the merge commit and push the tag:

   ```bash
   git switch master && git pull
   git tag v0.1.1
   git push origin v0.1.1
   ```

The workflow then checks the version and verifies that the tag is contained in
`master`, runs locked CI on every distribution, publishes to crates.io, creates
a GitHub release with generated notes and updates the API docs on GitHub Pages.

A published version cannot be overwritten. If a release is broken, fix it, bump
to the next patch version and yank the broken one with
`cargo yank --version <version>`.

## Keeping up with rclrs

- Dependabot opens a PR when `rclrs` or `ros-env` release a version outside the
  range in `Cargo.toml`. Both are updated in the same PR, since their message
  types must match. All other compatible dependency updates arrive together in
  one weekly PR; major updates get their own PR.
- The `rclrs main` workflow builds against the `main` branch of ros2_rust every
  Monday. A failure there is an early warning that the next rclrs release will
  need changes in tf2_rs.
- Required CI and releases use the committed lockfile. The weekly `CI` run also
  updates it in a temporary checkout and tests new compatible rclrs/ros-env
  patch releases on every distribution.

GitHub disables scheduled workflows after 60 days without activity in the
repository; re-enable them from the Actions tab if that happens.
