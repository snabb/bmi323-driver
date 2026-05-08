# Releasing `bmi323-driver`

This document describes the release process for the `bmi323-driver` crate.

## Before releasing

- Make sure the crate version in `Cargo.toml` is the intended next version.
- Review public API changes and confirm the version bump follows SemVer.
- Make sure `README.md`, examples, and crate metadata still match the current API.
- Make sure you are logged in to crates.io with `cargo login` if needed.

## Required checks

Run the same checks that CI uses before publishing:

```sh
cargo fmt --check
cargo clippy --all-targets --features async,defmt -- -D warnings
cargo clippy --all-targets --features blocking,defmt -- -D warnings
cargo test --features async
cargo test --features blocking
cargo check --examples --features async
cargo check --examples --features blocking
cargo publish --dry-run
```

Do not use `cargo test --all-features` for this crate. The `async` and
`blocking` features are intentionally mutually exclusive, so enabling both at
the same time fails by design.

## Publish

After the checks pass:

```sh
cargo publish
```

## After publishing

- Wait for the new version to appear on crates.io.
- Wait for docs.rs to build the new documentation version.
- Create and push a matching git tag:

```sh
git tag vX.Y.Z
git push origin vX.Y.Z
```

- Create a GitHub release if you want release notes attached to the tag.

## Notes

- The crate uses Rust edition 2024 and currently declares `rust-version = "1.85"`.
- CI also checks the STM32 example project under
  `embassy-stm32g030f6-examples/`. That project is not published as part of the
  crate, but keeping its checks green helps catch integration regressions before
  release.
