# `lsm303dlhc`

> A WIP, no_std, generic driver for the LSM303DLHC (accelerometer + magnetometer)

## What works

- Reading the accelerometer
- Reading the magnetometer and temperature sensor

## TODO

- [ ] Make sure this works with the `i2cdev` crate (i.e. with the Raspberry Pi)
- [ ] Configuration. e.g. selecting the accelerometer / magnetometer sensing range / sensitivity.
- [ ] How to make the API compatible with device specific features like DMA and interrupts?
- ???

## Examples

You should find at least one example in the [f3] repository. If that branch is gone, check
the master branch.

[f3]: https://github.com/japaric/f3/tree/singletons/examples

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the
work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.