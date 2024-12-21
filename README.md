# Driver for the [AHT2x](https://asairsensors.com/product/aht25-integrated-temperature-and-humidity-sensor/) temperature & humidity sensor

## Usage

Requirements:
* I2C interface implementing the [embedded-hal-async](https://crates.io/crates/embedded-hal-async) traits
* An timer implementing [DelayNs](https://docs.rs/embedded-hal-async/1.0.0/embedded_hal_async/delay/trait.DelayNs.html)

```rust
let i2c = unimplemented!();
let delay = unimplemented!();

let sensor = Aht2X::setup(&mut ic2, &mut delay).await?;

let (humidity, temperature) = sensor.measure(&mut ic2, &mut delay).await?.split(); 

println!("temperature: {}.{}C", temperature.celsius().0, temperature.celsius().1);
```
