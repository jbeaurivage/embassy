use core::future::Future;

#[cfg(feature = "defmt")]
use defmt::Format;

#[cfg(not(feature = "defmt"))]
/// Dummy trait to make builds work when `defmt` feature is off. A blanket impl
/// is provided for all types.
pub trait Format {}
#[cfg(not(feature = "defmt"))]
impl<T> Format for T {}

#[cfg(all(not(feature = "time"), not(feature = "fugit")))]
compile_error!("Must specify at least one of `time` or `fugit` Cargo features.");

#[cfg(feature = "time")]
type Duration = embassy_time::Duration;

#[cfg(not(feature = "time"))]
type Duration = fugit::MillisDurationU64;

#[cfg(feature = "time")]
fn get_duration(ticks: u64) -> Duration {
    embassy_time::Duration::from_millis(ticks)
}

#[cfg(not(feature = "time"))]
fn get_duration(ticks: u64) -> Duration {
    fugit::MillisDurationU64::from_ticks(ticks)
}

use embedded_hal_async::spi::*;
use lorawan_device::async_device::radio::{PhyRxTx, RfConfig, RxQuality, TxConfig};
use lorawan_device::async_device::Timings;

mod sx126x_lora;
use sx126x_lora::LoRa;

use self::sx126x_lora::mod_params::RadioError;

pub enum OscillatorMode {
    /// Temperature-compensated crystal oscillator
    Tcxo,
    /// Crystal oscillator
    Xosc,
}

impl OscillatorMode {
    fn dio3_as_tcxo_ctrl(&self) -> bool {
        match self {
            OscillatorMode::Tcxo => true,
            OscillatorMode::Xosc => false,
        }
    }
}

pub trait Board {
    /// Whether or not the SX126x radio is capable of automatically controlling
    /// the antenna switch via the DIO2 pin.
    const DIO2_ANTENNA_CONTROL: bool;

    /// The type of oscillator driving the SX126x chip. If a
    /// [`Tcxo`](OscillatorMode::Tcxo) is used, the DIO3 pin must be used to
    /// drive the TXCO.
    const OSC_MODE: OscillatorMode;

    /// An error which could happen when trying to change the state of a digital
    /// output pin. Some HALs will set this type to `()` or
    /// [`Infallible`](core::convert::Infallible).
    type DigitalOutputError;

    /// Set SPI CS pin high
    fn set_cs_high(&mut self) -> Result<(), Self::DigitalOutputError>;

    /// Set SPI CS pin low
    fn set_cs_low(&mut self) -> Result<(), Self::DigitalOutputError>;

    /// Set RESET pin high
    fn set_reset_high(&mut self) -> Result<(), Self::DigitalOutputError>;

    /// Set RESET pin low
    fn set_reset_low(&mut self) -> Result<(), Self::DigitalOutputError>;

    /// Setup antenna in TX mode. If DIO2 is used to control the antenna switch,
    /// this method could be left as a no-op.
    fn antenna_tx(&mut self) -> Result<(), Self::DigitalOutputError>;

    /// Setup antenna in RX mode. If DIO2 is used to control the antenna switch,
    /// this method could be left as a no-op.
    fn antenna_rx(&mut self) -> Result<(), Self::DigitalOutputError>;

    /// Setup antenna in sleep mode. In some board configurations, the antenna
    /// can only be in TX or RX mode (and no sleep mode). In those cases, this
    /// method should be a no-op.
    fn antenna_sleep(&mut self) -> Result<(), Self::DigitalOutputError>;

    /// An error which could occur when trying to wait for an interrupt to
    /// happen on DIO1.
    type WaitForDio1Error;
    async fn wait_for_dio1_high(&mut self) -> Result<(), Self::WaitForDio1Error>;

    /// An error which could occur when trying to wait for an interrupt to
    /// happen on BUSY.
    type WaitForBusyError;
    async fn wait_for_busy_low(&mut self) -> Result<(), Self::WaitForBusyError>;

    /// With the `time` feature disabled, you must provide your own
    /// implementation of `wait`.
    #[cfg(not(feature = "time"))]
    async fn wait(&mut self, duration: Duration);
}

/// Semtech Sx126x LoRa peripheral
pub struct Sx126xRadio<B, SPI, BUS>
where
    B: Board,
    SPI: SpiBus<u8, Error = BUS> + 'static,
    BUS: Error + Format + 'static,
{
    pub lora: LoRa<B, SPI>,
}

impl<B, SPI, BUS> Sx126xRadio<B, SPI, BUS>
where
    B: Board,
    SPI: SpiBus<u8, Error = BUS> + 'static,
    BUS: Error + Format + 'static,
{
    pub async fn new(spi: SPI, board: B, enable_public_network: bool) -> Result<Self, RadioError<BUS>> {
        let mut lora = LoRa::new(spi, board);
        lora.init().await?;
        lora.set_lora_modem(enable_public_network).await?;
        Ok(Self { lora })
    }
}

impl<B, SPI, BUS> Timings for Sx126xRadio<B, SPI, BUS>
where
    B: Board,
    SPI: SpiBus<u8, Error = BUS> + 'static,
    BUS: Error + Format + 'static,
{
    fn get_rx_window_offset_ms(&self) -> i32 {
        -50
    }
    fn get_rx_window_duration_ms(&self) -> u32 {
        1000
    }
}

impl<B, SPI, BUS> PhyRxTx for Sx126xRadio<B, SPI, BUS>
where
    B: Board,
    SPI: SpiBus<u8, Error = BUS> + 'static,
    BUS: Error + Format + 'static,
{
    type PhyError = RadioError<BUS>;

    type TxFuture<'m> = impl Future<Output = Result<u32, Self::PhyError>> + 'm
    where
        B: 'm,
        SPI: 'm,
        BUS: 'm;

    fn tx<'m>(&'m mut self, config: TxConfig, buffer: &'m [u8]) -> Self::TxFuture<'m> {
        trace!("TX START");
        async move {
            self.lora
                .set_tx_config(
                    config.pw,
                    config.rf.spreading_factor.into(),
                    config.rf.bandwidth.into(),
                    config.rf.coding_rate.into(),
                    8,
                    false,
                    true,
                    false,
                    0,
                    false,
                )
                .await?;
            self.lora.set_max_payload_length(buffer.len() as u8).await?;
            self.lora.set_channel(config.rf.frequency).await?;
            self.lora.send(buffer, 0xffffff).await?;
            self.lora.process_irq(None, None, None).await?;
            trace!("TX DONE");
            Ok(0)
        }
    }

    type RxFuture<'m> = impl Future<Output = Result<(usize, RxQuality), Self::PhyError>> + 'm
    where
        B: 'm,
        SPI: 'm,
        BUS: 'm;

    fn rx<'m>(&'m mut self, config: RfConfig, receiving_buffer: &'m mut [u8]) -> Self::RxFuture<'m> {
        trace!("RX START");
        async move {
            self.lora
                .set_rx_config(
                    config.spreading_factor.into(),
                    config.bandwidth.into(),
                    config.coding_rate.into(),
                    8,
                    4,
                    false,
                    0u8,
                    true,
                    false,
                    0,
                    true,
                    true,
                )
                .await?;
            self.lora.set_max_payload_length(receiving_buffer.len() as u8).await?;
            self.lora.set_channel(config.frequency).await?;
            self.lora.rx(90 * 1000).await?;
            let mut received_len = 0u8;
            self.lora
                .process_irq(Some(receiving_buffer), Some(&mut received_len), None)
                .await?;
            trace!("RX DONE");

            let packet_status = self.lora.get_latest_packet_status();
            let mut rssi = 0i16;
            let mut snr = 0i8;
            if let Some(s) = packet_status {
                rssi = s.rssi as i16;
                snr = s.snr;
            }

            Ok((received_len as usize, RxQuality::new(rssi, snr)))
        }
    }
}

#[cfg(feature = "rng")]
impl<B, SPI, BUS> rand_core::RngCore for Sx126xRadio<B, SPI, BUS>
where
    B: Board,
    SPI: SpiBus<u8, Error = BUS> + 'static,
    BUS: Error + Format + 'static,
{
    fn next_u32(&mut self) -> u32 {
        spin_on::spin_on(self.lora.get_random_value()).unwrap()
    }

    fn next_u64(&mut self) -> u64 {
        rand_core::impls::next_u64_via_u32(self)
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        rand_core::impls::fill_bytes_via_next(self, dest)
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        rand_core::impls::fill_bytes_via_next(self, dest);
        Ok(())
    }
}
