use core::future::Future;

use defmt::Format;
use embedded_hal_async::spi::*;
use lorawan_device::async_device::radio::{PhyRxTx, RfConfig, RxQuality, TxConfig};
use lorawan_device::async_device::Timings;

mod sx126x_lora;
use sx126x_lora::LoRa;

use self::sx126x_lora::mod_params::RadioError;

pub enum AntennaDirection {
    Rx,
    Tx,
}

pub trait Board {
    type OutputError;
    fn set_cs_high(&mut self) -> Result<(), Self::OutputError>;
    fn set_cs_low(&mut self) -> Result<(), Self::OutputError>;
    fn set_reset_high(&mut self) -> Result<(), Self::OutputError>;
    fn set_reset_low(&mut self) -> Result<(), Self::OutputError>;
    fn enable_antenna(&mut self, dir: AntennaDirection) -> Result<(), Self::OutputError>;
    fn disable_antenna(&mut self, dir: AntennaDirection) -> Result<(), Self::OutputError>;

    type WaitError;
    type WaitForDioFuture<'a>: Future<Output = Result<(), Self::WaitError>>
    where
        Self: 'a;

    fn wait_for_dio1(&mut self) -> Self::WaitForDioFuture<'_>;

    type WaitForBusyFuture<'a>: Future<Output = Result<(), Self::WaitError>>
    where
        Self: 'a;

    fn wait_for_busy(&mut self) -> Self::WaitForBusyFuture<'_>;
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
        1050
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
