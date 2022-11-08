use embedded_hal_async::spi::SpiBus;

use super::mod_params::*;
use super::LoRa;
use crate::sx126x::Board;

// Internal frequency of the radio
const SX126X_XTAL_FREQ: u32 = 32000000;

// Scaling factor used to perform fixed-point operations
const SX126X_PLL_STEP_SHIFT_AMOUNT: u32 = 14;

// PLL step - scaled with SX126X_PLL_STEP_SHIFT_AMOUNT
const SX126X_PLL_STEP_SCALED: u32 = SX126X_XTAL_FREQ >> (25 - SX126X_PLL_STEP_SHIFT_AMOUNT);

// Maximum value for parameter symbNum
const SX126X_MAX_LORA_SYMB_NUM_TIMEOUT: u8 = 248;

// Provides board-specific functionality for Semtech SX126x-based boards

impl<B, SPI, BUS> LoRa<B, SPI>
where
    B: Board,
    SPI: SpiBus<u8, Error = BUS>,
{
    // Initialize the radio driver
    pub(super) async fn sub_init(&mut self) -> Result<(), RadioError<BUS>> {
        self.brd_reset().await?;
        self.brd_wakeup().await?;
        self.sub_set_standby(StandbyMode::RC).await?;
        self.brd_io_tcxo_init().await?;
        self.brd_io_rf_switch_init().await?;
        self.image_calibrated = false;
        Ok(())
    }

    // Wakeup the radio if it is in Sleep mode and check that Busy is low
    pub(super) async fn sub_check_device_ready(&mut self) -> Result<(), RadioError<BUS>> {
        let operating_mode = self.brd_get_operating_mode();
        if operating_mode == RadioMode::Sleep || operating_mode == RadioMode::ReceiveDutyCycle {
            self.brd_wakeup().await?;
        }
        self.brd_wait_on_busy().await?;
        Ok(())
    }

    // Save the payload to be sent in the radio buffer
    pub(super) async fn sub_set_payload(&mut self, payload: &[u8]) -> Result<(), RadioError<BUS>> {
        self.brd_write_buffer(0x00, payload).await?;
        Ok(())
    }

    // Read the payload received.
    pub(super) async fn sub_get_payload(&mut self, buffer: &mut [u8]) -> Result<u8, RadioError<BUS>> {
        let (size, offset) = self.sub_get_rx_buffer_status().await?;
        if (size as usize) > buffer.len() {
            Err(RadioError::PayloadSizeMismatch(size as usize, buffer.len()))
        } else {
            self.brd_read_buffer(offset, buffer).await?;
            Ok(size)
        }
    }

    // Send a payload
    pub(super) async fn sub_send_payload(&mut self, payload: &[u8], timeout: u32) -> Result<(), RadioError<BUS>> {
        self.sub_set_payload(payload).await?;
        self.sub_set_tx(timeout).await?;
        Ok(())
    }

    // Get a 32-bit random value generated by the radio.  A valid packet type must have been configured before using this command.
    //
    // The radio must be in reception mode before executing this function.  This code can potentially result in interrupt generation. It is the responsibility of
    // the calling code to disable radio interrupts before calling this function, and re-enable them afterwards if necessary, or be certain that any interrupts
    // generated during this process will not cause undesired side-effects in the software.
    //
    // The random numbers produced by the generator do not have a uniform or Gaussian distribution. If uniformity is needed, perform appropriate software post-processing.
    pub(super) async fn sub_get_random(&mut self) -> Result<u32, RadioError<BUS>> {
        let mut reg_ana_lna_buffer_original = [0x00u8];
        let mut reg_ana_mixer_buffer_original = [0x00u8];
        let mut reg_ana_lna_buffer = [0x00u8];
        let mut reg_ana_mixer_buffer = [0x00u8];
        let mut number_buffer = [0x00u8, 0x00u8, 0x00u8, 0x00u8];

        self.brd_read_registers(Register::AnaLNA, &mut reg_ana_lna_buffer_original)
            .await?;
        reg_ana_lna_buffer[0] = reg_ana_lna_buffer_original[0] & (!(1 << 0));
        self.brd_write_registers(Register::AnaLNA, &reg_ana_lna_buffer).await?;

        self.brd_read_registers(Register::AnaMixer, &mut reg_ana_mixer_buffer_original)
            .await?;
        reg_ana_mixer_buffer[0] = reg_ana_mixer_buffer_original[0] & (!(1 << 7));
        self.brd_write_registers(Register::AnaMixer, &reg_ana_mixer_buffer)
            .await?;

        // Set radio in continuous reception
        self.sub_set_rx(0xFFFFFFu32).await?;

        self.brd_read_registers(Register::GeneratedRandomNumber, &mut number_buffer)
            .await?;

        self.sub_set_standby(StandbyMode::RC).await?;

        self.brd_write_registers(Register::AnaLNA, &reg_ana_lna_buffer_original)
            .await?;
        self.brd_write_registers(Register::AnaMixer, &reg_ana_mixer_buffer_original)
            .await?;

        Ok(Self::convert_u8_buffer_to_u32(&number_buffer))
    }

    // Set the radio in sleep mode
    pub(super) async fn sub_set_sleep(&mut self, sleep_config: SleepParams) -> Result<(), RadioError<BUS>> {
        self.brd_ant_sleep()?;

        if !sleep_config.warm_start {
            self.image_calibrated = false;
        }

        self.brd_write_command(OpCode::SetSleep, &[sleep_config.value()])
            .await?;
        self.brd_set_operating_mode(RadioMode::Sleep);
        Ok(())
    }

    // Set the radio in configuration mode
    pub(super) async fn sub_set_standby(&mut self, mode: StandbyMode) -> Result<(), RadioError<BUS>> {
        self.brd_write_command(OpCode::SetStandby, &[mode.value()]).await?;
        if mode == StandbyMode::RC {
            self.brd_set_operating_mode(RadioMode::StandbyRC);
        } else {
            self.brd_set_operating_mode(RadioMode::StandbyXOSC);
        }

        self.brd_ant_sleep()?;
        Ok(())
    }

    // Set the radio in FS mode
    pub(super) async fn sub_set_fs(&mut self) -> Result<(), RadioError<BUS>> {
        // antenna settings ???
        self.brd_write_command(OpCode::SetFS, &[]).await?;
        self.brd_set_operating_mode(RadioMode::FrequencySynthesis);
        Ok(())
    }

    // Set the radio in transmission mode with timeout specified
    pub(super) async fn sub_set_tx(&mut self, timeout: u32) -> Result<(), RadioError<BUS>> {
        let buffer = [
            Self::timeout_1(timeout),
            Self::timeout_2(timeout),
            Self::timeout_3(timeout),
        ];

        self.brd_ant_set_tx()?;

        self.brd_set_operating_mode(RadioMode::Transmit);
        self.brd_write_command(OpCode::SetTx, &buffer).await?;
        Ok(())
    }

    // Set the radio in reception mode with timeout specified
    pub(super) async fn sub_set_rx(&mut self, timeout: u32) -> Result<(), RadioError<BUS>> {
        let buffer = [
            Self::timeout_1(timeout),
            Self::timeout_2(timeout),
            Self::timeout_3(timeout),
        ];

        self.brd_ant_set_rx()?;

        self.brd_set_operating_mode(RadioMode::Receive);
        self.brd_write_registers(Register::RxGain, &[0x94u8]).await?;
        self.brd_write_command(OpCode::SetRx, &buffer).await?;
        Ok(())
    }

    // Set the radio in reception mode with Boosted LNA gain and timeout specified
    pub(super) async fn sub_set_rx_boosted(&mut self, timeout: u32) -> Result<(), RadioError<BUS>> {
        let buffer = [
            Self::timeout_1(timeout),
            Self::timeout_2(timeout),
            Self::timeout_3(timeout),
        ];

        self.brd_ant_set_rx()?;

        self.brd_set_operating_mode(RadioMode::Receive);
        // set max LNA gain, increase current by ~2mA for around ~3dB in sensitivity
        self.brd_write_registers(Register::RxGain, &[0x96u8]).await?;
        self.brd_write_command(OpCode::SetRx, &buffer).await?;
        Ok(())
    }

    // Set the Rx duty cycle management parameters
    pub(super) async fn sub_set_rx_duty_cycle(&mut self, rx_time: u32, sleep_time: u32) -> Result<(), RadioError<BUS>> {
        let buffer = [
            ((rx_time >> 16) & 0xFF) as u8,
            ((rx_time >> 8) & 0xFF) as u8,
            (rx_time & 0xFF) as u8,
            ((sleep_time >> 16) & 0xFF) as u8,
            ((sleep_time >> 8) & 0xFF) as u8,
            (sleep_time & 0xFF) as u8,
        ];

        // antenna settings ???

        self.brd_write_command(OpCode::SetRxDutyCycle, &buffer).await?;
        self.brd_set_operating_mode(RadioMode::ReceiveDutyCycle);
        Ok(())
    }

    // Set the radio in CAD mode
    pub(super) async fn sub_set_cad(&mut self) -> Result<(), RadioError<BUS>> {
        self.brd_ant_set_rx()?;

        self.brd_write_command(OpCode::SetCAD, &[]).await?;
        self.brd_set_operating_mode(RadioMode::ChannelActivityDetection);
        Ok(())
    }

    // Set the radio in continuous wave transmission mode
    pub(super) async fn sub_set_tx_continuous_wave(&mut self) -> Result<(), RadioError<BUS>> {
        self.brd_ant_set_tx()?;

        self.brd_write_command(OpCode::SetTxContinuousWave, &[]).await?;
        self.brd_set_operating_mode(RadioMode::Transmit);
        Ok(())
    }

    // Set the radio in continuous preamble transmission mode
    pub(super) async fn sub_set_tx_infinite_preamble(&mut self) -> Result<(), RadioError<BUS>> {
        self.brd_ant_set_tx()?;

        self.brd_write_command(OpCode::SetTxContinuousPremable, &[]).await?;
        self.brd_set_operating_mode(RadioMode::Transmit);
        Ok(())
    }

    // Decide which interrupt will stop the internal radio rx timer.
    //   false     timer stop after header/syncword detection
    //   true      timer stop after preamble detection
    pub(super) async fn sub_set_stop_rx_timer_on_preamble_detect(
        &mut self,
        enable: bool,
    ) -> Result<(), RadioError<BUS>> {
        self.brd_write_command(OpCode::SetStopRxTimerOnPreamble, &[enable as u8])
            .await?;
        Ok(())
    }

    // Set the number of symbols the radio will wait to validate a reception
    pub(super) async fn sub_set_lora_symb_num_timeout(&mut self, symb_num: u16) -> Result<(), RadioError<BUS>> {
        let mut exp = 0u8;
        let mut reg;
        let mut mant = ((core::cmp::min(symb_num, SX126X_MAX_LORA_SYMB_NUM_TIMEOUT as u16) as u8) + 1) >> 1;
        while mant > 31 {
            mant = (mant + 3) >> 2;
            exp += 1;
        }
        reg = mant << ((2 * exp) + 1);

        self.brd_write_command(OpCode::SetLoRaSymbTimeout, &[reg]).await?;

        if symb_num != 0 {
            reg = exp + (mant << 3);
            self.brd_write_registers(Register::SynchTimeout, &[reg]).await?;
        }

        Ok(())
    }

    // Set the power regulators operating mode (LDO or DC_DC).  Using only LDO implies that the Rx or Tx current is doubled
    pub(super) async fn sub_set_regulator_mode(&mut self, mode: RegulatorMode) -> Result<(), RadioError<BUS>> {
        self.brd_write_command(OpCode::SetRegulatorMode, &[mode.value()])
            .await?;
        Ok(())
    }

    // Calibrate the given radio block
    pub(super) async fn sub_calibrate(&mut self, calibrate_params: CalibrationParams) -> Result<(), RadioError<BUS>> {
        self.brd_write_command(OpCode::Calibrate, &[calibrate_params.value()])
            .await?;
        Ok(())
    }

    // Calibrate the image rejection based on the given frequency
    pub(super) async fn sub_calibrate_image(&mut self, freq: u32) -> Result<(), RadioError<BUS>> {
        let mut cal_freq = [0x00u8, 0x00u8];

        if freq > 900000000 {
            cal_freq[0] = 0xE1;
            cal_freq[1] = 0xE9;
        } else if freq > 850000000 {
            cal_freq[0] = 0xD7;
            cal_freq[1] = 0xDB;
        } else if freq > 770000000 {
            cal_freq[0] = 0xC1;
            cal_freq[1] = 0xC5;
        } else if freq > 460000000 {
            cal_freq[0] = 0x75;
            cal_freq[1] = 0x81;
        } else if freq > 425000000 {
            cal_freq[0] = 0x6B;
            cal_freq[1] = 0x6F;
        }
        self.brd_write_command(OpCode::CalibrateImage, &cal_freq).await?;
        Ok(())
    }

    // Activate the extention of the timeout when a long preamble is used
    pub(super) async fn sub_set_long_preamble(&mut self, _enable: u8) -> Result<(), RadioError<BUS>> {
        Ok(()) // no operation currently
    }

    // Set the transmission parameters
    //   hp_max          0 for sx1261, 7 for sx1262
    //   device_sel      1 for sx1261, 0 for sx1262
    //   pa_lut          0 for 14dBm LUT, 1 for 22dBm LUT
    pub(super) async fn sub_set_pa_config(
        &mut self,
        pa_duty_cycle: u8,
        hp_max: u8,
        device_sel: u8,
        pa_lut: u8,
    ) -> Result<(), RadioError<BUS>> {
        self.brd_write_command(OpCode::SetPAConfig, &[pa_duty_cycle, hp_max, device_sel, pa_lut])
            .await?;
        Ok(())
    }

    // Define into which mode the chip goes after a TX / RX done
    pub(super) async fn sub_set_rx_tx_fallback_mode(&mut self, fallback_mode: u8) -> Result<(), RadioError<BUS>> {
        self.brd_write_command(OpCode::SetTxFallbackMode, &[fallback_mode])
            .await?;
        Ok(())
    }

    // Set the IRQ mask and DIO masks
    pub(super) async fn sub_set_dio_irq_params(
        &mut self,
        irq_mask: u16,
        dio1_mask: u16,
        dio2_mask: u16,
        dio3_mask: u16,
    ) -> Result<(), RadioError<BUS>> {
        let mut buffer = [0x00u8; 8];

        buffer[0] = ((irq_mask >> 8) & 0x00FF) as u8;
        buffer[1] = (irq_mask & 0x00FF) as u8;
        buffer[2] = ((dio1_mask >> 8) & 0x00FF) as u8;
        buffer[3] = (dio1_mask & 0x00FF) as u8;
        buffer[4] = ((dio2_mask >> 8) & 0x00FF) as u8;
        buffer[5] = (dio2_mask & 0x00FF) as u8;
        buffer[6] = ((dio3_mask >> 8) & 0x00FF) as u8;
        buffer[7] = (dio3_mask & 0x00FF) as u8;
        self.brd_write_command(OpCode::CfgDIOIrq, &buffer).await?;
        Ok(())
    }

    // Return the current IRQ status
    pub(super) async fn sub_get_irq_status(&mut self) -> Result<u16, RadioError<BUS>> {
        let mut irq_status = [0x00u8, 0x00u8];
        self.brd_read_command(OpCode::GetIrqStatus, &mut irq_status).await?;
        Ok(((irq_status[0] as u16) << 8) | (irq_status[1] as u16))
    }

    // Indicate if DIO2 is used to control an RF Switch
    pub(super) async fn sub_set_dio2_as_rf_switch_ctrl(&mut self, enable: bool) -> Result<(), RadioError<BUS>> {
        self.brd_write_command(OpCode::SetRFSwitchMode, &[enable as u8]).await?;
        Ok(())
    }

    // Indicate if the radio main clock is supplied from a TCXO
    //   tcxo_voltage        voltage used to control the TCXO on/off from DIO3
    //   timeout             duration given to the TCXO to go to 32MHz
    pub(super) async fn sub_set_dio3_as_tcxo_ctrl(
        &mut self,
        tcxo_voltage: TcxoCtrlVoltage,
        timeout: u32,
    ) -> Result<(), RadioError<BUS>> {
        let buffer = [
            tcxo_voltage.value() & 0x07,
            Self::timeout_1(timeout),
            Self::timeout_2(timeout),
            Self::timeout_3(timeout),
        ];

        self.brd_write_command(OpCode::SetTCXOMode, &buffer).await?;

        Ok(())
    }

    // Set the RF frequency (Hz)
    pub(super) async fn sub_set_rf_frequency(&mut self, frequency: u32) -> Result<(), RadioError<BUS>> {
        let mut buffer = [0x00u8; 4];

        if !self.image_calibrated {
            self.sub_calibrate_image(frequency).await?;
            self.image_calibrated = true;
        }

        let freq_in_pll_steps = Self::convert_freq_in_hz_to_pll_step(frequency);

        buffer[0] = ((freq_in_pll_steps >> 24) & 0xFF) as u8;
        buffer[1] = ((freq_in_pll_steps >> 16) & 0xFF) as u8;
        buffer[2] = ((freq_in_pll_steps >> 8) & 0xFF) as u8;
        buffer[3] = (freq_in_pll_steps & 0xFF) as u8;
        self.brd_write_command(OpCode::SetRFFrequency, &buffer).await?;
        Ok(())
    }

    // Set the radio for the given protocol (LoRa or GFSK).  This method has to be called before setting RF frequency, modulation paramaters, and packet paramaters.
    pub(super) async fn sub_set_packet_type(&mut self, packet_type: PacketType) -> Result<(), RadioError<BUS>> {
        self.packet_type = packet_type;
        self.brd_write_command(OpCode::SetPacketType, &[packet_type.value()])
            .await?;
        Ok(())
    }

    // Get the current radio protocol (LoRa or GFSK)
    pub(super) fn sub_get_packet_type(&mut self) -> PacketType {
        self.packet_type
    }

    // Set the transmission parameters
    //   power           RF output power [-18..13] dBm
    //   ramp_time       transmission ramp up time
    pub(super) async fn sub_set_tx_params(
        &mut self,
        mut power: i8,
        ramp_time: RampTime,
    ) -> Result<(), RadioError<BUS>> {
        if self.brd_get_radio_type() == RadioType::SX1261 {
            if power == 15 {
                self.sub_set_pa_config(0x06, 0x00, 0x01, 0x01).await?;
            } else {
                self.sub_set_pa_config(0x04, 0x00, 0x01, 0x01).await?;
            }

            power = power.clamp(-17, 14);
        } else {
            // Provide better resistance of the SX1262 Tx to antenna mismatch (see DS_SX1261-2_V1.2 datasheet chapter 15.2)
            let mut tx_clamp_cfg = [0x00u8];
            self.brd_read_registers(Register::TxClampCfg, &mut tx_clamp_cfg).await?;
            tx_clamp_cfg[0] |= 0x0F << 1;
            self.brd_write_registers(Register::TxClampCfg, &tx_clamp_cfg).await?;

            self.sub_set_pa_config(0x04, 0x07, 0x00, 0x01).await?;

            power = power.clamp(-9, 22);
        }

        // power conversion of negative number from i8 to u8 ???
        self.brd_write_command(OpCode::SetTxParams, &[power as u8, ramp_time.value()])
            .await?;
        Ok(())
    }

    // Set the modulation parameters
    pub(super) async fn sub_set_modulation_params(&mut self) -> Result<(), RadioError<BUS>> {
        if self.modulation_params.is_some() {
            let mut buffer = [0x00u8; 4];

            // Since this driver only supports LoRa, ensure the packet type is set accordingly
            self.sub_set_packet_type(PacketType::LoRa).await?;

            let modulation_params = self.modulation_params.unwrap();
            buffer[0] = modulation_params.spreading_factor.value();
            buffer[1] = modulation_params.bandwidth.value();
            buffer[2] = modulation_params.coding_rate.value();
            buffer[3] = modulation_params.low_data_rate_optimize;

            self.brd_write_command(OpCode::SetModulationParams, &buffer).await?;
            Ok(())
        } else {
            Err(RadioError::ModulationParamsMissing)
        }
    }

    // Set the packet parameters
    pub(super) async fn sub_set_packet_params(&mut self) -> Result<(), RadioError<BUS>> {
        if self.packet_params.is_some() {
            let mut buffer = [0x00u8; 6];

            // Since this driver only supports LoRa, ensure the packet type is set accordingly
            self.sub_set_packet_type(PacketType::LoRa).await?;

            let packet_params = self.packet_params.unwrap();
            buffer[0] = ((packet_params.preamble_length >> 8) & 0xFF) as u8;
            buffer[1] = (packet_params.preamble_length & 0xFF) as u8;
            buffer[2] = packet_params.implicit_header as u8;
            buffer[3] = packet_params.payload_length;
            buffer[4] = packet_params.crc_on as u8;
            buffer[5] = packet_params.iq_inverted as u8;

            self.brd_write_command(OpCode::SetPacketParams, &buffer).await?;
            Ok(())
        } else {
            Err(RadioError::PacketParamsMissing)
        }
    }

    // Set the channel activity detection (CAD) parameters
    //   symbols            number of symbols to use for CAD operations
    //   det_peak           limit for detection of SNR peak used in the CAD
    //   det_min            minimum symbol recognition for CAD
    //   exit_mode          operation to be done at the end of CAD action
    //   timeout            timeout value to abort the CAD activity

    pub(super) async fn sub_set_cad_params(
        &mut self,
        symbols: CADSymbols,
        det_peak: u8,
        det_min: u8,
        exit_mode: CADExitMode,
        timeout: u32,
    ) -> Result<(), RadioError<BUS>> {
        let mut buffer = [0x00u8; 7];

        buffer[0] = symbols.value();
        buffer[1] = det_peak;
        buffer[2] = det_min;
        buffer[3] = exit_mode.value();
        buffer[4] = Self::timeout_1(timeout);
        buffer[5] = Self::timeout_2(timeout);
        buffer[6] = Self::timeout_3(timeout);

        self.brd_write_command(OpCode::SetCADParams, &buffer).await?;
        self.brd_set_operating_mode(RadioMode::ChannelActivityDetection);
        Ok(())
    }

    // Set the data buffer base address for transmission and reception
    pub(super) async fn sub_set_buffer_base_address(
        &mut self,
        tx_base_address: u8,
        rx_base_address: u8,
    ) -> Result<(), RadioError<BUS>> {
        self.brd_write_command(OpCode::SetBufferBaseAddress, &[tx_base_address, rx_base_address])
            .await?;
        Ok(())
    }

    // Get the current radio status
    pub(super) async fn sub_get_status(&mut self) -> Result<RadioStatus, RadioError<BUS>> {
        let status = self.brd_read_command(OpCode::GetStatus, &mut []).await?;
        Ok(RadioStatus {
            cmd_status: (status & (0x07 << 1)) >> 1,
            chip_mode: (status & (0x07 << 4)) >> 4,
        })
    }

    // Get the instantaneous RSSI value for the last packet received
    pub(super) async fn sub_get_rssi_inst(&mut self) -> Result<i8, RadioError<BUS>> {
        let mut buffer = [0x00u8];
        self.brd_read_command(OpCode::GetRSSIInst, &mut buffer).await?;
        let rssi: i8 = ((-(buffer[0] as i32)) >> 1) as i8; // check this ???
        Ok(rssi)
    }

    // Get the last received packet buffer status
    pub(super) async fn sub_get_rx_buffer_status(&mut self) -> Result<(u8, u8), RadioError<BUS>> {
        if self.packet_params.is_some() {
            let mut status = [0x00u8; 2];
            let mut payload_length_buffer = [0x00u8];

            self.brd_read_command(OpCode::GetRxBufferStatus, &mut status).await?;
            if (self.sub_get_packet_type() == PacketType::LoRa) && self.packet_params.unwrap().implicit_header {
                self.brd_read_registers(Register::PayloadLength, &mut payload_length_buffer)
                    .await?;
            } else {
                payload_length_buffer[0] = status[0];
            }

            let payload_length = payload_length_buffer[0];
            let offset = status[1];

            Ok((payload_length, offset))
        } else {
            Err(RadioError::PacketParamsMissing)
        }
    }

    // Get the last received packet payload status
    pub(super) async fn sub_get_packet_status(&mut self) -> Result<PacketStatus, RadioError<BUS>> {
        let mut status = [0x00u8; 3];
        self.brd_read_command(OpCode::GetPacketStatus, &mut status).await?;

        // check this ???
        let rssi = ((-(status[0] as i32)) >> 1) as i8;
        let snr = ((status[1] as i8) + 2) >> 2;
        let signal_rssi = ((-(status[2] as i32)) >> 1) as i8;
        let freq_error = self.frequency_error;

        Ok(PacketStatus {
            rssi,
            snr,
            signal_rssi,
            freq_error,
        })
    }

    // Get the possible system errors
    pub(super) async fn sub_get_device_errors(&mut self) -> Result<RadioSystemError, RadioError<BUS>> {
        let mut errors = [0x00u8; 2];
        self.brd_read_command(OpCode::GetErrors, &mut errors).await?;

        Ok(RadioSystemError {
            rc_64khz_calibration: (errors[1] & (1 << 0)) != 0,
            rc_13mhz_calibration: (errors[1] & (1 << 1)) != 0,
            pll_calibration: (errors[1] & (1 << 2)) != 0,
            adc_calibration: (errors[1] & (1 << 3)) != 0,
            image_calibration: (errors[1] & (1 << 4)) != 0,
            xosc_start: (errors[1] & (1 << 5)) != 0,
            pll_lock: (errors[1] & (1 << 6)) != 0,
            pa_ramp: (errors[0] & (1 << 0)) != 0,
        })
    }

    // Clear all the errors in the device
    pub(super) async fn sub_clear_device_errors(&mut self) -> Result<(), RadioError<BUS>> {
        self.brd_write_command(OpCode::ClrErrors, &[0x00u8, 0x00u8]).await?;
        Ok(())
    }

    // Clear the IRQs
    pub(super) async fn sub_clear_irq_status(&mut self, irq: u16) -> Result<(), RadioError<BUS>> {
        let mut buffer = [0x00u8, 0x00u8];
        buffer[0] = ((irq >> 8) & 0xFF) as u8;
        buffer[1] = (irq & 0xFF) as u8;
        self.brd_write_command(OpCode::ClrIrqStatus, &buffer).await?;
        Ok(())
    }

    // Utility functions

    fn timeout_1(timeout: u32) -> u8 {
        ((timeout >> 16) & 0xFF) as u8
    }
    fn timeout_2(timeout: u32) -> u8 {
        ((timeout >> 8) & 0xFF) as u8
    }
    fn timeout_3(timeout: u32) -> u8 {
        (timeout & 0xFF) as u8
    }

    // check this ???
    fn convert_u8_buffer_to_u32(buffer: &[u8; 4]) -> u32 {
        let b0 = buffer[0] as u32;
        let b1 = buffer[1] as u32;
        let b2 = buffer[2] as u32;
        let b3 = buffer[3] as u32;
        (b0 << 24) | (b1 << 16) | (b2 << 8) | b3
    }

    fn convert_freq_in_hz_to_pll_step(freq_in_hz: u32) -> u32 {
        // Get integer and fractional parts of the frequency computed with a PLL step scaled value
        let steps_int = freq_in_hz / SX126X_PLL_STEP_SCALED;
        let steps_frac = freq_in_hz - (steps_int * SX126X_PLL_STEP_SCALED);

        (steps_int << SX126X_PLL_STEP_SHIFT_AMOUNT)
            + (((steps_frac << SX126X_PLL_STEP_SHIFT_AMOUNT) + (SX126X_PLL_STEP_SCALED >> 1)) / SX126X_PLL_STEP_SCALED)
    }
}
